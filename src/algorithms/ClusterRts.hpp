#pragma once

#include <fcntl.h>
#include <ostream>
#include <unordered_map>
#include <vector>
#include "MemoryConfiguration.hpp"
#include "MetronomeException.hpp"
#include "OnlinePlanner.hpp"
#include "Planner.hpp"
#include "domains/SuccessorBundle.hpp"
#include "dynamic_priority_queue.hpp"
#include "easylogging++.h"
#include "experiment/Configuration.hpp"
#include "experiment/termination/TimeTerminationChecker.hpp"
#include "utils/Hash.hpp"
#include "utils/ObjectPool.hpp"
#include "utils/PriorityQueue.hpp"
#include "utils/TimeMeasurement.hpp"
#include "visualization/Visualizer.hpp"

namespace metronome {

template <typename Domain, typename TerminationChecker>
class ClusterRts final : public OnlinePlanner<Domain, TerminationChecker> {
 public:
  using State = typename Domain::State;
  using Action = typename Domain::Action;
  using Cost = typename Domain::Cost;
  using Planner = Planner<Domain>;
  using OnlinePlanner = OnlinePlanner<Domain, TerminationChecker>;
  using ActionBundle = typename Planner::ActionBundle;

  static constexpr std::size_t MAX_CLUSTER_COUNT = 100000;
  // Only used for visualizations
  static constexpr std::size_t NODE_ID_OFFSET = 100000;

  ClusterRts(const Domain &domain, const Configuration &configuration)
      : domain(domain),
        clusterNodeLimit(configuration.getLong(CLUSTER_NODE_LIMIT)),
        clusterDepthLimit(configuration.getLong(CLUSTER_DEPTH_LIMIT)),
        extractionCacheSize(configuration.getLong(EXTRACTION_CACHE_SIZE)),
        nodeWeight(configuration.getDouble(WEIGHT, 1.0)),
        openClusters(ClusterComparatorWeightedH(
            configuration.getDouble(CLUSTER_WEIGHT, nodeWeight))),
        tbaRouting(configuration.getBool(TBA_ROUTING, false)) {
    // Initialize hash table
    nodes.max_load_factor(1);
    nodes.reserve(Memory::NODE_LIMIT);
  }

  std::vector<ActionBundle> selectActions(
      const State &agentState,
      TerminationChecker &terminationChecker) override {
    ++iteration;
    OnlinePlanner::beginIteration();

    if (domain.isGoal(agentState)) {
      OnlinePlanner::incrementIdleIterationCount();
      // Goal is already reached
      return {};
    }

    // ---    Initialize    ---
    if (nodePool.empty()) {
      createInitialCluster(agentState);
      explore(agentState, terminationChecker);
    }

    const auto extractionStartTime = currentNanoTime();
    ActionBundle naiveAction;
    if (tbaRouting) {
      // Experimental action extraction
      naiveAction = extractNaiveAction(agentState);
    } else {
      // Default CRTS action extraction
      if (emptyCache) {
        emptyCache = false;
        cachedPath.clear();
      }
      if (cachedPath.size() <= cachedIndex ||
          cachedIndex > extractionCacheSize) {
        cachedPath = extractPath(agentState, extractionCacheSize);
        cachedIndex = 0;
      }
    }
    const auto extractionEndTime = currentNanoTime();

    const auto explorationStartTime = currentNanoTime();
    explore(agentState, terminationChecker);
    const auto explorationEndTime = currentNanoTime();

    if (!terminationChecker.reachedTermination()) {
      OnlinePlanner::incrementIdleIterationCount();
    }

#ifdef STREAM_GRAPH
    visualizeProgress(agentState, cachedPath);
#endif

    OnlinePlanner::recordAttribute("explorationTime",
                                   explorationEndTime - explorationStartTime);
    OnlinePlanner::recordAttribute("extractionTime",
                                   extractionEndTime - extractionStartTime);

    if (tbaRouting) {
      return {naiveAction};
    }

    if (cachedPath.empty()) throw MetronomeException("Cached path empty!");
    auto nextAction = cachedPath[cachedIndex++];
    return {nextAction};
  }

  void visualizeProgress(const State &agentState,
                         const std::vector<ActionBundle> &path) {
    visualizer.addNode(0, agentState.getX(), agentState.getY(), 0, 4, "source");

    std::size_t id = 0;
    for (const auto &actionBundle : path) {
      const auto &state = actionBundle.expectedTargetState;
      visualizer.addNode(++id, state.getX(), state.getY(), 0, 2, "path");
    }

    visualizer.post();

    visualizer.removeNode(0);
    while (id > 0) {
      visualizer.removeNode(id);
      --id;
    }
  }

 private:
  class Edge;
  class Cluster;

  class Node {
   public:
    Node(Node *parent, const State &state, Action action, Cost g, Cost h)
        : parent{parent}, state{state}, action{std::move(action)}, g{g}, h{h} {}

    Cost f() const { return g + h; }

    std::size_t hash() const { return state.hash(); }

    bool operator==(const Node &node) const { return state == node.state; }
    bool operator!=(const Node &node) const { return !(*this == node); }

    std::string toString() const {
      std::ostringstream stream;
      stream << "s: " << state << " g: " << g << " h: " << h << " f: " << f()
             << " a: " << action << " p: ";
      if (parent == nullptr) {
        stream << "None";
      } else {
        stream << parent->state;
      }
      return stream.str();
    }

    friend std::ostream &operator<<(std::ostream &os, const Node &node) {
      os << "state: " << node.state << " action: " << node.action
         << " g: " << node.g << " h: " << node.h;
      return os;
    }

    friend std::ostream &operator<<(std::ostream &os, const Node *node) {
      return operator<<(os, *node);
    }

    /** Parent node */
    Node *parent;
    /** Internal state */
    const State state;
    /** Action that led to the current node from the parent node */
    Action action;
    /** Cost from the root node */
    Cost g;
    /** Heuristic cost of the node */
    Cost h;

    Cluster *containingCluster = nullptr;
  };

  struct NodeComparatorWeightedF {
    NodeComparatorWeightedF(double weight) : w{weight} {}

    int operator()(const Node *lhs, const Node *rhs) const {
      if (lhs->g + lhs->h * w < rhs->g + rhs->h * w) return -1;
      if (lhs->g + lhs->h * w > rhs->g + rhs->h * w) return 1;
      if (lhs->g > rhs->g) return -1;
      if (lhs->g < rhs->g) return 1;
      return 0;
    }

    const double w;
  };

  struct NodeEquals {
    bool operator()(const Node *lhs, const Node *rhs) const {
      return lhs == rhs;
    }
  };

  class Edge {
   public:
    Edge(Node *predecessor, Action action, Cost actionCost)
        : predecessor{predecessor}, action{action}, actionCost{actionCost} {}

    Node *predecessor;
    const Action action;
    const Cost actionCost;
  };

  class ClusterEdge {
   public:
    Cost cost() const {
      return bestSourceFrontierNode->g + bestTargetFrontierNode->g;
    }

    std::vector<ActionBundle> actions() {
      if (actionsTowardsCluster.empty()) {
        auto coreToNodePath = extractCoreToNodePath(bestSourceFrontierNode);
        auto nodeToCorePath = extractNodeToCorePath(bestTargetFrontierNode);

        actionsTowardsCluster.reserve(coreToNodePath.size() +
                                      nodeToCorePath.size() + 1);

        actionsTowardsCluster.insert(
            std::end(actionsTowardsCluster),
            make_move_iterator(std::begin(coreToNodePath)),
            make_move_iterator(std::end(coreToNodePath)));

        ActionBundle bridgeActionBundle(connectingAction, connectionActionCost);
        bridgeActionBundle.expectedTargetState = bestTargetFrontierNode->state;
        bridgeActionBundle.label = "bridge - ";

        actionsTowardsCluster.push_back(std::move(bridgeActionBundle));

        actionsTowardsCluster.insert(
            std::end(actionsTowardsCluster),
            make_move_iterator(std::begin(nodeToCorePath)),
            make_move_iterator(std::end(nodeToCorePath)));
      }

      return actionsTowardsCluster;
    }

    std::vector<ActionBundle> inverseActions() {
      auto actions = this->actions();
      std::reverse(begin(actions), end(actions));
      for (auto &actionBundle : actions) {
        actionBundle.action = actionBundle.action.inverse();
      }

      return actions;
    }

    Cluster *cluster;
    Node *bestSourceFrontierNode;
    Node *bestTargetFrontierNode;
    Action connectingAction;
    Cost connectionActionCost;

    /**
     * Actions leading toward a cluster. Only populated on demand.
     * Should be reset when the best frontier node toward the cluster is
     * changed.
     */
    std::vector<ActionBundle> actionsTowardsCluster;
  };

  class Cluster {
   public:
    Cluster(double weight) : openList{{weight}} {}

    Node *coreNode = nullptr;
    /** Number of nodes claimed by this cluster not including open nodes. */
    std::size_t expandedNodeCount = 0;
    bool depleted = false;

    std::vector<ClusterEdge> reachableClusters;
    cserna::DynamicPriorityQueue<
        Node *,
        cserna::NonIntrusiveIndexFunction<Node *, Hash<Node>, NodeEquals>,
        NodeComparatorWeightedF,
        100,
        1000000>
        openList;
    std::unordered_map<State, Node *, typename metronome::Hash<State>>
        nodes;  // pre-allocate?

    std::size_t openListIndex = std::numeric_limits<std::size_t>::max();
    std::size_t goalOpenListIndex = std::numeric_limits<std::size_t>::max();

    // Readability
    std::string label = "";

    // Node properties
    Cost costToTarget;
    Cluster *parentCluster;
    ClusterEdge inEdge;
    std::size_t iteration = 0;
  };

  struct ClusterIndex {
    std::size_t &operator()(Cluster *cluster) const {
      return cluster->openListIndex;
    }
  };

  struct ClusterGoalIndex {
    std::size_t &operator()(Cluster *cluster) const {
      return cluster->goalOpenListIndex;
    }
  };

  struct ClusterHash {
    std::size_t operator()(const Cluster *cluster) const {
      return cluster->coreNode->hash();
    }
  };

  struct ClusterEquals {
    bool operator()(const Cluster *lhs, const Cluster *rhs) const {
      return lhs == rhs;
    }
  };

  struct ClusterComparatorWeightedH {
    ClusterComparatorWeightedH(double weight) : w{weight} {}

    int operator()(const Cluster *lhs, const Cluster *rhs) const {
      assert(!lhs->openList.empty() && !rhs->openList.empty() &&
             "Depleted cluster on open!");

      if (lhs->openList.top()->h * w < rhs->openList.top()->h * w) return -1;
      if (lhs->openList.top()->h * w > rhs->openList.top()->h * w) return 1;
      if (lhs->openList.top()->g < rhs->openList.top()->g) return -1;
      if (lhs->openList.top()->g > rhs->openList.top()->g) return 1;
      return 0;
    }

    const double w;
  };

  struct ClusterComparatorCoreH {
    int operator()(const Cluster *lhs, const Cluster *rhs) const {
      if (lhs->coreNode->h < rhs->coreNode->h) return -1;
      if (lhs->coreNode->h > rhs->coreNode->h) return 1;
      return 0;
    }
  };

  struct ClusterComparatorGoalCost {
    int operator()(const Cluster *lhs, const Cluster *rhs) const {
      if (lhs->costToTarget < rhs->costToTarget) return -1;
      if (lhs->costToTarget > rhs->costToTarget) return 1;
      return 0;
    }
  };

  void createInitialCluster(const State &initialState) {
    Planner::incrementGeneratedNodeCount();

    auto initialCluster = clusterPool.construct(nodeWeight);
    Node *&initialNode = initialCluster->nodes[initialState];

    initialNode = nodePool.construct(Node{
        nullptr, initialState, Action(), 0, domain.heuristic(initialState)});

    initialCluster->coreNode = initialNode;
    initialCluster->openList.push(initialNode);
    initialCluster->label = "1 - start";

    openClusters.push(initialCluster);
  }

  void explore(const State &, TerminationChecker &terminationChecker) {
    while (!terminationChecker.reachedTermination() && !openClusters.empty()) {
      auto cluster = openClusters.top();

      if (goalNode != nullptr) return;

      if (cluster->openList.empty()) {
        throw MetronomeException("Empty cluster on open!");
      }

      expandCluster(cluster);

      terminationChecker.notifyExpansion();
    }
  }

  bool expandCluster(Cluster *sourceCluster) {
    //    LOG(INFO) << "Expanding cluster: " <<
    //    clusterPool.index(sourceCluster);

    if (sourceCluster->depleted) {
      //      LOG(INFO) << "Expanding depleted cluster: "
      //                << clusterPool.index(sourceCluster);
      throw MetronomeException(
          "Trying to expand a depleted cluster. Such clusters should not be on "
          "open.");
    }

    auto currentNode = sourceCluster->openList.pop();
    // The local node list should only contain open nodes
    sourceCluster->nodes.erase(currentNode->state);

    assert(!sourceCluster->openList.contains(currentNode));
    //    assert(currentNode->containingCluster == sourceCluster && "Containing
    //    cluster: " + std::to_string(currentNode->containingCluster));

    if (nodes[currentNode->state] != nullptr) {
      // Remove local node
      nodePool.destruct(currentNode);

      manageCluster(sourceCluster);
      return false;
    }

    nodes[currentNode->state] = currentNode;

    bool spawnNewCore =
        sourceCluster->expandedNodeCount >= clusterNodeLimit ||
        currentNode->g / domain.getActionDuration() >= clusterDepthLimit;

    if (spawnNewCore) {
      // Reassign local node to a new cluster
      auto spawnedCluster = clusterPool.construct(nodeWeight);
      spawnedCluster->label = std::to_string(clusterPool.index(spawnedCluster));
      spawnedCluster->coreNode = currentNode;
      //      LOG(INFO) << "Creating cluster: " <<
      //      clusterPool.index(spawnedCluster);

      currentNode->containingCluster = spawnedCluster;
      // Reset values
      currentNode->g = 0;
      //      currentNode->parent = nullptr; // Keep the parent for debug
      //      currentNode->action = Action();
    } else {
      currentNode->containingCluster = sourceCluster;
    }

    ++(currentNode->containingCluster->expandedNodeCount);

    expandNode(currentNode);

    manageCluster(sourceCluster);

    if (spawnNewCore) manageCluster(currentNode->containingCluster);

    // Visualization

    std::size_t id = nodePool.index(currentNode) + NODE_ID_OFFSET;

    auto clusterLabel =
        "cluster:" +
        std::to_string(clusterPool.index(currentNode->containingCluster));

    visualizer.addNode(id,
                       currentNode->state.getX(),
                       currentNode->state.getY(),
                       0,
                       1,
                       clusterLabel);

    if (currentNode->parent != nullptr) {
      visualizer.addEdge(id,
                         nodePool.index(currentNode->parent) + NODE_ID_OFFSET,
                         id,
                         clusterLabel);
    }

    return true;
  }

  void expandNode(Node *sourceNode) {
    Planner::incrementExpandedNodeCount();
    auto containingCluster = sourceNode->containingCluster;

    if (domain.isGoal(sourceNode->state)) {
      LOG(INFO) << "Goal was expanded!";
      emptyCache = true;
      this->goalFound();
      goalNode = sourceNode;
    }

    for (auto successor : domain.successors(sourceNode->state)) {
      auto successorState = successor.state;

      Node *globalSuccessorNode = nodes[successorState];
      Node *&successorNode = containingCluster->nodes[successorState];

      if (globalSuccessorNode != nullptr) {
        // This node was already claimed

        if (globalSuccessorNode->containingCluster == containingCluster) {
          // It was claimed by the current cluster. No action is necessary
          continue;
        }

        // This is a neighboring cluster
        connectClusters(sourceNode,
                        globalSuccessorNode,
                        successor.action,
                        successor.actionCost);

        if (successorNode != nullptr) {
          //          LOG(INFO) << "Expanding successors from cluster: "
          //                    << clusterPool.index(containingCluster)
          //                    << " removing node: " <<
          //                    nodePool.index(successorNode)
          //                    << " as it was already claimed by: "
          //                    << clusterPool.index(
          //                           globalSuccessorNode->containingCluster);

          Node *successorRef = successorNode;
          containingCluster->openList.remove(successorNode);
          containingCluster->nodes.erase(successorNode->state);
          nodePool.destruct(successorRef);
        }

        // else: this node is not on the open list of this cluster, no cleanup
        // is necessary.

        continue;
      }

      if (successorNode == nullptr) {
        Planner::incrementGeneratedNodeCount();
        successorNode = nodePool.construct(sourceNode,
                                           successor.state,
                                           successor.action,
                                           std::numeric_limits<Cost>::max(),
                                           domain.heuristic(successor.state));
      }

      // Skip if we got back to the parent
      if (sourceNode->parent != nullptr &&
          successorState == sourceNode->parent->state) {
        continue;
      }

      Cost successorGValueFromCurrent{sourceNode->g + successor.actionCost};
      if (successorNode->g > successorGValueFromCurrent) {
        successorNode->g = successorGValueFromCurrent;
        successorNode->parent = sourceNode;
        successorNode->action = successor.action;

        containingCluster->openList.insertOrUpdate(successorNode);
      }
    }
  }

  void manageCluster(Cluster *cluster) {
    bool open = openClusters.contains(cluster);

    if (cluster->openList.empty()) {
      cluster->depleted = true;

      if (open) openClusters.remove(cluster);
    } else if (open) {
      openClusters.update(cluster);
    } else {
      openClusters.push(cluster);
    }
  }

  void connectClusters(Node *sourceNode,
                       Node *targetNode,
                       Action action,
                       Cost cost) {
    const auto sourceDangling = nodes[sourceNode->state] == nullptr;
    const auto targetDangling = nodes[targetNode->state] == nullptr;

    if (sourceDangling || targetDangling) {
      throw MetronomeException("can't connect dangling nodes");
    }

    bool sourceSuccess =
        connectClustersDirected(sourceNode, targetNode, action, cost);
    bool targetSuccess =
        connectClustersDirected(targetNode, sourceNode, action.inverse(), cost);

    if (sourceSuccess != targetSuccess)
      throw MetronomeException("Inconsistent edges");
  }

  bool connectClustersDirected(Node *sourceNode,
                               Node *targetNode,
                               Action action,
                               Cost cost) {
    const auto sourceCluster = sourceNode->containingCluster;
    const auto targetCluster = targetNode->containingCluster;
    const Cost connectionCost = sourceNode->g + targetNode->g;

    bool existingConnectionFound = false;

    for (ClusterEdge &existingEdge : sourceCluster->reachableClusters) {
      if (existingEdge.cluster == targetCluster) {
        const auto existingConnectionCost = existingEdge.cost();

        if (connectionCost < existingConnectionCost) {
          // Keep the better connection
          existingEdge.bestSourceFrontierNode = sourceNode;
          existingEdge.bestTargetFrontierNode = targetNode;
          existingEdge.connectingAction = action;
          existingEdge.connectionActionCost = cost;
          existingEdge.actionsTowardsCluster.clear();
          existingConnectionFound = true;
        }

        break;
      }
    }

    if (!existingConnectionFound) {
      ClusterEdge forwardEdge;
      forwardEdge.cluster = targetCluster;
      forwardEdge.bestSourceFrontierNode = sourceNode;
      forwardEdge.bestTargetFrontierNode = targetNode;
      forwardEdge.connectingAction = action;
      forwardEdge.connectionActionCost = cost;
      sourceCluster->reachableClusters.push_back(std::move(forwardEdge));
    }

    return existingConnectionFound;
  }

  std::vector<ActionBundle> join(std::vector<ActionBundle> &&segments...) {
    std::size_t totalSize;
    for (const auto &segment : segments) {
      totalSize += segment.size();
    }

    std::vector<ActionBundle> joinedSegments;
    joinedSegments.reserve(totalSize);

    for (auto &segment : segments) {
      joinedSegments.insert(std::end(joinedSegments),
                            make_move_iterator(std::begin(segment)),
                            make_move_iterator(std::end(segment)));
    }

    return joinedSegments;
  }

  std::vector<ActionBundle> extractPath(const State &agentState,
                                        const std::size_t length) {
    // LOG(INFO) << "Extracting path from: " << agentState;
    Node *agentNode = nodes[agentState];
    if (agentNode == nullptr) {
      throw MetronomeException(
          "Target not available. Insufficient time to expand path?");
    }

    Cluster *agentCluster = agentNode->containingCluster;

    Cluster *targetCluster = nullptr;
    Node *targetNode = nullptr;

    // Initialize the target cluster and target node
    if (goalNode != nullptr) {
      targetCluster = goalNode->containingCluster;
      targetNode = goalNode;
      // LOG(INFO) << "\tto the goal: " << goalNode;
    } else {
      if (openClusters.empty())
        throw MetronomeException(
            "Goal is not "
            "reachable");
      targetCluster = openClusters.top();
      targetNode = targetCluster->openList.top()->parent;
      // LOG(INFO) << "\tto: " << targetNode;
    }

    if (agentNode == targetNode) {
      return {
          ActionBundle(domain.getIdentityAction(), domain.getActionDuration())};
    }

    populateAgentToClusterCosts(agentCluster, targetCluster);

    auto fromLastCorePath = extractCoreToNodePath(targetNode);

    if (agentNode->containingCluster == targetNode->containingCluster) {
      for (auto it = std::begin(fromLastCorePath);
           it != std::end(fromLastCorePath);
           ++it) {
        if (it->expectedTargetState == agentState) {
          // The source and the target are in the same cluster and the
          // core-target path contains the source thus the agent can directly
          // go to the target

          // LOG(INFO) << "Last segment cut " << *it;

          return {it + 1, std::end(fromLastCorePath)};
        }
      }
    }

    auto skeletonPath = extractSkeletonPath(agentCluster, targetCluster);
    auto toFirstCorePath = extractNodeToCorePath(agentNode);
    auto interClusterPath = extractInterClusterPath(skeletonPath, length);

    // Look for shortcuts - this is not an optional step

    for (auto it = std::begin(interClusterPath);
         it != std::end(interClusterPath);
         ++it) {
      if (it->expectedTargetState == agentState) {
        // LOG(INFO) << "CUT " << *it;
        // LOG(INFO) << toFirstCorePath;

        toFirstCorePath.clear();
        decltype(interClusterPath)(it, std::end(interClusterPath))
            .swap(interClusterPath);
        break;
      }
    }

    // Join segments
    std::vector<ActionBundle> path;
    path.reserve(toFirstCorePath.size() + interClusterPath.size() +
                 fromLastCorePath.size());

    path.insert(std::end(path),
                make_move_iterator(std::begin(toFirstCorePath)),
                make_move_iterator(std::end(toFirstCorePath)));

    path.insert(std::end(path),
                make_move_iterator(std::begin(interClusterPath)),
                make_move_iterator(std::end(interClusterPath)));

    path.insert(std::end(path),
                make_move_iterator(std::begin(fromLastCorePath)),
                make_move_iterator(std::end(fromLastCorePath)));

    // Find abstract path to goal
    // 1. Find path to containing region center
    // 2. Find abstract path to goal via region centers
    // 3. Find last segment from region center to target node
    // If the path partially overlaps with the current path stitch

    return path;
  }

  static std::vector<ActionBundle> extractCoreToNodePath(Node *targetNode) {
    std::vector<ActionBundle> coreToNodePath;

    auto currentNode = targetNode;
    const auto coreNode = targetNode->containingCluster->coreNode;

    while (currentNode != coreNode) {
      //      LOG(INFO) << "--- n: " << currentNode
      //                << " a: " << currentNode->action.inverse();

      ActionBundle actionBundle(currentNode->action,
                                currentNode->g - currentNode->parent->g);
      actionBundle.expectedTargetState = currentNode->state;
      coreToNodePath.push_back(std::move(actionBundle));

      currentNode = currentNode->parent;
    }

    std::reverse(std::begin(coreToNodePath), std::end(coreToNodePath));

    return coreToNodePath;
  }

  std::vector<ActionBundle> extractInterClusterPath(
      std::vector<ClusterEdge> &skeletonPath, const std::size_t length) const {
    std::vector<ActionBundle> interClusterActions;

    // LOG(INFO) << "InterCluster Path";
    std::optional<std::size_t> firstSegmentSize;

    for (auto &skeletonPathSegment : skeletonPath) {
      /*LOG(INFO) << "  Segment:"
                << skeletonPathSegment.bestSourceFrontierNode->containingCluster
                       ->label
                << " >>> "
                << skeletonPathSegment.bestTargetFrontierNode->containingCluster
                       ->label;*/
      auto segmentActions = skeletonPathSegment.inverseActions();

      // Debug info
      for (auto &actionBundle : segmentActions) {
        Cluster *sourceCluster =
            skeletonPathSegment.bestSourceFrontierNode->containingCluster;
        Cluster *targetCluster =
            skeletonPathSegment.bestTargetFrontierNode->containingCluster;

        const auto sourceClusterId = clusterPool.index(sourceCluster);
        const auto targetClusterId = clusterPool.index(targetCluster);

        actionBundle.label += std::to_string(targetClusterId) + "->" +
                              std::to_string(sourceClusterId);
      }

      // NOTE: we use the assumption that the cost is symmetric and the
      // actions are reversible.
      //      std::reverse(begin(segmentActions), end(segmentActions));
      //      for (auto& actionBundle : segmentActions) {
      //        actionBundle.action = actionBundle.action.inverse();
      //      }

      if (!firstSegmentSize.has_value()) {
        // Initialize first segment size
        firstSegmentSize = segmentActions.size();
      }

      interClusterActions.insert(std::end(interClusterActions),
                                 make_move_iterator(std::begin(segmentActions)),
                                 make_move_iterator(std::end(segmentActions)));

      if (interClusterActions.size() - firstSegmentSize.value() > length) {
        // The first segment can be lost during optimization
        // The returned path has to be at least length size after optimization

        break;
      }
    }

    // LOG(INFO) << "InterCluster Path END";

    return interClusterActions;
  }

  static std::vector<ActionBundle> extractNodeToCorePath(Node *sourceNode) {
    std::vector<ActionBundle> nodeToCorePath;

    auto currentNode = sourceNode;
    const auto coreNode = sourceNode->containingCluster->coreNode;

    while (currentNode != coreNode) {
      // Note that the stored actions are towards the frontier
      // thus they have to be inverted
      //      LOG(INFO) << "... n: " << currentNode
      //                << " a: " << currentNode->action.inverse();

      ActionBundle actionBundle(currentNode->action.inverse(),
                                currentNode->g - currentNode->parent->g);
      actionBundle.expectedTargetState = currentNode->parent->state;
      nodeToCorePath.push_back(std::move(actionBundle));

      currentNode = currentNode->parent;
    }

    return nodeToCorePath;
  }

  /**
   * Extract the skeleton path from the agentCluster to the
   * targetCluster following the cluster-parent pointers. The cluster-parent
   * pointers mark the forward path as the cluster level search was initiated
   * at the targetCluster.
   */
  std::vector<ClusterEdge> extractSkeletonPath(const Cluster *agentCluster,
                                               Cluster *targetCluster) const {
    auto currentCluster = agentCluster;
    std::vector<ClusterEdge> skeletonPath;

    while (currentCluster != targetCluster) {
      skeletonPath.push_back(currentCluster->inEdge);
      currentCluster = currentCluster->parentCluster;
    }

    return skeletonPath;
  }

  /**
   * Run Dijkstra on the skeleton network to calculate the agent-cluster to
   * target-cluster distance.
   *
   * Note: Not all clusters will be touched nor updated by the search.
   *
   * @param agentCluster - Cluster that contains the agent.
   * @param targetCluster - Cluster that terminates the search when reached.
   */
  void populateAgentToClusterCosts(const Cluster *agentCluster,
                                   Cluster *targetCluster) {
    targetCluster->costToTarget = 0;
    targetCluster->iteration = iteration;

    goalSearchClustersOpen.clear();
    goalSearchClustersOpen.push(targetCluster);

    // TODO add projection
    while (goalSearchClustersOpen.top() != agentCluster) {
      auto sourceCluster = goalSearchClustersOpen.pop();

      for (ClusterEdge edge : sourceCluster->reachableClusters) {
        const auto predecessorCluster =
            edge.bestTargetFrontierNode->containingCluster;

        const auto newCost = sourceCluster->costToTarget + edge.cost();

        if (predecessorCluster->iteration != iteration) {
          predecessorCluster->iteration = iteration;
          predecessorCluster->parentCluster = sourceCluster;
          predecessorCluster->costToTarget = newCost;
          predecessorCluster->inEdge = edge;

          goalSearchClustersOpen.push(predecessorCluster);
        } else if (predecessorCluster->costToTarget > newCost) {
          predecessorCluster->parentCluster = sourceCluster;
          predecessorCluster->costToTarget = newCost;
          predecessorCluster->inEdge = edge;

          goalSearchClustersOpen.update(predecessorCluster);
        }
      }
    }
  }

  /**
   * Extract TBA* like path. This is not part of the core CRTS algorithm.
   *
   * @param targetNode
   * @param sourceNode
   * @return
   */
  ActionBundle extractNaiveAction(const State &agentState) {
    Node *agentNode = nodes[agentState];
    if (agentNode == nullptr) {
      throw MetronomeException(
          "Target not available. Insufficient time to expand path?");
    }

    Cluster *agentCluster = agentNode->containingCluster;

    Cluster *targetCluster = nullptr;
    Node *targetNode = nullptr;

    // Initialize the target cluster and target node
    if (goalNode != nullptr) {
      targetCluster = goalNode->containingCluster;
      targetNode = goalNode;
      // LOG(INFO) << "\tto the goal: " << goalNode;
    } else {
      assert(!openClusters.empty());
      targetCluster = openClusters.top();
      targetNode = targetCluster->openList.top()->parent;
      // LOG(INFO) << "\tto: " << targetNode;
    }

    if (agentNode == targetNode) {
      return {
          ActionBundle(domain.getIdentityAction(), domain.getActionDuration())};
    }

    std::vector<ActionBundle> actionBundles;
    auto currentNode = targetNode;

    while (currentNode->parent != nullptr && currentNode != agentNode) {
      ActionBundle actionBundle(currentNode->action,
                                currentNode->g - currentNode->parent->g);
      actionBundle.expectedTargetState = currentNode->state;
      actionBundles.push_back(std::move(actionBundle));

      currentNode = currentNode->parent;
    }

    if (currentNode != agentNode) {
      const Action action = agentNode->action.inverse();
      const Cost cost = agentNode->g - agentNode->parent->g;
      const State expectedState = agentNode->parent->state;

      ActionBundle actionBundle(action, cost);
      actionBundle.expectedTargetState = expectedState;
      actionBundle.label = "backtrack";
      return actionBundle;
    }

    //    std::reverse(actionBundles.begin(), actionBundles.end());
    return actionBundles.back();
  }

  const Domain &domain;
  const std::size_t clusterNodeLimit;
  const Cost clusterDepthLimit;
  const std::size_t extractionCacheSize;
  const double nodeWeight;
  const bool tbaRouting;
  
  cserna::DynamicPriorityQueue<Cluster *,
                               ClusterIndex,
                               ClusterComparatorWeightedH,
                               MAX_CLUSTER_COUNT,
                               MAX_CLUSTER_COUNT>
      openClusters;

  std::unordered_map<State, Node *, typename metronome::Hash<State>> nodes;
  ObjectPool<Node, Memory::NODE_LIMIT> nodePool;
  ObjectPool<Cluster, Memory::NODE_LIMIT> clusterPool;

  cserna::DynamicPriorityQueue<Cluster *,
                               ClusterGoalIndex,
                               ClusterComparatorGoalCost,
                               MAX_CLUSTER_COUNT,
                               MAX_CLUSTER_COUNT>
      goalSearchClustersOpen;
  std::size_t iteration = 0;
  std::vector<ActionBundle> cachedPath;
  std::size_t cachedIndex = 0;

  Node *goalNode = nullptr;
  bool emptyCache = false;

  Visualizer visualizer;
};

}  // namespace metronome
