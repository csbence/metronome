#pragma once

#include "experiment/termination/TimeTerminationChecker.hpp"

#include <fcntl.h>
#include <MemoryConfiguration.hpp>
#include <domains/SuccessorBundle.hpp>
#include <unordered_map>
#include <vector>
#include "MetronomeException.hpp"
#include "OnlinePlanner.hpp"
#include "Planner.hpp"
#include "dynamic_priority_queue.hpp"
#include "easylogging++.h"
#include "experiment/Configuration.hpp"
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
  using ActionBundle =
      typename OnlinePlanner<Domain, TerminationChecker>::ActionBundle;

  static constexpr std::size_t CLUSTER_NODE_LIMIT = 20;
  static constexpr std::size_t CLUSTER_G_RADIUS = 10000;
  static constexpr std::size_t MAX_CLUSTER_COUNT = 1000;

  ClusterRts(const Domain& domain, const Configuration&) : domain{domain} {
    // Initialize hash table
    nodes.max_load_factor(1);
    nodes.reserve(Memory::NODE_LIMIT);
  }

  std::vector<ActionBundle> selectActions(
      const State& agentState,
      TerminationChecker& terminationChecker) override {
    ++iteration;
    if (domain.isGoal(agentState)) {
      // Goal is already reached
      return {};
    }

    // ---    Initialize    ---
    if (clusterPool.empty()) createInitialCluster(agentState);

    explore(agentState, terminationChecker);

    // ---      Learn      ---
    // ???

    visualizer.post();

    return extractPath(agentState);
  }

 private:
  class Edge;
  class Cluster;

  class Node {
   public:
    Node(Node* parent, const State& state, Action action, Cost g, Cost h)
        : parent{parent}, state{state}, action{std::move(action)}, g{g}, h{h} {}

    Cost f() const { return g + h; }

    unsigned long hash() const { return state.hash(); }

    bool operator==(const Node& node) const { return state == node.state; }

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

    /** Parent node */
    Node* parent;
    /** Internal state */
    const State state;
    /** Action that led to the current node from the parent node */
    Action action;
    /** Cost from the root node */
    Cost g;
    /** Heuristic cost of the node */
    Cost h;
    /** List of all the predecessors that were discovered in the current
     * exploration phase. */
    std::vector<Edge> predecessors;

    Cluster* containingCluster = nullptr;
  };

  struct NodeComparatorF {
    int operator()(const Node* lhs, const Node* rhs) const {
      if (lhs->f() < rhs->f()) return -1;
      if (lhs->f() > rhs->f()) return 1;
      if (lhs->g > rhs->g) return -1;
      if (lhs->g < rhs->g) return 1;
      return 0;
    }
  };

  struct NodeEquals {
    bool operator()(const Node* lhs, const Node* rhs) const {
      return lhs == rhs;
    }
  };

  class Edge {
   public:
    Edge(Node* predecessor, Action action, Cost actionCost)
        : predecessor{predecessor}, action{action}, actionCost{actionCost} {}

    Node* predecessor;
    const Action action;
    const Cost actionCost;
  };

  class ClusterEdge {
   public:
    Cost cost() const {
      return bestSourceFrontierNode->g + bestTargetFrontierNode->g;
    }

    Cluster* cluster;
    Node* bestSourceFrontierNode;
    Node* bestTargetFrontierNode;

    /**
     * Actions leading toward a cluster. Only populated on demand.
     * Should be reset when the best frontier node toward the cluster is
     * changed.
     */
    std::vector<Action> actionsTowardsCluster;
  };

  class Cluster {
   public:
    Node* coreNode = nullptr;
    std::size_t nodeCount = 0;
    bool depleted = false;

    Node* bestHNode = nullptr;

    std::vector<ClusterEdge> reachableClusters;
    cserna::DynamicPriorityQueue<
        Node*,
        cserna::NonIntrusiveIndexFunction<Node*, Hash<Node>, NodeEquals>,
        NodeComparatorF,
        CLUSTER_NODE_LIMIT * 10,
        CLUSTER_NODE_LIMIT * 4>
        openList;
    std::unordered_map<State, Node*, typename metronome::Hash<State>>
        nodes;  // pre-allocate?

    std::size_t openListIndex = std::numeric_limits<std::size_t>::max();
    std::size_t goalOpenListIndex = std::numeric_limits<std::size_t>::max();

    // Node properties
    Cost goalDistance;
    Cluster* parentCluster;
    std::size_t iteration = 0;
  };

  struct ClusterIndex {
    std::size_t& operator()(Cluster* cluster) const {
      return cluster->openListIndex;
    }
  };
  
  struct ClusterGoalIndex {
    std::size_t& operator()(Cluster* cluster) const {
      return cluster->goalOpenListIndex;
    }
  };

  struct ClusterHash {
    std::size_t operator()(const Cluster* cluster) const {
      return cluster->coreNode->hash();
    }
  };

  struct ClusterEquals {
    bool operator()(const Cluster* lhs, const Cluster* rhs) const {
      return lhs == rhs;
    }
  };

  struct ClusterComparatorH {
    int operator()(const Cluster* lhs, const Cluster* rhs) const {
      if (lhs->bestHNode->h < rhs->bestHNode->h) return -1;
      if (lhs->bestHNode->h > rhs->bestHNode->h) return 1;
      if (lhs->bestHNode->g < rhs->bestHNode->g) return -1;
      if (lhs->bestHNode->g > rhs->bestHNode->g) return 1;
      return 0;
    }
  };

  struct ClusterComparatorCoreH {
    int operator()(const Cluster* lhs, const Cluster* rhs) const {
      if (lhs->coreNode->h < rhs->coreNode->h) return -1;
      if (lhs->coreNode->h > rhs->coreNode->h) return 1;
      return 0;
    }
  };

  struct ClusterComparatorGoalCost {
    int operator()(const Cluster* lhs, const Cluster* rhs) const {
      if (lhs->goalDistance < rhs->goalDistance) return -1;
      if (lhs->goalDistance > rhs->goalDistance) return 1;
      return 0;
    }
  };

  void createInitialCluster(const State& initialState) {
    Planner::incrementGeneratedNodeCount();

    auto initialCluster = clusterPool.construct();
    Node*& initialNode = initialCluster->nodes[initialState];

    initialNode = nodePool.construct(Node{
        nullptr, initialState, Action(), 0, domain.heuristic(initialState)});

    initialCluster->coreNode = initialNode;
    initialCluster->openList.push(initialNode);
    // Do not add to the local nodes set. Will be added when expanded.

    openClusters.push(initialCluster);
  }

  void explore(const State& startState,
               TerminationChecker& terminationChecker) {
    while (!terminationChecker.reachedTermination() && !openClusters.empty()) {
      auto cluster = openClusters.top();

      // todo check if the goal state was expanded

      bool expanded = expandCluster(cluster);

      terminationChecker.notifyExpansion();
    }
  }

  bool expandCluster(Cluster* sourceCluster) {
    LOG(INFO) << "Expanding cluster: " << clusterPool.index(sourceCluster);

    if (sourceCluster->depleted) {
      LOG(ERROR) << "Expanding depleted cluster: "
                 << clusterPool.index(sourceCluster);
      throw MetronomeException(
          "Trying to expand a depleted cluster. Such clusters should not be on "
          "open.");
    }

    if (sourceCluster->openList.empty()) {
      LOG(ERROR) << "Cluster depleted: " << clusterPool.index(sourceCluster);
      sourceCluster->depleted = true;

      openClusters.remove(sourceCluster);

      return false;
    }

    // A new cluster core should be spawned in the following cases:
    // * Cluster node limit is reached
    // * Node is beyond a threshold cost (g) from the cluster core
    // * Node is beyond a threshold distance (d) from the cluster core ->
    // todo implement d

    auto currentNode = sourceCluster->openList.pop();
    assert(!sourceCluster->openList.contains(currentNode));
    //    assert(currentNode->containingCluster == sourceCluster && "Containing
    //    cluster: " + std::to_string(currentNode->containingCluster));

    if (nodes[currentNode->state] != nullptr) {
      // Another cluster already claimed this node
      LOG(INFO) << "Expanding cluster: " << clusterPool.index(sourceCluster)
                << " removing node: " << nodePool.index(currentNode)
                << " as it was already claimed by: "
                << clusterPool.index(
                       nodes[currentNode->state]->containingCluster);

      // Remove local node
      sourceCluster->nodes.erase(currentNode->state);
      nodePool.destruct(currentNode);

      return false;
    }

    nodes[currentNode->state] = currentNode;

    bool spawnNewCore = sourceCluster->nodeCount >= CLUSTER_NODE_LIMIT ||
                        currentNode->g >= CLUSTER_G_RADIUS;

    if (spawnNewCore) {
      auto spawnedCluster = clusterPool.construct();
      spawnedCluster->coreNode = currentNode;
      LOG(INFO) << "Creating cluster: " << clusterPool.index(spawnedCluster);
      openClusters.push(spawnedCluster);

      currentNode->containingCluster = spawnedCluster;
    } else {
      currentNode->containingCluster = sourceCluster;
    }

    ++(currentNode->containingCluster->nodeCount);
    expandNode(currentNode);

    std::size_t id = nodePool.index(currentNode);
    visualizer.addNode(
        id, currentNode->state.getX(), currentNode->state.getY());
    if (currentNode->parent != nullptr) {
      visualizer.addEdge(id,
                         nodePool.index(currentNode->parent),
                         id,
                         "cluster:" + std::to_string(clusterPool.index(
                                          currentNode->containingCluster)));
    }

    return true;
  }

  void expandNode(Node* sourceNode) {
    Planner::incrementExpandedNodeCount();
    auto containingCluster = sourceNode->containingCluster;

    if (domain.isGoal(sourceNode->state)) {
      LOG(INFO) << "Goal was expanded!";
      goalNode = sourceNode;
    }

    for (auto successor : domain.successors(sourceNode->state)) {
      auto successorState = successor.state;

      Node* globalSuccessorNode = nodes[successorState];
      Node*& successorNode = containingCluster->nodes[successorState];

      if (globalSuccessorNode != nullptr) {
        // This node was already claimed

        if (globalSuccessorNode->containingCluster == containingCluster) {
          // It was claimed by the current cluster. No action is necessary
          continue;
        }

        // This is a neighboring cluster
        connectClusters(globalSuccessorNode, sourceNode);

        if (successorNode != nullptr) {
          LOG(INFO) << "Expanding successors from cluster: "
                    << clusterPool.index(containingCluster)
                    << " removing node: " << nodePool.index(successorNode)
                    << " as it was already claimed by: "
                    << clusterPool.index(
                           globalSuccessorNode->containingCluster);

          containingCluster->openList.remove(successorNode);
          containingCluster->nodes.erase(successorNode->state);
          nodePool.destruct(successorNode);
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

      // Add the current state as the predecessor of the child state
      successorNode->predecessors.emplace_back(
          sourceNode, successor.action, successor.actionCost);

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

  void connectClusters(Node* lhs, Node* rhs) {
    assert(nodes[lhs->state] != nullptr && nodes[rhs->state] != nullptr &&
           "can't connect dangling nodes");
    const auto lhsCluster = lhs->containingCluster;
    const auto rhsCluster = rhs->containingCluster;
    const Cost connectionCost = lhs->g + rhs->g;

    bool lhsConnectionFound = false;
    bool rhsConnectionFound = false;

    // fixme simplify command flow

    for (ClusterEdge& existingEdge : lhsCluster->reachableClusters) {
      if (existingEdge.cluster == rhsCluster) {
        const auto existingConnectionCost = existingEdge.cost();

        if (connectionCost < existingConnectionCost) {
          // Keep the better connection
          existingEdge.bestSourceFrontierNode = lhs;
          existingEdge.bestTargetFrontierNode = rhs;
          lhsConnectionFound = true;
        }

        break;
      }
    }

    for (ClusterEdge& existingEdge : rhsCluster->reachableClusters) {
      if (existingEdge.cluster == lhsCluster) {
        const auto existingConnectionCost = existingEdge.cost();

        if (connectionCost < existingConnectionCost) {
          // Keep the better connection
          existingEdge.bestSourceFrontierNode = rhs;
          existingEdge.bestTargetFrontierNode = lhs;
          rhsConnectionFound = true;
        }

        break;
      }
    }

    assert(lhsConnectionFound == rhsConnectionFound && "not synced edges");

    if (!lhsConnectionFound) {
      ClusterEdge lhsEdge;
      lhsEdge.cluster = rhsCluster;
      lhsEdge.bestSourceFrontierNode = lhs;
      lhsEdge.bestTargetFrontierNode = rhs;
      lhsCluster->reachableClusters.push_back(std::move(lhsEdge));

      ClusterEdge rhsEdge;
      rhsEdge.cluster = lhsCluster;
      rhsEdge.bestSourceFrontierNode = rhs;
      rhsEdge.bestTargetFrontierNode = lhs;
      rhsCluster->reachableClusters.push_back(std::move(rhsEdge));
    }

    LOG(INFO) << (lhsConnectionFound ? "Connection updated"
                                     : "new connection found")
              << " between cluster: " << clusterPool.index(lhsCluster)
              << " and cluster: " << clusterPool.index(rhsCluster);
  }

  std::vector<ActionBundle> extractPath(const State& agentState) {
    goalSearchClustersOpen.clear();

    // Initialize path search
    if (goalNode != nullptr) {
      goalSearchClustersOpen.push(goalNode->containingCluster);
    } else {
      assert(!openClusters.empty());
      auto top = openClusters.top();
      goalSearchClustersOpen.push(top);
    }

    goalSearchClustersOpen.top()->goalDistance = 0;
    goalSearchClustersOpen.top()->iteration = iteration;

    auto agentNode = nodes[agentState];
    assert(agentNode != nullptr);
    auto agentCluster = agentNode->containingCluster;

    if (agentCluster == goalSearchClustersOpen.top()) {
      // The agent is currently in the target cluster
      return {};
    }

    while (goalSearchClustersOpen.top() != agentCluster) {
      auto sourceCluster = goalSearchClustersOpen.pop();

      for (ClusterEdge& edge : sourceCluster->reachableClusters) {
        const auto targetCluster =
            edge.bestTargetFrontierNode->containingCluster;

        const auto newCost = sourceCluster->goalDistance + edge.cost();
        
        if (targetCluster->iteration != iteration) {
          targetCluster->iteration = iteration;
          targetCluster->parentCluster = sourceCluster;
          targetCluster->goalDistance = newCost;
          
          goalSearchClustersOpen.push(targetCluster);
        } else if (targetCluster->goalDistance > newCost) {
          targetCluster->parentCluster = sourceCluster;
          targetCluster->goalDistance = newCost;
          
          goalSearchClustersOpen.update(targetCluster);
        }
      }
    }
    
    // Build inter-cluster path

    // Find abstract path to goal
    // 1. Find path to containing region center
    // 2. Find abstract path to goal via region centers
    // 3. Find last segment from region center to target node
    // If the path partially overlaps with the current path stitch

    return {};
  }

#ifdef VISUALIZER

#endif

  const Domain& domain;
  cserna::DynamicPriorityQueue<Cluster*,
                               ClusterIndex,
                               ClusterComparatorCoreH,
                               MAX_CLUSTER_COUNT,
                               MAX_CLUSTER_COUNT>
      openClusters;

  std::unordered_map<State, Node*, typename metronome::Hash<State>> nodes;
  ObjectPool<Node, Memory::NODE_LIMIT> nodePool;
  ObjectPool<Cluster, Memory::NODE_LIMIT> clusterPool;

  cserna::DynamicPriorityQueue<Cluster*,
                               ClusterGoalIndex,
                               ClusterComparatorGoalCost,
                               MAX_CLUSTER_COUNT,
                               MAX_CLUSTER_COUNT>
      goalSearchClustersOpen;
  std::size_t iteration = 0;

  Node* goalNode = nullptr;

  Visualizer visualizer;
};

}  // namespace metronome
