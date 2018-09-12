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

  static constexpr std::size_t CLUSTER_NODE_LIMIT = 1000;
  static constexpr std::size_t CLUSTER_G_RADIUS = 10;
  static constexpr std::size_t MAX_CLUSTER_COUNT = 1000;

  ClusterRts(const Domain& domain, const Configuration&) : domain{domain} {
    // Initialize hash table
    nodes.max_load_factor(1);
    nodes.reserve(Memory::NODE_LIMIT);
  }

  std::vector<ActionBundle> selectActions(
      const State& agentState,
      TerminationChecker& terminationChecker) override {
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

    return extractPath();
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

    Cluster* containingCluster;
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
    Cluster* cluster;
    Node* bestFrontierNode;

    /**
     * Actions leading toward a cluster. Only populated on demand.
     * Should be reset when the best frontier node toward the cluster is
     * changed.
     */
    std::vector<Action> actionsTowardsCluster;
  };

  class Cluster {
   public:
    Node* coreNode;
    std::size_t nodeCount = 0;
    bool depleted = false;

    Node* bestHNode;

    std::vector<ClusterEdge> reachableClusters;
    cserna::DynamicPriorityQueue<
        Node*,
        cserna::NonIntrusiveIndexFunction<Node*, Hash<Node>, NodeEquals>,
        NodeComparatorF,
        CLUSTER_NODE_LIMIT,
        CLUSTER_NODE_LIMIT>
        openList;

    std::size_t openListIndex = std::numeric_limits<std::size_t>::max();
  };

  struct ClusterIndex {
    std::size_t& operator()(Cluster* cluster) const {
      return cluster->openListIndex;
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

  void createInitialCluster(const State& initialState) {
    Planner::incrementGeneratedNodeCount();

    Node*& initialNode = nodes[initialState];

    initialNode = nodePool.construct(Node{
        nullptr, initialState, Action(), 0, domain.heuristic(initialState)});

    auto spawnedCluster = clusterPool.construct();
    spawnedCluster->coreNode = initialNode;
    openClusters.push(spawnedCluster);

    initialNode->containingCluster = spawnedCluster;
  }

  void explore(const State& startState,
               TerminationChecker& terminationChecker) {
    while (!terminationChecker.reachedTermination() &&
           openClusters.isNotEmpty()) {
      auto cluster = openClusters.pop();
      // todo check if the goal state was expanded

      bool expanded = expandCluster(cluster);
      if (expanded) terminationChecker.notifyExpansion();
    }
  }

  bool expandCluster(Cluster* sourceCluster) {
    if (sourceCluster->depleted) {
      return false;
    }

    if (sourceCluster->openList.isEmpty()) {
      sourceCluster->depleted = true;
      return false;
    }

    // A new cluster core should be spawned in the following cases:
    // * Cluster node limit is reached
    // * Node is beyond a threshold cost (g) from the cluster core
    // * Node is beyond a threshold distance (d) from the cluster core ->
    // todo implement d

    auto currentNode = sourceCluster->openList.pop();

    if (currentNode->containingCluster != nullptr) {
      return false;
    }

    bool spawnNewCore = sourceCluster->nodeCount >= CLUSTER_NODE_LIMIT ||
                        currentNode->g >= CLUSTER_G_RADIUS;

    if (spawnNewCore) {
      auto spawnedCluster = clusterPool.construct();
      spawnedCluster->coreNode = currentNode;
      openClusters.push(spawnedCluster);

      currentNode->containingCluster = spawnedCluster;
    } else {
      currentNode->containingCluster = sourceCluster;
    }

    ++(currentNode->containingCluster->nodeCount);
    expandNode(currentNode);

    auto gridState = static_cast<GridWorld::State>(currentNode->state);

    std::size_t id = nodePool.index(currentNode);
    visualizer.addNode(id, gridState.getX(), gridState.getY());
    visualizer.addEdge(
        id,
        nodePool.index(currentNode->parent),
        id,
        "cluster:" + clusterPool.index(currentNode->containingCluster));

    return true;
  }

  void expandNode(Node* sourceNode) {
    Planner::incrementExpandedNodeCount();
    auto containingCluster = sourceNode->containingCluster;

    for (auto successor : domain.successors(sourceNode->state)) {
      auto successorState = successor.state;

      Node*& successorNode = nodes[successorState];

      if (successorNode == nullptr) {
        Planner::incrementGeneratedNodeCount();
        successorNode = nodePool.construct(sourceNode,
                                           successor.state,
                                           successor.action,
                                           std::numeric_limits<Cost>::max(),
                                           domain.heuristic(successor.state));
      }

      // This node is already taken by another cluster
      if (successorNode->containingCluster != nullptr) {
        continue;
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

        sourceNode->containingCluster->openList.insertOrUpdate(successorNode);
      }
    }
  }

  std::vector<ActionBundle> extractPath() const {
    // Find abstract path to goal
    // 1. Find path to containing region center
    // 2. Find abstract path to goal via region centers
    // 3. Find last segment from region center to target node
    // If the path partially overlaps with the current path stitch

    for (auto const& s : states) {
      for (auto const& a : s.get_actions()) {
        for (auto const& t : a.get_outcomes()) {
          if (!t.is_normalized()) return false;
        }
      }
    }

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

  Visualizer visualizer;
};

}  // namespace metronome
