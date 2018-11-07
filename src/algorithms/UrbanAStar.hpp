#pragma once

#include <fcntl.h>
#include <deque>
#include <ostream>
#include <unordered_map>
#include <vector>
#include "MemoryConfiguration.hpp"
#include "MetronomeException.hpp"
#include "OfflinePlanner.hpp"
#include "Planner.hpp"
#include "domains/SuccessorBundle.hpp"
#include "dynamic_priority_queue.hpp"
#include "easylogging++.h"
#include "experiment/Configuration.hpp"
#include "utils/Hash.hpp"
#include "utils/ObjectPool.hpp"
#include "utils/PriorityQueue.hpp"
#include "utils/TimeMeasurement.hpp"
#include "visualization/Visualizer.hpp"

namespace metronome {

template <typename Domain>
class UrbanAStar final : public OfflinePlanner<Domain> {
 public:
  using State = typename Domain::State;
  using Action = typename Domain::Action;
  using Cost = typename Domain::Cost;
  using Planner = Planner<Domain>;
  using ActionBundle = typename Planner::ActionBundle;

  // Only used for visualizations
  static constexpr std::size_t NODE_ID_OFFSET = 100000;

  UrbanAStar(const Domain &domain, const Configuration &configuration)
      : domain(domain),
        weight(configuration.getDouble(WEIGHT)),
        safetyVersion(configuration.getLong("safetyVersion")) {
    // Initialize hash table
    nodes.max_load_factor(1);
    nodes.reserve(Memory::NODE_LIMIT);
  }

  std::vector<Action> plan(const State &agentState) {
    createInitialNode(agentState);
    explore(agentState);

    std::vector<ActionBundle> path;
    if (safetyVersion == 1) {
      path = extractPath(partialGoalNode, rootNode);
      std::size_t index = findFirstSafeNode(path);

      for (std::size_t i = index + 2; i < path.size(); ++i) {
        assert(i > 0);

        const auto fastSuccessors =
            domain.fastSuccessors(path[i - 1].expectedTargetState);

        assert(fastSuccessors.size() == 1 || i == path.size() - 1);

        ActionBundle actionBundle;

        if (i == path.size() - 1) {
          auto fastSuccessor =
              domain.getStopActionBundle(path[i - 1].expectedTargetState);

          actionBundle = {fastSuccessor.action, fastSuccessor.actionCost};
          actionBundle.expectedTargetState = fastSuccessor.state;
        } else {
          auto fastSuccessor = fastSuccessors[0];

          actionBundle = {fastSuccessor.action, fastSuccessor.actionCost};
          actionBundle.expectedTargetState = fastSuccessor.state;
        }

        path[i] = actionBundle;
      }

    } else if (safetyVersion == 2) {
      auto nodePath = extractNodePath(partialGoalNode, rootNode);
      proveSafety(nodePath);
      // TODO remove
      visualizeProgress(agentState, path);

      if (goalNode == nullptr)
        throw MetronomeException("Goal is not reachable.");
      path = extractPath(goalNode, rootNode);
    } else {
      if (goalNode == nullptr)
        throw MetronomeException("Goal is not reachable.");

      path = extractPath(goalNode, rootNode);
    }

#ifdef STREAM_GRAPH
    visualizeProgress(agentState, path);
#endif

    std::vector<Action> actions;
    actions.reserve(path.size());

    for (const auto &actionBundle : path) {
      actions.push_back(actionBundle.action);
    }

    return actions;
  }

  std::size_t findFirstSafeNode(std::vector<ActionBundle> path) {
    // TODO set goalNode
    const auto partialGoalState = domain.getPartialGoalState();
    auto currentState = partialGoalState;

    // Add first node

    while (true) {
      const auto predecessors = domain.partialPredecessors(currentState);
      if (predecessors.empty())
        throw MetronomeException(
            "Backward safety "
            "search can't reach "
            "the path.");

      // We only handle a single emergency action (trivial to extend)
      assert(predecessors.size() == 1);
      const State predecessor = predecessors[0];

      // If in path stop else do more

      std::size_t intersectionIndex = 0;
      for (const auto &actionBundle : path) {
        if (domain.isSafetyProof(actionBundle.expectedTargetState,
                                 currentState)) {
          return intersectionIndex;
        }

        ++intersectionIndex;
      }

      currentState = predecessor;
    }
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

  class Node {
   public:
    Node(Node *parent, const State &state, Action action, Cost g, Cost h)
        : parent{parent}, state{state}, action{std::move(action)}, g{g}, h{h} {}

    Cost f() const { return g + h; }

    unsigned long hash() const { return state.hash(); }

    bool operator==(const Node &node) const { return state == node.state; }

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
    State state;
    /** Action that led to the current node from the parent node */
    Action action;
    /** Cost from the root node */
    Cost g;
    /** Heuristic cost of the node */
    Cost h;
    bool closed = false;
  };

  struct NodeComparator {
    NodeComparator(const Domain &domain) : domain(domain) {}

    int operator()(const Node *lhs, const Node *rhs) const {
      if (domain.less(lhs->state, rhs->state)) return -1;
      if (domain.less(rhs->state, lhs->state)) return 1;

      if (lhs->g > rhs->g) return -1;
      if (lhs->g < rhs->g) return 1;

      return 0;
    }

    const Domain &domain;
  };

  struct SafeNodeComparator {
    SafeNodeComparator(const Domain &domain) : domain(domain) {}

    int operator()(const Node *lhs, const Node *rhs) const {
      if (domain.safeLess(lhs->state, rhs->state)) return -1;
      if (domain.safeLess(rhs->state, lhs->state)) return 1;

      if (lhs->g > rhs->g) return -1;
      if (lhs->g < rhs->g) return 1;

      return 0;
    }

    const Domain &domain;
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

  void createInitialNode(const State &initialState) {
    Planner::incrementGeneratedNodeCount();

    Node *&initialNode = nodes[initialState];
    initialNode = nodePool.construct(Node{
        nullptr, initialState, Action(), 0, domain.heuristic(initialState)});

    rootNode = initialNode;
    openList.push(initialNode);
  }

  void explore(const State &) {
    while (!openList.empty()) {
      if (goalNode != nullptr) return;
      if (safetyVersion > 0 && partialGoalNode != nullptr) return;

      auto currentNode = openList.pop();
      expandNode(currentNode);

      // Visualization
      std::size_t id = nodePool.index(currentNode) + NODE_ID_OFFSET;
      visualizer.addNode(
          id, currentNode->state.getX(), currentNode->state.getY(), 0, 1);

      if (currentNode->parent != nullptr) {
        visualizer.addEdge(
            id, nodePool.index(currentNode->parent) + NODE_ID_OFFSET, id);
      }
    }
  }

  void proveSafety(std::vector<Node *> path) {
    while (!path.empty() && !domain.canBeSafe(path.back()->state)) {
      path.pop_back();
    }

    // Reverse the vector
    std::reverse(path.begin(), path.end());

    for (auto node : path) {
      findSafetyProof(node);

      // If the proof was successful
      if (goalNode != nullptr) return;
    }
  }

  void findSafetyProof(Node *source) {
    assert(safetyOpenList.empty());

    safetyOpenList.push(source);

    // visualization
    std::string sourceId =
        std::to_string(nodePool.index(source) + NODE_ID_OFFSET);

    while (!safetyOpenList.empty()) {
      if (goalNode != nullptr) return;

      const auto currentNode = safetyOpenList.pop();

      safetyExpandNode(currentNode);

      std::size_t id = nodePool.index(currentNode) + NODE_ID_OFFSET;
      visualizer.addNode(id,
                         currentNode->state.getX(),
                         currentNode->state.getY(),
                         0,
                         1,
                         "safety" + sourceId);

      if (currentNode->parent != nullptr) {
        visualizer.addEdge(id,
                           nodePool.index(currentNode->parent) + NODE_ID_OFFSET,
                           id,
                           "safety" + sourceId);
      }
    }
  }

  void safetyExpandNode(Node *sourceNode) {
    Planner::incrementExpandedNodeCount();

    //    if (sourceNode->closed) return;
    sourceNode->closed = true;

    if (domain.isGoal(sourceNode->state)) {
      LOG(INFO) << "Goal was expanded!";
      goalNode = sourceNode;
    }

    auto fastSuccessors = domain.fastSuccessors(sourceNode->state);
    auto successors = domain.successors(sourceNode->state);

    //ugly
    for (const auto& fastSuccessor : fastSuccessors) {
      successors.push_back(fastSuccessor);
    }

    for (auto successor : successors) {
      auto successorState = successor.state;
      Node *&successorNode = nodes[successorState];

      if (successorNode == nullptr) {
        Planner::incrementGeneratedNodeCount();
        successorNode = nodePool.construct(sourceNode,
                                           successor.state,
                                           successor.action,
                                           std::numeric_limits<Cost>::max(),
                                           domain.heuristic(successor.state));

        // Insert new region
        safetyOpenList.insertOrUpdate(successorNode);
      } else if (domain.safeLess(successorState, successorNode->state)) {
        if (successorNode->closed) continue;

        successorNode->g = sourceNode->g + successor.actionCost;
        successorNode->parent = sourceNode;
        successorNode->action = successor.action;
        successorNode->state = successorState;

        // Update existing region
        safetyOpenList.insertOrUpdate(successorNode);
      }
    }
  }

  void expandNode(Node *sourceNode) {
    Planner::incrementExpandedNodeCount();

    if (sourceNode->closed) return;
    sourceNode->closed = true;

    if (domain.isGoal(sourceNode->state)) {
      LOG(INFO) << "Goal was expanded!";
      goalNode = sourceNode;
    }

    if (domain.isPartialGoal(sourceNode->state)) {
      partialGoalNode = sourceNode;
    }

    for (auto successor : domain.successors(sourceNode->state)) {
      auto successorState = successor.state;
      Node *&successorNode = nodes[successorState];

      if (successorNode == nullptr) {
        Planner::incrementGeneratedNodeCount();
        successorNode = nodePool.construct(sourceNode,
                                           successor.state,
                                           successor.action,
                                           std::numeric_limits<Cost>::max(),
                                           domain.heuristic(successor.state));

        // Insert new region
        openList.insertOrUpdate(successorNode);
      } else if (domain.less(successorState, successorNode->state)) {
        successorNode->g = sourceNode->g + successor.actionCost;
        successorNode->parent = sourceNode;
        successorNode->action = successor.action;
        successorNode->state = successorState;

        // Update existing region
        openList.insertOrUpdate(successorNode);
      }
    }
  }

  std::vector<ActionBundle> extractPath(const Node *targetNode,
                                        const Node *sourceNode) const {
    if (targetNode == sourceNode) return {};

    std::vector<ActionBundle> actionBundles;
    auto currentNode = targetNode;

    while (currentNode != sourceNode) {
      ActionBundle actionBundle(currentNode->action,
                                currentNode->g - currentNode->parent->g);
      actionBundle.expectedTargetState = currentNode->state;
      actionBundles.push_back(std::move(actionBundle));

      currentNode = currentNode->parent;
    }

    std::reverse(actionBundles.begin(), actionBundles.end());
    return actionBundles;
  }

  std::vector<Node *> extractNodePath(Node *targetNode,
                                      const Node *sourceNode) const {
    if (targetNode == sourceNode) return {};

    std::vector<Node *> nodePath;
    auto currentNode = targetNode;

    while (currentNode != sourceNode) {
      nodePath.emplace_back(currentNode);
      currentNode = currentNode->parent;
    }

    std::reverse(nodePath.begin(), nodePath.end());
    return nodePath;
  }

  const Domain &domain;
  const double weight;
  const Node *rootNode;

  cserna::DynamicPriorityQueue<
      Node *,
      cserna::NonIntrusiveIndexFunction<Node *, Hash<Node>, NodeEquals>,
      NodeComparator,
      Memory::OPEN_LIST_SIZE,
      Memory::OPEN_LIST_SIZE>
      openList{{domain}};

  cserna::DynamicPriorityQueue<
      Node *,
      cserna::NonIntrusiveIndexFunction<Node *, Hash<Node>, NodeEquals>,
      SafeNodeComparator,
      Memory::OPEN_LIST_SIZE,
      Memory::OPEN_LIST_SIZE>
      safetyOpenList{{domain}};

  std::unordered_map<State, Node *, typename metronome::Hash<State>> nodes;
  ObjectPool<Node, Memory::NODE_LIMIT> nodePool;

  std::size_t iteration = 0;

  Node *goalNode = nullptr;
  Node *partialGoalNode = nullptr;

  Visualizer visualizer;
  long long int safetyVersion;
};

}  // namespace metronome
