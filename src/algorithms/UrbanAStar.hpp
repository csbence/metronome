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
class UrbanAStar final : public OnlinePlanner<Domain, TerminationChecker> {
 public:
  using State = typename Domain::State;
  using Action = typename Domain::Action;
  using Cost = typename Domain::Cost;
  using ActionBundle =
      typename OnlinePlanner<Domain, TerminationChecker>::ActionBundle;

  // Only used for visualizations
  static constexpr std::size_t NODE_ID_OFFSET = 100000;

  UrbanAStar(const Domain &domain, const Configuration &configuration)
      : domain(domain), weight(configuration.getDouble(WEIGHT)) {
    // Initialize hash table
    nodes.max_load_factor(1);
    nodes.reserve(Memory::NODE_LIMIT);
  }

  std::vector<ActionBundle> selectActions(
      const State &agentState,
      TerminationChecker &terminationChecker) override {
    ++iteration;
    if (domain.isGoal(agentState)) {
      // Goal is already reached
      return {};
    }

    // ---    Initialize    ---
    if (nodePool.empty()) createInitialNode(agentState);

    explore(agentState, terminationChecker);

    std::vector<ActionBundle> rootToTargetPath;

    if (goalNode != nullptr) {
      rootToTargetPath = extractPath(goalNode, rootNode);
    } else {
      rootToTargetPath = extractPath(openList.top(), rootNode);
    }

#ifdef STREAM_GRAPH
    visualizeProgress(agentState, rootToTargetPath);
#endif

    return rootToTargetPath;
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
    const State state;
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

  void explore(const State &, TerminationChecker &terminationChecker) {
    while (!terminationChecker.reachedTermination() && !openList.empty()) {
      if (goalNode != nullptr) return;

      auto currentNode = openList.pop();
      expandNode(currentNode);
      terminationChecker.notifyExpansion();

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

  void expandNode(Node *sourceNode) {
    Planner::incrementExpandedNodeCount();

    if (sourceNode->closed) return;
    sourceNode->closed = true;

    if (domain.isGoal(sourceNode->state)) {
      LOG(INFO) << "Goal was expanded!";
      goalNode = sourceNode;
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
  std::unordered_map<State, Node *, typename metronome::Hash<State>> nodes;
  ObjectPool<Node, Memory::NODE_LIMIT> nodePool;

  std::size_t iteration = 0;

  Node *goalNode = nullptr;

  Visualizer visualizer;
};

}  // namespace metronome
