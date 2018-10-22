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
class TimeBoundedAStar final
    : public OnlinePlanner<Domain, TerminationChecker> {
 public:
  using State = typename Domain::State;
  using Action = typename Domain::Action;
  using Cost = typename Domain::Cost;
  using ActionBundle =
      typename OnlinePlanner<Domain, TerminationChecker>::ActionBundle;

  // Only used for visualizations
  static constexpr std::size_t NODE_ID_OFFSET = 100000;

  TimeBoundedAStar(const Domain &domain, const Configuration &configuration)
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

    for (std::size_t i = 0; i < rootToTargetPath.size(); ++i) {
      if (rootToTargetPath[i].expectedTargetState == agentState) {
        if (rootToTargetPath.size() == i + 1) {
          ActionBundle actionBundle{domain.getIdentityAction(),
                                    domain.getActionDuration()};
          actionBundle.label = "wait at the frontier";
          actionBundle.expectedTargetState = agentState;
          return {actionBundle};
        }

        auto actionBundle = rootToTargetPath.at(i + 1);
        actionBundle.label = "forward step";

        return {actionBundle};
      }
    }

    if (agentState == rootNode->state) {
      if (rootToTargetPath.empty()) {
        throw MetronomeException("No path found from the root.");
      }

      auto firstActionBundle = rootToTargetPath[0];
      firstActionBundle.label = "firstStep";
      return {firstActionBundle};
    }

    // Backtrack
    ActionBundle actionBundle = backtrackFromNode(agentState);
    return {actionBundle};
  }

  ActionBundle backtrackFromNode(const State &sourceState) const {
    const auto nodeIterator = nodes.find(sourceState);
    if (nodeIterator == nodes.cend()) {
      throw MetronomeException(
          "Agent node was not created yet. Cannot "
          "backtrack.");
    }

    const Node *agentNode = nodeIterator->second;
    const Action action = agentNode->action.inverse();
    const Cost cost = agentNode->g - agentNode->parent->g;
    const State expectedState = agentNode->parent->state;

    ActionBundle actionBundle(action, cost);
    actionBundle.expectedTargetState = expectedState;
    actionBundle.label = "backtrack";
    return actionBundle;
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

  struct NodeComparatorWeightedF {
    NodeComparatorWeightedF(double weight) : weight(weight) {}

    int operator()(const Node *lhs, const Node *rhs) const {
      if (lhs->g + weight * lhs->h < rhs->g + weight * rhs->h) return -1;
      if (lhs->g + weight * lhs->h > rhs->g + weight * rhs->h) return 1;
      if (lhs->g > rhs->g) return -1;
      if (lhs->g < rhs->g) return 1;
      return 0;
    }

    const double weight;
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
      NodeComparatorWeightedF,
      Memory::OPEN_LIST_SIZE,
      Memory::OPEN_LIST_SIZE>
      openList{{weight}};
  std::unordered_map<State, Node *, typename metronome::Hash<State>> nodes;
  ObjectPool<Node, Memory::NODE_LIMIT> nodePool;

  std::size_t iteration = 0;

  Node *goalNode = nullptr;

  Visualizer visualizer;
};

}  // namespace metronome
