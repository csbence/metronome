#pragma once

#include <fcntl.h>
#include <MemoryConfiguration.hpp>
#include <domains/SuccessorBundle.hpp>
#include <unordered_map>
#include <vector>
#include "MetronomeException.hpp"
#include "OnlinePlanner.hpp"
#include "easylogging++.h"
#include "experiment/Configuration.hpp"
#include "utils/Hash.hpp"
#include "utils/ObjectPool.hpp"
#include "utils/PriorityQueue.hpp"
#include "utils/TimeMeasurement.hpp"

namespace metronome {

template <typename Domain, typename TerminationChecker>
class LssLrtaStar final : public OnlinePlanner<Domain, TerminationChecker> {
 public:
  using State = typename Domain::State;
  using Action = typename Domain::Action;
  using Cost = typename Domain::Cost;
  using Planner = Planner<Domain>;
  using ActionBundle = typename Planner::ActionBundle;

  LssLrtaStar(const Domain& domain, const Configuration&) : domain{domain} {
    // Initialize hash table
    nodes.max_load_factor(1);
    nodes.reserve(Memory::NODE_LIMIT);
  }

  std::vector<ActionBundle> selectActions(
      const State& startState,
      TerminationChecker& terminationChecker) override {
    if (domain.isGoal(startState)) {
      // Goal is already reached
      return std::vector<ActionBundle>();
    }

    // Learning phase
    if (openList.isNotEmpty()) {
      learn(terminationChecker);
    }

    const auto bestNode = explore(startState, terminationChecker);

    return extractPath(bestNode, nodes[startState]);
  }

 private:
  class Edge;

  class Node {
   public:
    Node(Node* parent,
         const State& state,
         Action action,
         Cost g,
         Cost h,
         bool open,
         unsigned int iteration = 0)
        : parent{parent},
          state{state},
          action{std::move(action)},
          g{g},
          h{h},
          open{open},
          iteration{iteration} {}

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
      stream << (open ? " Open" : " Not Open");
      return stream.str();
    }

    /** Index used by the priority queue */
    mutable unsigned int index;
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
    /** True if the node is in the open list */
    bool open;
    /** Last iteration when the node was updated */
    unsigned int iteration;
    /** List of all the predecessors that were discovered in the current
     * exploration phase. */
    std::vector<Edge> predecessors;
  };

  class Edge {
   public:
    Edge(Node* predecessor, Action action, Cost actionCost)
        : predecessor{predecessor}, action{action}, actionCost{actionCost} {}

    Node* predecessor;
    const Action action;
    const Cost actionCost;
  };

  void learn(TerminationChecker& terminationChecker) {
    ++iterationCounter;

    // Reorder the open list based on the heuristic values
    openList.reorder(hComparator);

    while (!terminationChecker.reachedTermination() && openList.isNotEmpty()) {
      auto currentNode = popOpenList();
      currentNode->iteration = iterationCounter;

      Cost currentHeuristicValue = currentNode->h;

      // update heuristic actionDuration of each predecessor
      for (auto predecessor : currentNode->predecessors) {
        Node* predecessorNode = predecessor.predecessor;

        if (predecessorNode->iteration == iterationCounter &&
            !predecessorNode->open) {
          // This node was already learned and closed in the current iteration
          continue;
          // TODO Review this. This could be incorrect if the action costs are
          // not uniform
        }

        if (!predecessorNode->open) {
          // This node is not open yet, because it was not visited in the
          // current planning iteration

          predecessorNode->h = currentHeuristicValue + predecessor.actionCost;
          assert(predecessorNode->iteration == iterationCounter - 1);
          predecessorNode->iteration = iterationCounter;

          addToOpenList(*predecessorNode);
        } else if (predecessorNode->h >
                   currentHeuristicValue + predecessor.actionCost) {
          // This node was visited in this learning phase, but the current path
          // is better then the previous
          predecessorNode->h = currentHeuristicValue + predecessor.actionCost;
          openList.update(*predecessorNode);
        }
      }
    }
  }

  const Node* explore(const State& startState,
                      TerminationChecker& terminationChecker) {
    ++iterationCounter;
    clearOpenList();
    openList.reorder(fComparator);

    Planner::incrementGeneratedNodeCount();
    Node*& startNode = nodes[startState];

    if (startNode == nullptr) {
      startNode = nodePool.construct(Node{nullptr,
                                          startState,
                                          Action(),
                                          0,
                                          domain.heuristic(startState),
                                          true});
    } else {
      startNode->g = 0;
      startNode->action = Action();
      startNode->predecessors.clear();
      startNode->parent = nullptr;
    }

    startNode->iteration = iterationCounter;
    addToOpenList(*startNode);

    while (!terminationChecker.reachedTermination() && openList.isNotEmpty()) {
      Node* const listTopNode = topOpenList();

      if (domain.isGoal(listTopNode->state)) {
        return listTopNode;
      }
      Node* const currentNode = popOpenList();

      terminationChecker.notifyExpansion();
      expandNode(currentNode);
    }

    return openList.top();
  }

  void expandNode(Node* sourceNode) {
    Planner::incrementExpandedNodeCount();

    for (auto successor : domain.successors(sourceNode->state)) {
      auto successorState = successor.state;

      Node*& successorNode = nodes[successorState];

      if (successorNode == nullptr) {
        successorNode = createNode(sourceNode, successor);
      }

      // If the node is outdated it should be updated.
      if (successorNode->iteration != iterationCounter) {
        successorNode->iteration = iterationCounter;
        successorNode->predecessors.clear();
        successorNode->g = std::numeric_limits<Cost>::max();
        successorNode->open =
            false;  // It is not on the open list yet, but it will be
        // parent, action, and actionCost is outdated too, but not relevant.
      }

      // Add the current state as the predecessor of the child state
      successorNode->predecessors.emplace_back(
          sourceNode, successor.action, successor.actionCost);

      // Skip if we got back to the parent
      if (sourceNode->parent != nullptr &&
          successorState == sourceNode->parent->state) {
        continue;
      }

      // only generate those state that are not visited yet or whose cost value
      // are lower than this path
      Cost successorGValueFromCurrent{sourceNode->g + successor.actionCost};
      if (successorNode->g > successorGValueFromCurrent) {
        successorNode->g = successorGValueFromCurrent;
        successorNode->parent = sourceNode;
        successorNode->action = successor.action;

        if (!successorNode->open) {
          addToOpenList(*successorNode);
        } else {
          openList.update(*successorNode);
        }
      }
    }
  }

  Node* createNode(Node* sourceNode, SuccessorBundle<Domain> successor) {
    Planner::incrementGeneratedNodeCount();
    Node* newNode = nodePool.construct(Node{sourceNode,
                                   successor.state,
                                   successor.action,
                                   std::numeric_limits<Cost>::max(),
                                   domain.heuristic(successor.state),
                                   true});

    nodes[newNode->state] = newNode;
    return newNode;
  }

  void clearOpenList() {
    openList.forEach([](Node* node) { node->open = false; });
    openList.clear();
  }

  Node* topOpenList() const {
	if (openList.isEmpty()) {
      throw MetronomeException("Open list was empty, goal not reachable");
	}

	return openList.top();
  }

  Node* popOpenList() {
    if (openList.isEmpty()) {
      throw MetronomeException("Open list was empty, goal not reachable.");
    }

    Node* node = openList.pop();
    node->open = false;
    return node;
  }

  void addToOpenList(Node& node) {
    node.open = true;
    openList.push(node);
  }

  std::vector<ActionBundle> extractPath(const Node* targetNode,
                                        const Node* sourceNode) const {
    if (targetNode == sourceNode) {
      //                        LOG(INFO) << "We didn't move:" <<
      //                        sourceNode->toString();
      return std::vector<ActionBundle>();
    }

    std::vector<ActionBundle> actionBundles;
    auto currentNode = targetNode;

    while (currentNode != sourceNode) {
      // The g difference of the child and the parent gives the action cost from
      // the parent
      actionBundles.emplace_back(currentNode->action,
                                 currentNode->g - currentNode->parent->g);
      currentNode = currentNode->parent;
    }

    std::reverse(actionBundles.begin(), actionBundles.end());
    return actionBundles;
  }

  static int fComparator(const Node& lhs, const Node& rhs) {
    if (lhs.f() < rhs.f()) return -1;
    if (lhs.f() > rhs.f()) return 1;
    if (lhs.g > rhs.g) return -1;
    if (lhs.g < rhs.g) return 1;
    return 0;
  }

  static int hComparator(const Node& lhs, const Node& rhs) {
    if (lhs.h < rhs.h) return -1;
    if (lhs.h > rhs.h) return 1;
    return 0;
  }

  const Domain& domain;
  PriorityQueue<Node> openList{Memory::OPEN_LIST_SIZE, fComparator};
  std::unordered_map<State, Node*, typename metronome::Hash<State>> nodes{};
  ObjectPool<Node, Memory::NODE_LIMIT> nodePool;
  unsigned int iterationCounter{0};
};
}  // namespace metronome
