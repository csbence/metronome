#pragma once

#include <easylogging++.h>
#include <MemoryConfiguration.hpp>
#include <domains/Traffic.hpp>
#include <unordered_map>
#include <utils/PriorityQueue.hpp>
#include <vector>
#include "OfflinePlanner.hpp"
#include "Planner.hpp"
#include "experiment/Configuration.hpp"
#include "utils/Hash.hpp"
#include "utils/ObjectPool.hpp"
#include "visualization/Visualizer.hpp"

namespace metronome {

template <typename Domain>
class SuboptimalAStar final : public OfflinePlanner<Domain> {
  using State = typename Domain::State;
  using Action = typename Domain::Action;
  using Cost = typename Domain::Cost;
  using Planner = metronome::Planner<Domain>;

 public:
  SuboptimalAStar(const Domain& domain, const Configuration&)
      : domain(domain), openList(Memory::OPEN_LIST_SIZE, fValueComparator) {
    // Initialize hash table
    nodes.max_load_factor(1);
    nodes.reserve(Memory::NODE_LIMIT);
  }

  std::vector<Action> plan(const State& startState) override {
    Cost heuristic = domain.heuristic(startState);
    Node localStartNode = Node(nullptr, startState, Action(), 0, heuristic);

    auto startNode = nodePool.construct(localStartNode);
    Planner::incrementGeneratedNodeCount();

    nodes[localStartNode.state] = startNode;

    openList.push(localStartNode);

    bool optimal = true;

    while (!openList.isEmpty()) {
      if (domain.isGoal(openList.top()->state)) {
        std::vector<Action> actions;
        // Goal is reached

        Node* currentNode = openList.top();
        while (startState != currentNode->state) {
          actions.push_back(currentNode->action);
          currentNode = currentNode->parent;
        }

        std::reverse(actions.begin(), actions.end());
        visualizer.post();
        return actions;
      }

      constexpr double w = 1.2;

      if (optimal) {
        Node* topNode = openList.top();
        Cost startH = startNode->f;
        Cost currentH = topNode->f - topNode.g;
        Cost currentG = topNode->g;
        double stepError = (currentG - startH - currentH) / currentG;

        double costEstimate = currentH * stepError + currentG;
        double upperBound = w * (currentH + currentG);

        if (costEstimate * 1.1 < upperBound) {
          optimal = false;
          
          suboptimalOpenList.clear();
          suboptimalOpenList.queue = openList.queue;
          
          continue;
        }

        // 1. Do A* to expand bound (C)
        Planner::incrementExpandedNodeCount();
        Node* currentNode = openList.pop();

        visualizer.addNode(nodePool.index(currentNode));

        expandNode(currentNode);

      } else {
        // 2. Do suboptimal expansion
        
        
      }
    }

    throw MetronomeException("Goal can't be reached!");
  }

  void expandNode(const Node* currentNode) {
    for (auto successor : domain.successors(currentNode->state)) {
      if (successor.state == currentNode->state) {
        continue;  // Skip parent TODO this might be unnecessary
      }

      auto& successorNode = nodes[successor.state];
      auto newCost = successor.actionCost + currentNode->g;

      if (successorNode == nullptr) {
        incrementGeneratedNodeCount();

        // New state discovered
        const Node tempSuccessorNode(
            currentNode,
            successor.state,
            successor.action,
            newCost,
            newCost + domain.heuristic(successor.state));

        successorNode = nodePool.construct(std::__1::move(tempSuccessorNode));
        openList.push(*successorNode);
      } else if (successorNode->g > newCost) {
        // Better path found to an existing state
        successorNode->parent = currentNode;
        successorNode->action = successor.action;
        successorNode->g = newCost;
        successorNode->f = newCost + domain.heuristic(successor.state);

        openList.insertOrUpdate(*successorNode);
      } else {
        // The new path is not better than the existing
      }
    }
  }

 private:
  class Node {
   public:
    Node(Node* parent, const State state, Action action, Cost g, Cost f)
        : parent{parent}, state{state}, action{std::move(action)}, g{g}, f{f} {}

    unsigned long hash() const { return state->hash(); }

    std::string toString() const {
      std::ostringstream stream;
      stream << "s: " << state << " g: " << g << " h: " << f - g << " f: " << f
             << " a: " << action << " p: ";
      if (parent == nullptr) {
        stream << "None";
      } else {
        stream << parent->state;
      }
      return stream.str();
    }

    bool operator==(const Node& node) const { return state == node.state; }

    mutable unsigned int index{std::numeric_limits<unsigned int>::max()};
    Node* parent;
    const State state;
    Action action;
    Cost g;
    Cost f;
  };

  static int fValueComparator(const Node& lhs, const Node& rhs) {
    if (lhs.f < rhs.f) return -1;
    if (lhs.f > rhs.f) return 1;
    if (lhs.g > rhs.g) return -1;
    if (lhs.g < rhs.g) return 1;
    return 0;
  }

  const Domain& domain;
  PriorityQueue<Node> openList;
  PriorityQueue<Node> suboptimalOpenList;
  
  std::unordered_map<State, Node*, typename metronome::Hash<State>> nodes;
  ObjectPool<Node, Memory::NODE_LIMIT> nodePool;

  Visualizer visualizer;
};

}  // namespace metronome
