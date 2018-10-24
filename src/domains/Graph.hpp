#ifndef METRONOME_GRAPH_HPP
#define METRONOME_GRAPH_HPP

#include <experiment/Configuration.hpp>
#include <limits>
#include <optional>
#include <string>
#include <utils/String.hpp>
#include <vector>
#include "SuccessorBundle.hpp"
#include "easylogging++.h"

namespace metronome {

class Graph {
 public:
  typedef long long int Cost;
  static constexpr Cost COST_MAX = std::numeric_limits<Cost>::max();

  class Action {
   public:
    Action() : sourceStateId{0}, targetStateId{0}, actionCost{0} {}

    Action(std::size_t sourceStateId,
           std::size_t targetStateId,
           Cost actionCost)
        : sourceStateId{sourceStateId},
          targetStateId{targetStateId},
          actionCost{actionCost} {}

    Action(const Action& action)
        : sourceStateId{action.sourceStateId},
          targetStateId{action.targetStateId},
          actionCost{action.actionCost} {}

    Action& operator=(const Action&) = default;

    static Action getIdentity() {
      throw MetronomeException("Graph does not have an identity action");
    }

    std::string toString() const {
      return "s: " + std::to_string(sourceStateId) +
             " t: " + std::to_string(targetStateId) +
             " c: " + std::to_string(actionCost);
    }

    bool operator==(const Action& action) const {
      return action.sourceStateId == sourceStateId &&
             action.targetStateId == targetStateId &&
             action.actionCost == actionCost;
    }

    bool operator!=(const Action& rhs) const { return !(rhs == *this); }

    friend std::ostream& operator<<(std::ostream& stream,
                                    const Action& action) {
      return stream;  // TODO
    }

    std::size_t getSourceStateId() const { return sourceStateId; }
    std::size_t getTargetStateId() const { return targetStateId; }
    Cost getActionCost() const { return actionCost; }

   private:
    std::size_t sourceStateId;
    std::size_t targetStateId;
    Cost actionCost;
  };

  class State {
   public:
    State(size_t id) : id{id} {}
    State(const State&) = default;
    State& operator=(const State&) = default;
    bool operator==(const State& state) const { return state.id == id; }
    bool operator!=(const State& rhs) const { return !(rhs == *this); }
    std::size_t hash() const { return id; }
    std::size_t getId() const { return id; }
    std::string toString() const { return std::to_string(id); }

    friend std::ostream& operator<<(std::ostream& stream, const State& state) {
      stream << "State: " << state.getId();
      return stream;
    }

   private:
    std::size_t id;
  };

  Graph(const Configuration& configuration, std::istream& input) {
    std::string line;

    while (getline(input, line)) {
      if (line.empty()) {
        continue;
      }

      const char& firstChar = line[0];
      auto tokens = split(line, ' ');

      if (firstChar == 'e') {
        parseEdge(tokens);
      } else if (firstChar == 'n') {
        parseNode(tokens);
      } else if (firstChar == 'g') {
        parseAttributes(tokens);
      }
    }
  }

  std::optional<State> transition(const State& state,
                                  const Action& action) const {
    if (state.getId() != action.getSourceStateId()) {
      return {};
    }
    LOG(INFO) << "Current: state.id " << state.getId()
              << " next: " << action.getTargetStateId()
              << " Cost: " << action.getActionCost();

    return State{action.getTargetStateId()};
  }

  bool isGoal(const State& state) const { return state.getId() == goalStateId; }

  Cost distance(const State& state) const {
    return nodes[state.getId()].distance;
  }

  Cost heuristic(const State& state) const {
    return nodes[state.getId()].heuristic;
  }

  std::vector<SuccessorBundle<Graph>> successors(const State& state) const {
    const size_t sourceNodeId = state.getId();

    const Node& sourceNode = nodes[sourceNodeId];

    std::vector<SuccessorBundle<Graph>> successors{};

    for (auto edge : sourceNode.edges) {
      successors.emplace_back(State{edge.target},
                              Action{sourceNodeId, edge.target, edge.cost},
                              edge.cost);
    }

    return successors;
  }

  const State getStartState() const { return State{startStateId}; }

  Cost getActionDuration() const { return 0; }

  bool safetyPredicate(const State& state) const {
    throw MetronomeException("Graph state does not have a safety predicate");
  }

 private:
  struct Edge;

  struct Node {
    Cost heuristic{0};
    Cost distance{0};
    std::vector<Edge> edges{0};
  };

  struct Edge {
    Edge() : id{0} {}
    Edge(size_t id, size_t target, Cost cost)
        : id{id}, target{target}, cost{cost} {}
    size_t id;
    size_t target{0};
    Cost cost{0};
  };

  void parseEdge(const std::vector<std::string>& tokens) {
    if (tokens.size() < 5) {
      throw MetronomeException("Invalid edge!");
    }

    addEdge((size_t)std::stoi(tokens[1]),
            (size_t)std::stoi(tokens[2]),
            (size_t)std::stoi(tokens[3]),
            std::stoi(tokens[4]));
  }

  void parseNode(const std::vector<std::string>& tokens) {
    if (tokens.size() < 4) {
      throw MetronomeException("Invalid node!");
    }

    addNode((size_t)std::stoi(tokens[1]),
            std::stoi(tokens[2]),
            std::stoi(tokens[3]));
  }

  void parseAttributes(const std::vector<std::string>& tokens) {
    if (tokens[0] != "graph") {
      throw MetronomeException("Graph attributes can't start with: " +
                               tokens[0]);
    }

    if (tokens.size() < 4) {
      throw MetronomeException("Graph attributes are invalid");
    }

    nodeCount = (size_t)std::stoi(tokens[1]);
    startStateId = (size_t)std::stoi(tokens[2]);
    goalStateId = (size_t)std::stoi(tokens[3]);

    // Initialize graph
    nodes = std::vector<Node>{nodeCount + 1};
  }

  void addNode(size_t id, Cost heuristic, Cost distance) {
    Node& node = nodes[id];
    node.heuristic = heuristic;
    node.distance = distance;
  }

  void addEdge(size_t edgeId, size_t sourceId, size_t targetId, Cost cost) {
    Node& node = nodes[sourceId];
    node.edges.emplace_back(edgeId, targetId, cost);
  }

  size_t nodeCount;
  size_t startStateId;
  size_t goalStateId;

  std::vector<Node> nodes;
};
}  // namespace metronome

#endif  // METRONOME_GRAPH_HPP
