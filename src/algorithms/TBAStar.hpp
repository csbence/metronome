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
#include "algorithms/planner_tools/Comparators.hpp"
#include "utils/Hash.hpp"
#include "utils/ObjectPool.hpp"
#include "utils/PriorityQueue.hpp"
#include "utils/TimeMeasurement.hpp"

namespace metronome {

/**
 * TBA* implementation.
 * Algorithm performs a single A* search interrupted
 * by the time bound. When interrupted, returns path
 * to the best-so-far node on the search graph
 */
template <typename Domain, typename TerminationChecker>
class TBAStar final : public OnlinePlanner<Domain, TerminationChecker> {
 public:
  typedef typename Domain::State State;
  typedef typename Domain::Action Action;
  typedef typename Domain::Cost Cost;
  typedef typename OnlinePlanner<Domain, TerminationChecker>::ActionBundle
      ActionBundle;

  TBAStar(const Domain& domain, const Configuration& config)
      : domain{domain}, config{config} {
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

    // test
    Node n1{nullptr, startState, Action(), 0, 100, true};
    Node n2{nullptr, startState, Action(), 60, 200, true};

    LOG(INFO) << fComparator(n1, n2) << "\n";
    throw 1;
  }

 private:
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
  };

  typedef int (*Comparator)(const Node&, const Node&);
  static constexpr Comparator fComparator = &fComparator<Node>;

  const Domain& domain;
  const Configuration& config;
  PriorityQueue<Node> openList{Memory::OPEN_LIST_SIZE, fComparator};
  std::unordered_map<State, Node*, typename metronome::Hash<State>> nodes{};
  ObjectPool<Node, Memory::NODE_LIMIT> nodePool;
  unsigned int iterationCounter{0};
};
}  // namespace metronome