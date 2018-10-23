#pragma once

/**
 *  Super classes for nodes
 */

#include <sstream>
#include <string>

namespace metronome {

template <typename Domain>
class SearchNode {
 public:
  typedef typename Domain Domain;
  typedef typename Domain::State State;
  typedef typename Domain::Action Action;
  typedef typename Domain::Cost Cost;

  SearchNode(SearchNode* parent,
             const State& state,
             Action action,
             Cost actionCost,
             Cost g,
             Cost h,
             bool open)
      : parent{parent},
        state{state},
        action{std::move(action)},
        actionCost{actionCost},
        g{g},
        h{h},
        open{open} {}

  Cost f() const { return g + h; }

  unsigned long hash() const { return state.hash(); }

  bool operator==(const SearchNode& node) const { return state == node.state; }

  virtual std::string toString() const {
    std::ostringstream stream;
    stream << "s: " << state << " g: " << g << " h: " << h << " f: " << f()
           << " a: " << action << " p: ";
    if (parent == nullptr) {
      stream << "None";
    } else {
      stream << parent->state;
    }
    if (open) {
      stream << " Open";
    } else {
      stream << (closed ? " Closed" : " Not Open");
    }
    return stream.str();
  }

  /** Index used by the priority queue */
  mutable unsigned int index{std::numeric_limits<unsigned int>::max()};
  /** Parent node */
  SearchNode* parent;
  /** Internal state */
  const State state;
  /** Action that led to the current node from the parent node */
  Action action;
  /** Cost of the action */
  Cost actionCost;
  /** Cost from the root node */
  Cost g;
  /** Heuristic cost of the node */
  Cost h;
  /** True if the node is in the open list */
  bool open;
  /** True if the node has been popped from the open list */
  bool closed{false};
};

template <typename Domain>
class RealtimeSearchNode : public SearchNode<Domain> {
 public:
  RealtimeSearchNode(SearchNode* parent,
                     const State& state,
                     Action action,
                     Cost actionCost,
                     Cost g,
                     Cost h,
                     bool open,
                     unsigned int iteration = 0)
      : SearchNode(parent, state, action, actionCost, g, h, open),
        iteration{iteration} {}
  unsigned int iteration;

  virtual std::string to_string() const override {
    std::ostringstream stream;
    stream << SearchNode::to_string() << " Iteration " << iteration;
    return stream.str();
  }
};

template <typename Domain>
struct Edge {
 public:
  using Action = typename SearchNode<Domain>::Action;
  using Cost = typename SearchNode<Domain>::Cost;

  SearchNode<Domain>* predecessor;
  SearchNode<Domain>* successor;
  Action action;
  Cost actionCost;
};

}  // namespace metronome