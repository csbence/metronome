#pragma once

#include <ostream>

namespace metronome {

template <typename Domain>
class SuccessorBundle {
 public:
  typedef typename Domain::State State;
  typedef typename Domain::Action Action;
  typedef typename Domain::Cost Cost;

  SuccessorBundle(State state, Action action, Cost actionCost)
      : state(state), action(action), actionCost(actionCost) {}

  const State state;
  const Action action;
  const Cost actionCost;

    friend std::ostream& operator<<(std::ostream& os, const SuccessorBundle& bundle) {
      os << "SuccessorBundle: state: " << bundle.state
         << ", action: " << bundle.action
         << ", cost: " << bundle.actionCost;
      return os;
    }
};

}  // namespace metronome
