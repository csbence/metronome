#pragma once

namespace metronome {

template <typename Domain>
class SuccessorBundle {
 public:
  typedef typename Domain::State State;
  typedef typename Domain::Action Action;
  typedef typename Domain::Cost Cost;

  SuccessorBundle(State state, Action action, Cost actionCost)
      : state(state), action(action), actionCost(actionCost) {}

  SuccessorBundle(const SuccessorBundle&) = default;
  SuccessorBundle& operator=(const SuccessorBundle&) = default;

  const State state;
  const Action action;
  const Cost actionCost;
};

}  // namespace metronome