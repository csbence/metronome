#pragma once

#include <ostream>
#include <string>
#include <vector>
#include "Planner.hpp"
#include "experiment/termination/TimeTerminationChecker.hpp"

namespace metronome {

template <typename Domain, typename TerminationChecker>
class OnlinePlanner : public Planner {
 public:
  class ActionBundle final {
   public:
    ActionBundle() = default;
    ActionBundle(typename Domain::Action action,
                 typename Domain::Cost actionDuration)
        : action{action}, actionDuration{actionDuration} {}
    ActionBundle(const ActionBundle&) = default;
    ActionBundle(ActionBundle&&) = default;
    ActionBundle& operator=(ActionBundle&) = default;
    ActionBundle& operator=(ActionBundle&&) = default;
    ~ActionBundle() = default;

    friend std::ostream& operator<<(std::ostream& os,
                                    const ActionBundle& bundle) {
      os << "Action: " << bundle.action
         << " expectedTargetState: " << bundle.expectedTargetState
         << " label: " << bundle.label;
      return os;
    }

    typename Domain::Action action;
    typename Domain::Cost actionDuration;
    typename Domain::State expectedTargetState;
    std::string label;
  };

  ~OnlinePlanner() override = default;

  virtual std::vector<ActionBundle> selectActions(
      const typename Domain::State& startState,
      TerminationChecker& terminationChecker) = 0;

  friend std::ostream& operator<<(
      std::ostream& os, const std::vector<ActionBundle>& actionBundles) {
    for (const auto& actionBundle : actionBundles) {
      os << actionBundle << "\n";
    }
    return os;
  }
};

}  // namespace metronome
