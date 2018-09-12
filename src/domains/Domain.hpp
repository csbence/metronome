#pragma once

#include <experiment/Configuration.hpp>
#include <limits>
#include <string>
#include <vector>
#include "SuccessorBundle.hpp"

namespace metronome {

class Domain {
 public:
  typedef long long int Cost;
  static constexpr Cost COST_MAX = std::numeric_limits<Cost>::max();

  class Action {
   public:
    static Action getIdentity() {
      // TODO
    }

    std::string toString() const {
      // TODO
    }
  };

  class State {
   public:
    bool operator==(const State& state) const {
      // TODO
    }

    std::size_t hash() const {
      // TODO
    }

    std::string toString() const {
      // TODO
    }
  };

  Domain(const Configuration& configuration, std::istream& input) {
    // TODO
  }

  const State transition(const State& state, const Action& action) const {
    // TODO
  }

  bool isGoal(const State& location) const {
    // TODO
  }

  Cost distance(const State& state) const {
    // TODO
  }

  Cost heuristic(const State& state) const {
    // TODO
  }

  std::vector<SuccessorBundle<Domain>> successors(const State& state) const {
    // TODO
  }

  const State getStartState() const {
    // TODO
  }

  Cost getActionDuration() const {
    // TODO
  }

  Action getIdentityAction() const {
    // TODO
  }

  bool safetyPredicate(const State& state) const {
    // TODO
  }
};

}  // namespace metronome
