#pragma once

#include <cmath>
#include <experiment/Configuration.hpp>
#include <functional>
#include <limits>
#include <optional>
#include <ostream>
#include <string>
#include <vector>
#include "SuccessorBundle.hpp"

namespace metronome {

class UrbanDriving {
 public:
  typedef long long int Cost;

  static constexpr double TIME_RESOLUTION = 1.0;
  static constexpr double SPEED_RESOLUTION = 1.0;

  class Action {
   public:
    Action() {}
    Action(double acceleration, bool isComfortable)
        : acceleration(acceleration), isComfortable(isComfortable) {}

    std::string toString() const {
      // TODO
    }

    double acceleration;
    bool isComfortable;
  };

  class State {
   public:
    State(size_t spatialStateIndex, double time, double speed)
        : spatialStateIndex(spatialStateIndex), time(time), speed(speed) {}

    bool operator==(const State& state) const {
      return this->spatialStateIndex == state.spatialStateIndex &&
             this->getDiscreteTime() == state.getDiscreteTime() &&
             this->getDiscreteSpeed() == state.getDiscreteSpeed();
    }

    std::size_t hash() const {
      //      static const auto hash = std::hash<double>{};
      //      return hash(time) ^ hash(speed) ^ spatialStateIndex;
      return getDiscreteSpeed() ^ getDiscreteTime() ^ spatialStateIndex;
    }

    std::size_t getDiscreteTime() const {
      return static_cast<std::size_t>(time / TIME_RESOLUTION);
    }

    std::size_t getDiscreteSpeed() const {
      return static_cast<std::size_t>(speed / SPEED_RESOLUTION);
    }

    friend std::ostream& operator<<(std::ostream& os, const State& state) {
      os << "spatialStateIndex: " << state.spatialStateIndex
         << " time: " << state.time << " speed: " << state.speed;
      return os;
    }

    std::size_t spatialStateIndex;
    double time;
    double speed;
  };

  class SpatialState {
   public:
    explicit SpatialState(double distance) : distance(distance) {}
    double distance;
  };

  UrbanDriving(const Configuration& configuration, std::istream& input) {
    auto spatialDistances = configuration.getDoubles("spatialDistances");
    
    spatialStates.reserve(spatialDistances.size());
    
    for (double distance : spatialDistances) {
      spatialStates.emplace_back(distance);
    }
  }

  std::optional<State> transition(const State& state,
                                  const Action& action) const {
    // We can't move beyond the last spatial state;
    if (state.spatialStateIndex == spatialStates.size() - 1) return {};

    // Ignore identity action and moving backward
    if (state.speed < 1.0e-8 && action.acceleration < 1.0e-5) return {};

    const auto targetStateIndex = state.spatialStateIndex + 1;

    double distance = spatialStates[targetStateIndex].distance -
                      spatialStates[state.spatialStateIndex].distance;

    double squaredFinalSpeed =
        2 * action.acceleration * distance + state.speed * state.speed;

    double finalSpeed;

    // Clip speed at 0
    if (squaredFinalSpeed < 1.0e-5) {
      finalSpeed = 0;
    } else {
      finalSpeed = std::sqrt(squaredFinalSpeed);
    }

    double averageSpeed = (finalSpeed + state.speed) / 2.0;
    assert(averageSpeed > 1.0e-6);

    double time = distance / averageSpeed;

    return {State(targetStateIndex, state.time + time, finalSpeed)};
  }

  bool isGoal(const State& state) const {
    return state.spatialStateIndex == spatialStates.size() - 1;
//    &&
//           state.speed < 1.0e-5;
  }

  Cost distance(const State& state) const { return 0; }

  Cost heuristic(const State& state) const { return 0; }

  std::vector<SuccessorBundle<UrbanDriving>> successors(const State& state) 
  const {
    if (state.spatialStateIndex == spatialStates.size() - 1) return {};

    static const std::vector<Action> actions = {
        Action(1, true), Action(0, true), Action(-1, true), Action(-2, false)};

    std::vector<SuccessorBundle<UrbanDriving>> successors;
    successors.reserve(actions.size());

    for (const auto& action : actions) {
      const auto& targetState = transition(state, action);
      if (targetState.has_value()) {
        // TODO cost
        successors.emplace_back(targetState.value(), action, 1);
      }
    }

    return successors;
  }

  const State getStartState() const { return {0, 0, 0}; }

  Cost getActionDuration() const {
    // TODO
  }

  Action getIdentityAction() const {
    throw MetronomeException("UrbanDriving does not have an identity action!");
  }

  bool isSafe(const State& state) const { return state.speed < 1.0e-5; }

  bool less(const State& lhs, const State& rhs) {
    // 1. Goal distance
    if (lhs.spatialStateIndex > rhs.spatialStateIndex) return true;
    if (lhs.spatialStateIndex < rhs.spatialStateIndex) return false;

    // 2. Time
    if (lhs.time < rhs.time) return true;
    if (lhs.time > rhs.time) return false;

    // 3. Distance
    if (lhs.speed < rhs.speed) return true;
    if (lhs.speed > rhs.speed) return false;
  }

 private:
  std::vector<SpatialState> spatialStates;
};

}  // namespace metronome
