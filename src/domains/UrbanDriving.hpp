#pragma once

#include <algorithm>
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
  typedef double Cost;

  static constexpr double TIME_RESOLUTION = 0.1;
  static constexpr double SPEED_RESOLUTION = 0.1;

  class Action {
   public:
    Action() {}
    Action(double acceleration, bool isComfortable)
        : acceleration(acceleration), isComfortable(isComfortable) {}

    friend std::ostream& operator<<(std::ostream& os, const Action& action) {
      os << "acceleration: " << action.acceleration
         << " isComfortable: " << action.isComfortable;
      return os;
    }

    double acceleration;
    bool isComfortable;
  };

  class State {
   public:
    State() : spatialStateIndex(0), time(0), speed(0) {}
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

    unsigned int getX() const {
      return static_cast<unsigned int>(spatialStateIndex) * 10;
    }

    unsigned int getY() const {
      return static_cast<unsigned int>(getDiscreteSpeed() * 10);// +
//                                       getDiscreteTime());
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

  UrbanDriving(const Configuration& configuration, std::istream& input)
      : speedLimit(configuration.getDouble("speedLimit")),
        startSpeed(configuration.getDouble("startSpeed")) {
    auto spatialDistances = configuration.getDoubles("spatialDistances");
    auto comfortableAccelerations =
        configuration.getDoubles("comfortableAccelerations");
    auto uncomfortableAccelerations =
        configuration.getDoubles("uncomfortableAccelerations");

    spatialStates.reserve(spatialDistances.size());
    this->comfortableAccelerations.reserve(comfortableAccelerations.size());
    this->uncomfortableAccelerations.reserve(uncomfortableAccelerations.size());

    for (double distance : spatialDistances) {
      spatialStates.emplace_back(distance);
    }

    for (double comfortableAcceleration : comfortableAccelerations) {
      this->comfortableAccelerations.push_back(comfortableAcceleration);
    }

    for (double uncomfortableAcceleration : uncomfortableAccelerations) {
      this->uncomfortableAccelerations.push_back(uncomfortableAcceleration);
    }
  }

  std::optional<State> transition(const State& state,
                                  const Action& action) const {
    // We can't move beyond the last spatial state;
    if (state.spatialStateIndex == spatialStates.size() - 1) return {};

    // Ignore identity action and moving backward
    if (state.speed < 1.0e-8 && action.acceleration < 1.0e-5) return {};

    const auto targetStateIndex = state.spatialStateIndex + 1;

    const double distance = spatialStates[targetStateIndex].distance -
                            spatialStates[state.spatialStateIndex].distance;

    const double squaredFinalSpeed =
        2 * action.acceleration * distance + state.speed * state.speed;

    double finalSpeed;

    // Clip speed at 0
    if (squaredFinalSpeed < 1.0e-5) {
      finalSpeed = 0;
    } else {
      finalSpeed = std::sqrt(squaredFinalSpeed);
    }

    if (finalSpeed > speedLimit) return {};

    const double averageSpeed = (finalSpeed + state.speed) / 2.0;
    assert(averageSpeed > 1.0e-6);

    const double time = distance / averageSpeed;

    return {State(targetStateIndex, state.time + time, finalSpeed)};
  }

  std::optional<State> partialInverseTransition(const State& state,
                                                double acceleration) const {
    if (state.spatialStateIndex == 0) return {};
    if (acceleration > 0) return {};

    const auto sourceStateIndex = state.spatialStateIndex - 1;

    const double distance = spatialStates[state.spatialStateIndex].distance -
                            spatialStates[sourceStateIndex].distance;

    const double squaredSourceSpeed =
        -2 * acceleration * distance + state.speed * state.speed;

    const double finalSpeed = std::sqrt(squaredSourceSpeed);
    assert(finalSpeed > state.speed);

    return {State(sourceStateIndex, 0, finalSpeed)};
  }

  bool isGoal(const State& state) const {
    return state.spatialStateIndex == spatialStates.size() - 1 &&
           state.speed < 1.0e-5;
  }

  bool isPartialGoal(const State& state) const {
    return state.spatialStateIndex == spatialStates.size() - 1;
  }

  SuccessorBundle<UrbanDriving> getStopActionBundle(const State& state) const {
    const double velocity = state.speed;
    assert(velocity > 0);

    const double distance = spatialStates[spatialStates.size() - 1].distance -
                            spatialStates[state.spatialStateIndex].distance;

    assert(distance > 0);

    const double a = velocity * velocity / (2 * distance);
    const double time = distance / velocity * 2;

    State finalState(spatialStates.size() - 1, 0, 0);
    Action action(-a, false);

    return {finalState, action, time};
  }

  bool canBeSafe(const State& state) const {
    if (state.spatialStateIndex == spatialStates.size() - 1) return false;

    const auto actionBundle = getStopActionBundle(state);
    assert(!uncomfortableAccelerations.empty());

    // Unsafe
    double min = *(std::min_element(uncomfortableAccelerations.begin(),
                                    uncomfortableAccelerations.end()));

    const auto requiredAcceleration = actionBundle.action.acceleration;

    // Note: negative numbers
    return min < requiredAcceleration;
  }

  std::vector<State> partialPredecessors(const State& state) const {
    std::vector<State> predecessors;
    predecessors.reserve(uncomfortableAccelerations.size());

    for (double uncomfortableAcceleration : uncomfortableAccelerations) {
      auto predecessor =
          partialInverseTransition(state, uncomfortableAcceleration);
      if (predecessor.has_value()) {
        predecessors.push_back(predecessor.value());
      }
    }

    return predecessors;
  }

  State getPartialGoalState() const {
    return State(spatialStates.size() - 1, 0, 0);
  }

  bool isSafetyProof(const State& state, const State& safeState) const {
    return state.spatialStateIndex == safeState.spatialStateIndex &&
           state.speed < safeState.speed;
  }

  Cost distance(const State&) const { return 0; }

  Cost heuristic(const State&) const { return 0; }

  std::vector<SuccessorBundle<UrbanDriving>> successors(
      const State& state) const {
    if (state.spatialStateIndex == spatialStates.size() - 1) return {};

    std::vector<SuccessorBundle<UrbanDriving>> successors;
    successors.reserve(comfortableAccelerations.size());

    for (double acceleration : comfortableAccelerations) {
      Action action(acceleration, true);

      const auto targetState = transition(state, action);
      if (targetState.has_value()) {
        double cost = targetState.value().time - state.time;
        successors.emplace_back(targetState.value(), action, cost);
      }
    }

    return successors;
  }

  std::vector<SuccessorBundle<UrbanDriving>> fastSuccessors(
      const State& state) const {
    if (state.spatialStateIndex == spatialStates.size() - 1) return {};

    std::vector<SuccessorBundle<UrbanDriving>> successors;
    successors.reserve(uncomfortableAccelerations.size());

    for (double acceleration : uncomfortableAccelerations) {
      Action action(acceleration, false);

      const auto targetState = transition(state, action);
      if (targetState.has_value()) {
        double cost = targetState.value().time - state.time;
        successors.emplace_back(targetState.value(), action, cost);
      }
    }

    return successors;
  }

  const State getStartState() const { return {0, 0, startSpeed}; }

  Cost getActionDuration() const {
    // TODO
  }

  Action getIdentityAction() const {
    throw MetronomeException("UrbanDriving does not have an identity action!");
  }

  bool isSafe(const State& state) const { return state.speed < 1.0e-5; }

  bool less(const State& lhs, const State& rhs) const {
    // 1. Goal distance
    if (lhs.spatialStateIndex > rhs.spatialStateIndex) return true;
    if (lhs.spatialStateIndex < rhs.spatialStateIndex) return false;

    // 2. Time
    if (lhs.time < rhs.time) return true;
    if (lhs.time > rhs.time) return false;
    
    // 3. Speed
    if (lhs.speed > rhs.speed) return true;
    if (lhs.speed < rhs.speed) return false;

    return false;
  }

  bool safeLess(const State& lhs, const State& rhs) const {
    // 1. Goal distance
    if (lhs.spatialStateIndex > rhs.spatialStateIndex) return true;
    if (lhs.spatialStateIndex < rhs.spatialStateIndex) return false;

    // 2. Speed
    if (lhs.speed < rhs.speed) return true;
    if (lhs.speed > rhs.speed) return false;

    return false;
  }

 private:
  const double speedLimit;
  const double startSpeed;
  std::vector<SpatialState> spatialStates;
  std::vector<double> comfortableAccelerations;
  std::vector<double> uncomfortableAccelerations;
};

}  // namespace metronome
