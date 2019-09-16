#pragma once

#include <algorithm>
#include <cstdlib>
#include <experiment/Configuration.hpp>
#include <functional>
#include <limits>
#include <optional>
#include <ostream>
#include <sstream>
#include <unordered_set>
#include <utils/Hash.hpp>
#include <vector>
#include "MetronomeException.hpp"
#include "SuccessorBundle.hpp"

namespace metronome {
class GridWorld {
 public:
  typedef long long int Cost;

  class Action {
   public:
    Action() : dY(0), aY(0){};
    Action(short dVX, short dVY) : dY(dVX), aY(dVY){};
    Action(const Action&) = default;
    Action(Action&&) = default;
    Action& operator=(const Action&) = default;
    ~Action() = default;

    static std::vector<Action>& getActions() {
      static std::vector<Action> actions{Action(-1, 1),
                                         Action(0, 1),
                                         Action(1, 1),
                                         Action(-1, 0),
                                         Action(0, 0),
                                         Action(1, 0),
                                         Action(-1, -1),
                                         Action(0, -1),
                                         Action(1, -1)};
      return actions;
    }

    short getAX() const { return dY; }
    short getAY() const { return aY; }

    Action inverse() const {
      throw MetronomeException("Domain is not invertable!");
    }

    bool operator==(const Action& rhs) const {
      return dY == rhs.dY && aY == rhs.aY;
    }

    bool operator!=(const Action& rhs) const { return !(rhs == *this); }

    friend std::ostream& operator<<(std::ostream& os, const Action& action) {
      os << " (ax: " << action.dY << " ay: " << action.aY << ")";
      return os;
    }

   private:
    short dY;
    short aY;
  };

  class State {
   public:
    State() : x(0), y(0), vX(0), vY(0) {}
    State(unsigned int x, unsigned int y, short vX, short vY)
        : x(x), y(y), vX(vX), vY(vY) {}
    /*Standard getters for the State(x,y)*/
    unsigned int getX() const { return x; }
    unsigned int getY() const { return y; }
    short getVX() const { return vX; }
    short getVY() const { return vY; }

    std::size_t hash() const {
      return x ^ y << 16 ^ y >> 16 ^ vX << 2 ^ vY << 8;
    }

    bool operator==(const State& state) const {
      return x == state.x && y == state.y && vX == state.vX && vY == state.vY;
    }

    bool operator!=(const State& state) const { return !(*this == state); }

    State& operator=(State toCopy) {
      swap(*this, toCopy);
      return *this;
    }

    friend std::ostream& operator<<(std::ostream& stream,
                                    const GridWorld::State& state) {
      stream << "x: " << state.getX() << " y: " << state.getY()
             << " vx: " << state.vX << " vy: " << state.vY;
      return stream;
    }

   private:
    /*State(x,y) representation*/
    unsigned int x;
    unsigned int y;
    short vX;
    short vY;

    /*Function facilitating operator==*/
    friend void swap(State& first, State& second) {
      using std::swap;
      swap(first.x, second.x);
      swap(first.y, second.y);
      swap(first.vX, second.vX);
      swap(first.vY, second.vY);
    }
  };

  struct LocationHash {
    size_t operator()(const State& state) const {
      size_t seed = 37;

      seed = seed * 17 + std::hash<unsigned int>{}(state.getX());
      seed = seed * 17 + std::hash<unsigned int>{}(state.getY());
      return seed;
    }
  };

  struct LocationEquals {
    bool operator()(const State& lhs, const State& rhs) const {
      return lhs.getX() == rhs.getX() && lhs.getY() == rhs.getY();
    }
  };

  /*Entry point for using this Domain*/
  GridWorld(const Configuration& configuration, std::istream& input)
      : actionDuration(configuration.getLong(ACTION_DURATION)),
        heuristicMultiplier(configuration.getDouble(HEURISTIC_MULTIPLIER)) {
    unsigned int currentHeight = 0;
    unsigned int currentWidth = 0;
    std::string line;
    char* end;
    getline(input, line);  // get the width

    std::stringstream convertWidth(line);
    if (std::strtol(line.c_str(), &end, 10) == 0) {
      throw MetronomeException("GridWorld first line must be a number.");
    }

    convertWidth >> width;
    getline(input, line);  // get the height
    if (std::strtol(line.c_str(), &end, 10) == 0) {
      throw MetronomeException("GridWorld second line must be a number.");
    }

    std::stringstream convertHeight(line);
    convertHeight >> height;

    std::optional<State> tempStarState;

    while (getline(input, line) && currentHeight < height) {
      for (char it : line) {
        if (it == '@') {  // find the start location
          tempStarState = State(currentWidth, currentHeight, 0, 0);
        } else if (it == '*') {  // find the goal location
          abstractGoalStates.emplace_back(currentWidth, currentHeight);
        } else if (it == '#') {  // store the objects
          State obstacle(currentWidth, currentHeight, 0, 0);
          obstacles.insert(obstacle);
        } else {
          // its an open cell nothing needs to be done
        }
        if (currentWidth == width) break;

        ++currentWidth;  // at the end of the character parse move along
      }
      if (currentWidth < width) {
        throw MetronomeException(
            "RaceTrack is not complete. Width doesn't match input "
            "configuration.");
      }

      currentWidth = 0;  // restart character parse at beginning of line
      ++currentHeight;   // move down one line in charadter parse
    }

    if (currentHeight != height) {
      throw MetronomeException(
          "RaceTrack is not complete. Height doesn't match input "
          "configuration.");
    }

    if (!tempStarState.has_value() || abstractGoalStates.empty()) {
      throw MetronomeException(
          "RaceTrack unknown start or goal location. Start or goal location is "
          "not defined.");
    }

    startLocation = tempStarState.value();
  }

  /**
   * Transition to a new state from the given state applying the given action.
   *
   * @return the original state if the transition is not possible
   */
  std::optional<State> transition(const State& sourceState,
                                  const Action& action) const {
    short vX = sourceState.getVX() + action.getAX();
    short vY = sourceState.getVY() + action.getAY();

    // Limiting maximum valid speed
    if (std::abs(vX) > maxSpeed || std::abs(vY) > maxSpeed) return {};

    State targetState(sourceState.getX() + vX, sourceState.getY() + vY, vX, vY);

    if (isLegalLocation(targetState)) {
      return targetState;
    }

    return {};
  }

  /*Validating a goal state*/
  bool isGoal(const State& location) const {
    for (const auto& goal : abstractGoalStates) {
      if (goal.getX() == location.getX() && goal.getY() == location.getY()) {
        return true;
      }
    }
    return false;
  }

  /*Validating an obstacle state*/
  bool isObstacle(const State& location) const {
    return obstacles.find(location) != obstacles.end();
  }

  /*Validating the agent can visit the state*/
  bool isLegalLocation(const State& state) const {
    return state.getX() < width && state.getX() >= 0 && state.getY() < height &&
           state.getY() >= 0 && !isObstacle(state);
  }

  /*Standard getters for the (width,height) of the domain*/
  unsigned int getWidth() const { return width; }
  unsigned int getHeight() const { return height; }

  /*Adding an obstacle to the domain*/
  bool addObstacle(const State& toAdd) {
    if (isLegalLocation(toAdd)) {
      obstacles.insert(toAdd);
      return true;
    }
    return false;
  }

  std::vector<State>::size_type getNumberObstacles() {
    return obstacles.size();
  }

  State getStartState() const { return startLocation; }

  bool isStart(const State& state) const {
    return state.getX() == startLocation.getX() &&
           state.getY() == startLocation.getY();
  }

  Cost distance(const State& state, const State& other) const {
    unsigned int verticalDistance = std::max(other.getY(), state.getY()) -
                                    std::min(other.getY(), state.getY());

    unsigned int horizontalDistance = std::max(other.getX(), state.getX()) -
                                      std::min(other.getX(), state.getX());

    unsigned int totalDistance =
        verticalDistance / maxSpeed + horizontalDistance / maxSpeed;

    return static_cast<Cost>(totalDistance);
  }
  
  Cost safetyDistance(const State& state) const {
    return std::max(state.getVX(), state.getVY());
  }

  Cost distance(const State& state) const {
    Cost minDistance = std::numeric_limits<Cost>::max();

    for (const auto& goal : abstractGoalStates) {
      minDistance = std::min(distance(goal, state), minDistance);
    }

    return minDistance;
  }

  Cost heuristic(const State& state) const {
    return distance(state) * actionDuration;
  }

  Cost heuristic(const State& state, const State& other) const {
    return distance(state, other) * actionDuration;
  }

  bool isSafe(const State& state) const {
    return state.getVX() == 0 && state.getVY() == 0;
  }

  std::vector<SuccessorBundle<GridWorld>> successors(const State& state) const {
    std::vector<SuccessorBundle<GridWorld>> successors;
    successors.reserve(9);

    for (auto& action : Action::getActions()) {
      addValidSuccessor(successors, state, action);
    }

    return successors;
  }

  Cost getActionDuration() const { return actionDuration; }
  Cost getActionDuration(const Action& action) const { return actionDuration; }

  Action getIdentityAction() const {
    throw MetronomeException("RaceTrack does not have an identity action");
  }

 private:
  void addValidSuccessor(std::vector<SuccessorBundle<GridWorld>>& successors,
                         const State& sourceState,
                         Action action) const {
    auto successor = transition(sourceState, action);

    if (successor.has_value()) {
      successors.emplace_back(successor.value(), action, actionDuration);
    }
  }

  const int maxSpeed = 10;

  /*
   * maxActions <- maximum number of actions
   * width/height <- internal size representation of world
   * obstacles <- stores locations of the objects in world
   * dirtyCells <- stores locations of dirt in the world
   * startLocation <- where the agent begins
   * goalLocation <- where the agent needs to end up
   * initalAmountDirty <- how many cells are dirty
   * initialCost <- constant cost value
   * obstacles <- stores references to obstacles
   */
  unsigned int width;
  unsigned int height;
  std::unordered_set<State, LocationHash, LocationEquals> obstacles;
  State startLocation{};
  std::vector<State> abstractGoalStates;
  const Cost actionDuration;
  const double heuristicMultiplier;
};

}  // namespace metronome
