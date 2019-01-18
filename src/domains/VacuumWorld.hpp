#pragma once

#include <algorithm>
#include <cstdlib>
#include <experiment/Configuration.hpp>
#include <functional>
#include <limits>
#include <optional>
#include <ostream>
#include <unordered_set>
#include <utils/Hash.hpp>
#include <vector>
#include "MetronomeException.hpp"
#include "SuccessorBundle.hpp"

namespace metronome {
class VacuumWorld {
 public:
  typedef long long int Cost;

  class Action {
   public:
    Action() : label{'~'} {};
    explicit Action(char label) : label{label} {}
    Action(const Action &) = default;
    Action(Action &&) = default;
    Action &operator=(const Action &) = default;
    ~Action() = default;

    static std::vector<Action> &getActions() {
      static std::vector<Action> actions{Action('V'),
                                         Action('N'),
                                         Action('S'),
                                         Action('W'),
                                         Action('E'),
                                         Action('D')};
      return actions;
    }

    int relativeX() const {
      if (label == 'N') return 0;
      if (label == 'S') return 0;
      if (label == 'W') return -1;
      if (label == 'E') return 1;
      if (label == 'V') return 0;
      if (label == 'D') return 0;
      if (label == '0') return 0;

      return 0;
    }

    int relativeY() const {
      if (label == 'N') return -1;
      if (label == 'S') return 1;
      if (label == 'W') return 0;
      if (label == 'E') return 0;
      if (label == 'V') return 0;
      if (label == 'D') return 0;
      if (label == '0') return 0;

      return 0;
    }

    Action inverse() const {
      if (label == 'N') return Action('S');
      if (label == 'S') return Action('N');
      if (label == 'W') return Action('E');
      if (label == 'E') return Action('W');
      if (label == 'V') return Action('D');
      if (label == 'D') return Action('V');
      if (label == '0') return Action('0');

      throw MetronomeException("Unknown action to invert: " +
                               std::to_string(label));
    }

    bool operator==(const Action &rhs) const { return label == rhs.label; }

    bool operator!=(const Action &rhs) const { return !(rhs == *this); }

    char toChar() const { return label; }

    friend std::ostream &operator<<(std::ostream &os, const Action &action) {
      os << action.label << " (dx: " << action.relativeX()
         << " dy: " << action.relativeY() << ")";
      return os;
    }

   private:
    char label;
  };

  class Location {
   public:
    Location() : x(0), y(0) {}
    Location(unsigned int x, unsigned int y) : x(x), y(y) {}

    unsigned int getX() const { return x; }
    unsigned int getY() const { return y; }
    std::size_t hash() const { return x ^ y << 16 ^ y >> 16; }

    bool operator==(const Location &rhs) const {
      return x == rhs.x && y == rhs.y;
    }

    bool operator!=(const Location &rhs) const { return !(rhs == *this); }

   private:
    unsigned int x;
    unsigned int y;
  };

  class State {
   public:
    State() : x(0), y(0) {}
    State(unsigned int x, unsigned int y) : x(x), y(y) {}
    State(unsigned int x,
          unsigned int y,
          const std::unordered_set<Location, Hash<Location>> &dirtLocations)
        : x(x), y(y), dirtLocations(dirtLocations) {}
    /*Standard getters for the State(x,y)*/
    unsigned int getX() const { return x; }
    unsigned int getY() const { return y; }

    const std::unordered_set<Location, Hash<Location>> &getDirtLocations()
        const {
      return dirtLocations;
    }

    bool removeDirtCell() {
      auto removedCount = dirtLocations.erase(Location(x, y));

      return removedCount != 0;
      //      if (removedCount == 0) {
      //        throw MetronomeException("Can't remove non existing dirt
      //        cell.");
      //      }
    }

    bool addDirtCell(const Location &location) {
      auto iterator = dirtLocations.find(location);
      if (iterator != dirtLocations.end()) return false;

      dirtLocations.insert(location);
      return true;
    }

    bool addDirtCell() {
      Location currentLocation(x, y);
      return addDirtCell(currentLocation);
    }

    bool hasDirtCell(unsigned int x, unsigned int y) const {
      auto iterator = dirtLocations.find(Location(x, y));
      return iterator != dirtLocations.end();
    }

    std::size_t hash() const { return x ^ y << 16 ^ y >> 16; }

    bool operator==(const State &state) const {
      return x == state.x && y == state.y &&
             state.getDirtLocations() == dirtLocations;
    }

    bool operator!=(const State &rhs) const { return !(rhs == *this); }

    State &operator=(State toCopy) {
      swap(*this, toCopy);
      return *this;
    }

    friend std::ostream &operator<<(std::ostream &stream,
                                    const VacuumWorld::State &state) {
      stream << "x: " << state.getX() << " y: " << state.getY();
      return stream;
    }

   private:
    /*State(x,y) representation*/
    unsigned int x;
    unsigned int y;
    std::unordered_set<Location, Hash<Location>> dirtLocations;

    /*Function facilitating operator==*/
    friend void swap(State &first, State &second) {
      using std::swap;
      swap(first.x, second.x);
      swap(first.y, second.y);
      swap(first.dirtLocations, second.dirtLocations);
    }
  };

  /*Entry point for using this Domain*/
  VacuumWorld(const Configuration &configuration, std::istream &input)
      : actionDuration(configuration.getLong(ACTION_DURATION)),
        heuristicMultiplier(configuration.getDouble(HEURISTIC_MULTIPLIER)) {
    unsigned int currentHeight = 0;
    unsigned int currentWidth = 0;
    std::string line;
    char *end;
    getline(input, line);  // get the width

    std::stringstream convertWidth(line);
    if (std::strtol(line.c_str(), &end, 10) == 0) {
      throw MetronomeException("VacuumWorld first line must be a number.");
    }

    convertWidth >> width;
    getline(input, line);  // get the height
    if (std::strtol(line.c_str(), &end, 10) == 0) {
      throw MetronomeException("VacuumWorld second line must be a number.");
    }

    std::stringstream convertHeight(line);
    convertHeight >> height;

    std::optional<State> tempStarState;
    std::vector<Location> dirtCells;

    while (getline(input, line) && currentHeight < height) {
      for (char it : line) {
        if (it == '@') {  // find the start location
          tempStarState = State(currentWidth, currentHeight);
        } else if (it == '*') {  // find the goal location
          dirtCells.emplace_back(currentWidth, currentHeight);
        } else if (it == '#') {  // store the objects
          Location obstacle = Location(currentWidth, currentHeight);
          obstacles.insert(obstacle);
        } else {
          // its an open cell nothing needs to be done
        }
        if (currentWidth == width) break;

        ++currentWidth;  // at the end of the character parse move along
      }
      if (currentWidth < width) {
        throw MetronomeException(
            "VacuumWorld is not complete. Width doesn't match input "
            "configuration.");
      }

      currentWidth = 0;  // restart character parse at beginning of line
      ++currentHeight;   // move down one line in charadter parse
    }

    if (currentHeight != height) {
      throw MetronomeException(
          "VacuumWorld is not complete. Height doesn't match input "
          "configuration.");
    }

    if (!tempStarState.has_value()) {
      throw MetronomeException(
          "VacuumWorld unknown start. Start location is not defined.");
    }

    if (dirtCells.empty()) {
      throw MetronomeException(
          "VacuumWorld unknown goal. Dirt cell locations are not defined.");
    }

    startState = tempStarState.value();
    for (const auto &dirtCell : dirtCells) {
      startState.addDirtCell(dirtCell);
    }
  }

  /**
   * Transition to a new state from the given state applying the given action.
   *
   * @return the original state if the transition is not possible
   */
  std::optional<State> transition(const State &sourceState,
                                  const Action &action) const {
    if (action.toChar() == 'V') {
      // Attempt to vacuum
      State targetState(sourceState);
      if (targetState.removeDirtCell()) return targetState;
      return {};
    }

    if (action.toChar() == 'D') {
      State targetState(sourceState);
      targetState.addDirtCell();
      return targetState;
    }

    State targetState(sourceState.getX() + action.relativeX(),
                      sourceState.getY() + action.relativeY(),
                      sourceState.getDirtLocations());

    if (isLegalLocation(Location(targetState.getX(), targetState.getY()))) {
      return targetState;
    }

    return {};
  }

  /*Validating a goal state*/
  bool isGoal(const State &state) const {
    return state.getDirtLocations().empty();
  }
  /*Validating an obstacle state*/
  bool isObstacle(const Location &obstacle) const {
    return obstacles.find(obstacle) != obstacles.end();
  }
  /*Validating the agent can visit the state*/
  bool isLegalLocation(const Location &location) const {
    const auto withinBounds =
        location.getX() < width && location.getY() < height;

    return withinBounds && !isObstacle(location);
  }
  /*Standard getters for the (width,height) of the domain*/
  unsigned int getWidth() const { return width; }
  unsigned int getHeight() const { return height; }

  /*Adding an obstacle to the domain*/
  bool addObstacle(const Location &obstacle) {
    if (isLegalLocation(obstacle)) {
      obstacles.insert(obstacle);
      return true;
    }

    return false;
  }

  std::vector<State>::size_type getNumberObstacles() {
    return obstacles.size();
  }

  State getStartState() const { return startState; }

  bool isStart(const State &state) const {
    return state.getX() == startState.getX() &&
           state.getY() == startState.getY();
  }

  // Distance from a goal state
  Cost distance(const State &state) const {
    if (state.getDirtLocations().empty()) return 0;

    auto minX = std::numeric_limits<unsigned int>::max();
    auto minY = std::numeric_limits<unsigned int>::max();
    auto maxX = std::numeric_limits<unsigned int>::min();
    auto maxY = std::numeric_limits<unsigned int>::min();

    for (const auto &dirtLocation : state.getDirtLocations()) {
      minX = std::min(minX, dirtLocation.getX());
      minY = std::min(minY, dirtLocation.getY());
      maxX = std::max(maxX, dirtLocation.getX());
      maxY = std::max(maxY, dirtLocation.getY());
    }

    minX = std::min(minX, state.getX());
    minY = std::min(minY, state.getY());
    maxX = std::max(maxX, state.getX());
    maxY = std::max(maxY, state.getY());

    const auto spanX = maxX - minX;
    const auto spanY = maxY - minY;

    const auto dirtRectangleSize = spanX + spanY;
    return dirtRectangleSize + state.getDirtLocations().size();
  }

  // Distance between states
  Cost distance(const State &state, const State &otherState) const {
    auto manhattan = manhattanDistance(state, otherState);

    auto maxSize = std::max(state.getDirtLocations().size(), otherState.getDirtLocations().size());
    auto minSize = std::min(state.getDirtLocations().size(), otherState.getDirtLocations().size());
    auto sizeDiff = maxSize - minSize;
    if (sizeDiff == 0) {
      return manhattan;
    }

    // We specifically want the state with more dirt so that we
    // can use its dirt spots as benchmarks
    const State &moreDirtState = state.getDirtLocations().size() == maxSize ? state : otherState;

    auto minDistX = std::numeric_limits<unsigned int>::max();
    auto minDistY = std::numeric_limits<unsigned int>::max();

    for (const auto &dirtLocation : moreDirtState.getDirtLocations()) {
      auto maxX = std::max(moreDirtState.getX(), dirtLocation.getX());
      auto maxY = std::max(moreDirtState.getY(), dirtLocation.getY());
      auto minX = std::min(moreDirtState.getX(), dirtLocation.getX());
      auto minY = std::min(moreDirtState.getY(), dirtLocation.getY());

      minDistX = std::min(minDistX, maxX - minX);
      minDistY = std::min(minY, maxY - minY);
    }

    return std::max(manhattan, minDistX + minDistY) + sizeDiff;
  }

  Cost heuristic(const State &state) const {
    return distance(state) * actionDuration;
  }

  Cost heuristic(const State &state, const State &otherState) const {
    return distance(state, otherState) * actionDuration;
  }

  bool safetyPredicate(const State &) const { return true; }

  std::vector<SuccessorBundle<VacuumWorld>> successors(
      const State &state) const {
    std::vector<SuccessorBundle<VacuumWorld>> successors;
    successors.reserve(5);

    for (auto &action : Action::getActions()) {
      addValidSuccessor(
          successors, state, action.relativeX(), action.relativeY(), action);
    }

    return successors;
  }

  void visualize(std::ostream &display) const {
    for (unsigned int i = 0; i < height; ++i) {
      for (unsigned int j = 0; j < width; ++j) {
        if (startState.getX() == j && startState.getY() == i) {
          display << '@';
        } else if (isObstacle(Location(j, i))) {
          display << '#';
        } else {
          display << '_';
        }
      }
      display << "\n";
    }
    display << "\n";
  }

  Cost getActionDuration() const { return actionDuration; }

  Action getIdentityAction() const { return Action('0'); }

 private:
  unsigned int manhattanDistance(const State &state, const State &otherState) const {
    auto minManX = std::min(state.getX(), otherState.getX());
    auto minManY = std::min(state.getY(), otherState.getY());
    auto maxManX = std::max(state.getX(), otherState.getX());
    auto maxManY = std::max(state.getX(), otherState.getX());

    return (maxManX - minManX) + (maxManY - minManY);
  }

  void addValidSuccessor(std::vector<SuccessorBundle<VacuumWorld>> &successors,
                         const State &sourceState,
                         const int relativeX,
                         const int relativeY,
                         Action &action) const {
    if (action.toChar() == 'V') {
      // Attempt to vacuum
      State targetState(sourceState);
      if (targetState.removeDirtCell()) {
        successors.emplace_back(targetState, action, actionDuration);
      }

      return;
    }

    if (action.toChar() == 'D' &&
        startState.hasDirtCell(sourceState.getX(), sourceState.getY())) {
      State targetState(sourceState);
      if (targetState.addDirtCell()) {
        successors.emplace_back(targetState, action, actionDuration);
      }

      return;
    }

    auto successor = getSuccessor(sourceState, relativeX, relativeY);
    if (successor.has_value()) {
      successors.emplace_back(successor.value(), action, actionDuration);
    }
  }

  std::optional<State> getSuccessor(const State &sourceState,
                                    int relativeX,
                                    int relativeY) const {
    auto newX = static_cast<unsigned int>(static_cast<int>(sourceState.getX()) +
                                          relativeX);
    auto newY = static_cast<unsigned int>(static_cast<int>(sourceState.getY()) +
                                          relativeY);

    State newState = State(newX, newY, sourceState.getDirtLocations());

    if (isLegalLocation(Location(newState.getX(), newState.getY()))) {
      return newState;
    }

    return {};
  }

  /*
   * width/height <- internal size representation of world
   * obstacles <- stores locations of the objects in world
   * dirtyCells <- stores locations of dirt in the world
   * startState <- where the agent begins
   * goalLocation <- where the agent needs to end up
   */
  unsigned int width;
  unsigned int height;
  std::unordered_set<Location, Hash<Location>> obstacles;
  State startState;
  const Cost actionDuration;
  const double heuristicMultiplier;
};  // namespace metronome

}  // namespace metronome
