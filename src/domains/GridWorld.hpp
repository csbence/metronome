#pragma once

#include <boost/assert.hpp>
#include <boost/optional.hpp>
#include <cstdlib>
#include <experiment/Configuration.hpp>
#include <functional>
#include <limits>
#include <ostream>
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
    Action() : label{'~'} {};

    static std::vector<Action>& getActions() {
      static std::vector<Action> actions{
          Action('N'), Action('S'), Action('W'), Action('E')};
      return actions;
    }

    Action inverse() const {
      if (label == 'N') return Action('S');
      if (label == 'S') return Action('N');
      if (label == 'W') return Action('E');
      if (label == 'E') return Action('W');

      throw MetronomeException("Unknown action to invert: " +
                               std::to_string(label));
    }

    bool operator==(const Action& rhs) const { return label == rhs.label; }

    bool operator!=(const Action& rhs) const { return !(rhs == *this); }

    char toChar() const { return label; }

    static Action getIdentity() { return Action('0'); }

    std::string toString() const { return std::string{label}; }

    friend std::ostream& operator<<(std::ostream& os, const Action& action) {
      os << action.label;
      return os;
    }

   private:
    Action(char label) : label{label} {}
    char label;
  };

  class State {
   public:
    State() : x(0), y(0) {}
    State(unsigned int x, unsigned int y) : x{x}, y{y} {}
    /*Standard getters for the State(x,y)*/
    unsigned int getX() const { return x; }
    unsigned int getY() const { return y; }
    std::size_t hash() const { return x ^ y << 16 ^ y >> 16; }
    bool operator==(const State& state) const {
      return x == state.x && y == state.y;
    }
    bool operator!=(const State& state) const {
      return x != state.x || y != state.y;
    }
    State& operator=(State toCopy) {
      swap(*this, toCopy);
      return *this;
    }
    std::string toString() {
      std::string string{"x: "};
      return string + std::to_string(x) + " y: " + std::to_string(y);
    }

    friend std::ostream& operator<<(std::ostream& stream,
                                    const GridWorld::State& state) {
      stream << "x: " << state.getX() << " y: " << state.getY();
      return stream;
    }

   private:
    /*State(x,y) representation*/
    unsigned int x;
    unsigned int y;
    /*Function facilitating operator==*/
    friend void swap(State& first, State& second) {
      using std::swap;
      swap(first.x, second.x);
      swap(first.y, second.y);
    }
  };
  /*Entry point for using this Domain*/
  GridWorld(const Configuration& configuration, std::istream& input)
      : actionDuration(configuration.getLong(ACTION_DURATION)) {
    obstacles = std::unordered_set<State, typename metronome::Hash<State>>{};
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

    boost::optional<State> tempStarState;
    boost::optional<State> tempGoalState;

    while (getline(input, line) && currentHeight < height) {
      for (auto it = line.cbegin(); it != line.cend(); ++it) {
        if (*it == '@') {  // find the start location
          tempStarState = State(currentWidth, currentHeight);
        } else if (*it == '*') {  // find the goal location
          tempGoalState = State(currentWidth, currentHeight);
        } else if (*it == '#') {  // store the objects
          State object = State(currentWidth, currentHeight);
          obstacles.insert(object);
        } else {
          // its an open cell nothing needs to be done
        }
        if (currentWidth == width) break;
        
        ++currentWidth;  // at the end of the character parse move along
      }
      if (currentWidth < width) {
        throw MetronomeException(
            "GridWorld is not complete. Width doesn't match input "
            "configuration.");
      }

      currentWidth = 0;  // restart character parse at beginning of line
      ++currentHeight;   // move down one line in charadter parse
    }

    if (currentHeight != height) {
      throw MetronomeException(
          "GridWorld is not complete. Height doesn't match input "
          "configuration.");
    }

    if (!tempStarState.is_initialized() || !tempGoalState.is_initialized()) {
      throw MetronomeException(
          "Traffic unknown start or goal location. Start or goal location is "
          "not defined.");
    }

    startLocation = tempStarState.get();
    goalLocation = tempGoalState.get();
  }

  /**
   * Transition to a new state from the given state applying the given action.
   *
   * @return the original state if the transition is not possible
   */
  boost::optional<State> transition(const State& sourceState,
                                    const Action& action) const {
    boost::optional<State> targetState;

    if (action.toChar() == 'N') {
      State newState = State(sourceState.getX(), sourceState.getY() - 1);
      if (isLegalLocation(newState)) {
        targetState = newState;
      }
    } else if (action.toChar() == 'E') {
      State newState = State(sourceState.getX() + 1, sourceState.getY());
      if (isLegalLocation(newState)) {
        targetState = newState;
      }
    } else if (action.toChar() == 'S') {
      State newState = State(sourceState.getX(), sourceState.getY() + 1);
      if (isLegalLocation(newState)) {
        targetState = newState;
      }
    } else if (action.toChar() == 'W') {
      State newState = State(sourceState.getX() - 1, sourceState.getY());
      if (isLegalLocation(newState)) {
        targetState = newState;
      }
    } else if (action.toChar() == '0') {
      return boost::make_optional(sourceState);
    }

    return targetState;
  }
  /*Validating a goal state*/
  bool isGoal(const State& location) const {
    return location.getX() == goalLocation.getX() &&
           location.getY() == goalLocation.getY();
  }
  /*Validating an obstacle state*/
  bool isObstacle(const State& location) const {
    return obstacles.find(location) != obstacles.end();
  }
  /*Validating the agent can visit the state*/
  bool isLegalLocation(const State& location) const {
    return location.getX() < width && location.getY() < height &&
           !isObstacle(location);
  }
  /*Standard getters for the (width,height) of the domain*/
  unsigned int getWidth() { return width; }
  unsigned int getHeight() { return height; }
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

  Cost distance(const State& state) const {
    unsigned int verticalDistance =
        std::max(goalLocation.getY(), state.getY()) -
        std::min(goalLocation.getY(), state.getY());
    unsigned int horizontalDistance =
        std::max(goalLocation.getX(), state.getX()) -
        std::min(goalLocation.getX(), state.getX());
    unsigned int totalDistance = verticalDistance + horizontalDistance;
    Cost manhattanDistance = static_cast<Cost>(totalDistance);
    return manhattanDistance;
  }

  Cost heuristic(const State& state) const {
    return distance(state) * actionDuration;
  }

  bool safetyPredicate(const State&) const { return true; }

  std::vector<SuccessorBundle<GridWorld>> successors(const State& state) const {
    std::vector<SuccessorBundle<GridWorld>> successors;
    successors.reserve(4);

    addValidSuccessor(successors, state, 0, 1, Action::getActions()[1]);
    addValidSuccessor(successors, state, 0, -1, Action::getActions()[0]);
    addValidSuccessor(successors, state, -1, 0, Action::getActions()[2]);
    addValidSuccessor(successors, state, 1, 0, Action::getActions()[3]);

    return successors;
  }

  void visualize(std::ostream& display) const {
    for (unsigned int i = 0; i < height; ++i) {
      for (unsigned int j = 0; j < width; ++j) {
        if (startLocation.getX() == j && startLocation.getY() == i) {
          display << '@';
        } else if (goalLocation.getX() == j && goalLocation.getY() == i) {
          display << '*';
        } else if (isObstacle(State(j, i))) {
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

 private:
  void addValidSuccessor(std::vector<SuccessorBundle<GridWorld>>& successors,
                         const State& sourceState,
                         const int relativeX,
                         const int relativeY,
                         Action& action) const {
    const boost::optional<State>& successor =
        getSuccessor(sourceState, relativeX, relativeY);
    if (successor.is_initialized()) {
      successors.emplace_back(successor.get(), action, actionDuration);
    }
  }

  boost::optional<State> getSuccessor(const State& sourceState,
                                      int relativeX,
                                      int relativeY) const {
    auto newX = static_cast<unsigned int>(static_cast<int>(sourceState.getX()) +
                                          relativeX);
    auto newY = static_cast<unsigned int>(static_cast<int>(sourceState.getY()) +
                                          relativeY);

    State newState = State(newX, newY);

    if (isLegalLocation(newState)) {
      return boost::make_optional(newState);
    }

    return boost::none;
  }

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
  std::unordered_set<State, typename metronome::Hash<State>> obstacles;
  State startLocation{};
  State goalLocation{};
  const Cost actionDuration;
};

}  // namespace metronome
