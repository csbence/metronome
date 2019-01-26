#pragma once

#include <algorithm>
#include <cstdlib>
#include <cstring>
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
class OrientationGrid {
  enum CostType {
    CARDINAL, DIAGONAL, ROTATION, DEFAULT
  };

 public:
  typedef long long int Cost;

  static constexpr Cost CARDINAL_MOVEMENT_COST = 1;
  static constexpr double DIAGONAL_MOVEMENT_FACTOR = 1.0; //consider using sqrt 2 * CARDINAL_MOVEMENT_COST
  static constexpr Cost ROTATION_COST = 1;

  class Action {
   public:
    Action() : label{"~"}, costType{DEFAULT} {};
    explicit Action(const char* label, CostType costType) : costType{costType} {
      strcpy(this->label, label);
    }
    Action(const Action&) = default;
    Action(Action&&) = default;
    Action& operator=(const Action&) = default;
    ~Action() = default;

    static std::vector<Action>& getActions() {
      static std::vector<Action> actions{Action("N", CARDINAL), //0
                                         Action("NE", DIAGONAL), //1
                                         Action("NW", DIAGONAL), //2
                                         Action("S", CARDINAL), //3
                                         Action("SE", DIAGONAL), //4
                                         Action("SW", DIAGONAL), //5
                                         Action("W", CARDINAL), //6
                                         Action("E", CARDINAL), //7
                                         //Rotation actions: Left or Right
                                         Action("L", ROTATION), //8
                                         Action("R", ROTATION)}; //9
      return actions;
    }

    static Action& getIdentityAction() {
      static Action identityAction = Action("0", DEFAULT);
      return identityAction;
    }

    static Action& getLeftRotationAction() { return getActions()[8]; }
    static Action& getRightRotationAction() { return getActions()[9]; }

    int relativeX() const {
      if (label[0] == 'W' || label[1] == 'W') return -1;
      if (label[0] == 'E' || label[1] == 'E') return 1;
      return 0;
    }

    int relativeY() const {
      if (label[0] == 'N') return -1;
      if (label[0] == 'S') return 1;
      return 0;
    }

    const char* getOrientation(const char* sourceOrientation) const {
      if (label[0] == 'L') {
        if (strcmp(sourceOrientation, "N") == 0) return "NW";
        if (strcmp(sourceOrientation, "NE") == 0) return "N";
        if (strcmp(sourceOrientation, "NW") == 0) return "W";
        if (strcmp(sourceOrientation, "S") == 0) return "SE";
        if (strcmp(sourceOrientation, "SE") == 0) return "E";
        if (strcmp(sourceOrientation, "SW") == 0) return "S";
        if (strcmp(sourceOrientation, "W") == 0) return "SW";
        if (strcmp(sourceOrientation, "E") == 0) return "NE";
      } else if (label[0] == 'R') {
        if (strcmp(sourceOrientation, "N") == 0) return "NE";
        if (strcmp(sourceOrientation, "NE") == 0) return "E";
        if (strcmp(sourceOrientation, "NW") == 0) return "N";
        if (strcmp(sourceOrientation, "S") == 0) return "SW";
        if (strcmp(sourceOrientation, "SE") == 0) return "S";
        if (strcmp(sourceOrientation, "SW") == 0) return "W";
        if (strcmp(sourceOrientation, "W") == 0) return "NW";
        if (strcmp(sourceOrientation, "E") == 0) return "SE";
      }

      return sourceOrientation;
    }

    const char* getLabel() const { return label; }
    CostType getCostType() const { return costType; }

    Action inverse() const {
      if (strcmp(label, "N") == 0) return getActions()[3];
      if (strcmp(label, "NE") == 0) return getActions()[5];
      if (strcmp(label, "NW") == 0) return getActions()[4];
      if (strcmp(label, "S") == 0) return getActions()[0];
      if (strcmp(label, "SE") == 0) return getActions()[2];
      if (strcmp(label, "SW") == 0) return getActions()[1];
      if (strcmp(label, "W") == 0) return getActions()[7];
      if (strcmp(label, "E") == 0) return getActions()[6];
      if (strcmp(label, "L") == 0) return getActions()[9];
      if (strcmp(label, "R") == 0) return getActions()[8];
      if (strcmp(label, "0") == 0) return getIdentityAction();

      throw MetronomeException("Unknown action to invert: " +
                               std::string(label));
    }

    bool operator==(const Action& rhs) const { return strcmp(label, rhs.label); }

    bool operator!=(const Action& rhs) const { return !(rhs == *this); }

    std::string toString() const { return std::string{label}; }

    friend std::ostream& operator<<(std::ostream& os, const Action& action) {
      os << action.label << " (dx: " << action.relativeX()
         << " dy: " << action.relativeY() << ")";
      return os;
    }

   private:
    char label[3];
    CostType costType;
  };

  class State {
   public:
    State() : x(0), y(0), theta{"N"} {}
    State(unsigned int x, unsigned int y, const char* theta)
        : x{x}, y{y} { strcpy(this->theta, theta); }
    //For storing state locations where orientation doesn't make sense
    State(unsigned int x, unsigned int y) : x{x}, y{y}, theta{"0"} {}
    /*Standard getters for the State(x,y,theta)*/
    unsigned int getX() const { return x; }
    unsigned int getY() const { return y; }
    const char* getTheta() const { return theta; }

    std::size_t hash() const {
      std::size_t seed = x ^ y << 16 ^ y >> 16;
      seed = seed * 17 + std::hash<char>{}(theta[0]);
      seed = seed * 17 + std::hash<char>{}(theta[1]);
      return seed;
    }

    bool operator==(const State& state) const {
      return x == state.x && y == state.y && strcmp(theta, state.theta) == 0;
    }

    bool operator!=(const State& state) const {
      return !(*this == state);
    }

    State& operator=(State toCopy) {
      swap(*this, toCopy);
      return *this;
    }

    friend std::ostream& operator<<(std::ostream& stream,
                                    const OrientationGrid::State& state) {
      stream << "x: " << state.getX() << " y: " << state.getY()
             << " theta: " << state.getTheta();
      return stream;
    }

   private:
    /*State(x,y) representation*/
    unsigned int x;
    unsigned int y;
    char theta[3]; //c-style string for conservation of memory
    /*Function facilitating operator=*/
    friend void swap(State& first, State& second) {
      using std::swap;
      swap(first.x, second.x);
      swap(first.y, second.y);
      swap(first.theta, second.theta);
    }
  };

  //hash for obstacles (location only, no orientation)
  struct LocationHash {
    size_t operator()(const State& state) const {
      size_t seed = 37;

      seed = seed * 17 + std::hash<unsigned int>{}(state.getX());
      seed = seed * 17 + std::hash<unsigned int>{}(state.getY());
      return seed;
    }
  };

  //equals for obstacles (location only, no orientation)
  struct LocationEquals {
    bool operator()(const State& lhs, const State& rhs) const {
      return lhs.getX() == rhs.getX() && lhs.getY() == rhs.getY();
    }
  };

  /*Entry point for using this Domain*/
  OrientationGrid(const Configuration& configuration, std::istream& input)
      : actionDuration(configuration.getLong(ACTION_DURATION)), 
      heuristicMultiplier(configuration.getDouble(HEURISTIC_MULTIPLIER)) {
    
    //init cost values
    double durationAsDouble = static_cast<double>(actionDuration);
    double diagonalCostAsDouble = DIAGONAL_MOVEMENT_FACTOR * durationAsDouble;
    Cost diagonalCost = static_cast<Cost>(diagonalCostAsDouble);

    costValues[CARDINAL] = CARDINAL_MOVEMENT_COST * actionDuration;
    costValues[DIAGONAL] = diagonalCost;
    costValues[ROTATION] = ROTATION_COST * actionDuration;
    costValues[DEFAULT] = actionDuration;

    //parse domain
    unsigned int currentHeight = 0;
    unsigned int currentWidth = 0;
    std::string line;
    char* end;
    getline(input, line);  // get the width

    std::stringstream convertWidth(line);
    if (std::strtol(line.c_str(), &end, 10) == 0) {
      throw MetronomeException("OrientationWorld first line must be a number.");
    }

    convertWidth >> width;
    getline(input, line);  // get the height
    if (std::strtol(line.c_str(), &end, 10) == 0) {
      throw MetronomeException("OrientationWorld second line must be a number.");
    }

    std::stringstream convertHeight(line);
    convertHeight >> height;

    std::optional<State> tempStartState;
    std::optional<State> tempGoalState;

    while (getline(input, line) && currentHeight < height) {
      for (char it : line) {
        if (it == '@') {  // find the start location
          // default orientation to N
          tempStartState = State(currentWidth, currentHeight, "N");
        } else if (it == '*') {  // find the goal location
          tempGoalState = State(currentWidth, currentHeight);
        } else if (it == '#') {  // store the objects
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
            "OrientationWorld is not complete. Width doesn't match input "
            "configuration.");
      }

      currentWidth = 0;  // restart character parse at beginning of line
      ++currentHeight;   // move down one line in charadter parse
    }

    if (currentHeight != height) {
      throw MetronomeException(
          "OrientationWorld is not complete. Height doesn't match input "
          "configuration.");
    }

    if (!tempStartState.has_value() || !tempGoalState.has_value()) {
      throw MetronomeException(
          "OrientationWorld unknown start or goal location. Start or goal location is "
          "not defined.");
    }

    startLocation = tempStartState.value();
    goalLocation = tempGoalState.value();
  }

  /**
   * Transition to a new state from the given state applying the given action.
   *
   * @return no value if the transition is not possible
   */
  std::optional<State> transition(const State& sourceState,
                                  const Action& action) const {
    if (!isLegalTransition(sourceState, action)) {
      return {};
    }

    State targetState(sourceState.getX() + action.relativeX(),
                      sourceState.getY() + action.relativeY(),
                      action.getOrientation(sourceState.getTheta()));

    if (isLegalLocation(targetState)) {
      return targetState;
    }

    return {};
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
  /* Validating orientation.
   * We can only move forward and backward or rotate
   * Allow identity action as special case
   */
  bool isLegalTransition(const State& sourceState, const Action& action) const {
    return strcmp(action.getLabel(), "L") == 0 ||
           strcmp(action.getLabel(), "R") == 0 ||
           strcmp(action.getLabel(), "0") == 0 ||
           strcmp(sourceState.getTheta(), action.getLabel()) == 0 ||
           strcmp(sourceState.getTheta(), action.inverse().getLabel()) == 0;
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

  /**
   * Heuristic function: Orthogonal(ish) distance with orientation
   * Determine vertical direction of goal
   * Determine horizontal direction of goal
   * Determine possible diagonal movements and subtract from manahattan distance
   * Determine orientation changes needed to start and transition from
   *  diagonals
   * Return estimate scaling each type of action by its configured cost
   *  (Types of actions: cardinal movements, diagonal movements, rotations)
   */
  Cost heuristic(const State& state) const {
    return heuristic(state, goalLocation);
  }
  Cost heuristic(const State& state, const State& otherState) const {
    bool goalIsNorth = otherState.getY() < state.getY();
    bool goalIsSouth = otherState.getY() > state.getY();
    bool goalIsWest = otherState.getX() < state.getX();
    bool goalIsEast = otherState.getX() > state.getX();
    // return early if we are at the goal
    if ((goalIsNorth || goalIsSouth || goalIsWest || goalIsEast) == false) {
      return static_cast<Cost>(0);
    }

    unsigned int rotations = 0; //init expected rotations

    unsigned int verticalDistance = std::max(otherState.getY(), state.getY()) -
                                    std::min(otherState.getY(), state.getY());
    unsigned int horizontalDistance =
        std::max(otherState.getX(), state.getX()) -
        std::min(otherState.getX(), state.getX());

    unsigned int manhattanDistance = verticalDistance + horizontalDistance;
    unsigned int diagonalMoves = std::min(verticalDistance, horizontalDistance);
    unsigned int cardinalMoves = manhattanDistance - (2*diagonalMoves); //no underflow possible

    //if we move diagonally then have to rotate to face the goal,
    //that means at least one rotation
    if (diagonalMoves > 0 && cardinalMoves > 0) rotations++;

    //determine target rotations
    char direction[3];
    char inverseDirection[3];
    unsigned int index = 0;

    if (goalIsNorth) {
      direction[index] = 'N';
      inverseDirection[index] = 'S';
      index++;
    }
    else if (goalIsSouth) {
      direction[index] = 'S';
      inverseDirection[index] = 'N';
      index++;
    }

    if (goalIsWest) {
      direction[index] = 'W';
      inverseDirection[index] = 'E';
      index++;
    }
    else if (goalIsEast){
      direction[index] = 'E';
      inverseDirection[index] = 'W';
      index++;
    }

    direction[index] = '\0'; //terminate string
    inverseDirection[index] = '\0';

    Action& rotateLeft = Action::getLeftRotationAction();
    Action& rotateRight = Action::getRightRotationAction();

    //initialize left (counterclockwise) and right (clockwise) rotations from current state
    char leftOrientation[3];
    char rightOrientation[3];

    strcpy(leftOrientation, state.getTheta());
    strcpy(rightOrientation, state.getTheta());

    //find minimum rotations to reach required initial orientation
    auto x = state.getX();
    auto y = state.getY();
    while (strcmp(leftOrientation, direction) != 0 &&
           strcmp(rightOrientation, direction) != 0 &&
           strcmp(leftOrientation, inverseDirection) != 0 &&
           strcmp(rightOrientation, inverseDirection) != 0) {
      rotations++;

      strcpy(leftOrientation, rotateLeft.getOrientation(leftOrientation));
      strcpy(rightOrientation, rotateRight.getOrientation(rightOrientation));
    }

    Cost estimate = (costValues[CARDINAL] * static_cast<Cost>(cardinalMoves)) +
                    (costValues[DIAGONAL] * static_cast<Cost>(diagonalMoves)) +
                    (costValues[ROTATION] * static_cast<Cost>(rotations));
    return estimate * heuristicMultiplier;
  }

  bool safetyPredicate(const State&) const { return true; }

  std::vector<SuccessorBundle<OrientationGrid>> successors(const State& state) const {
    std::vector<SuccessorBundle<OrientationGrid>> successors;
    successors.reserve(4); // Forward, backward, rotate left, rotate right

    for (auto& action : Action::getActions()) {
      addValidSuccessor(successors, state, action);
    }

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
  Cost getActionDuration(const Action& action) const { return costValues[action.getCostType()]; }

  Action getIdentityAction() const { return Action::getIdentityAction(); }

 private:
  void addValidSuccessor(
      std::vector<SuccessorBundle<OrientationGrid>>& successors,
                         const State& sourceState,
                         Action& action) const {
    if (!isLegalTransition(sourceState, action)) return;
    auto successor = getSuccessor(sourceState, action);
    if (successor.has_value()) {
      successors.emplace_back(successor.value(), action, costValues[action.getCostType()]);
    }
  }

  std::optional<State> getSuccessor(const State& sourceState, Action& action) const {
    auto newX = static_cast<unsigned int>(static_cast<int>(sourceState.getX()) +
                                          action.relativeX());
    auto newY = static_cast<unsigned int>(static_cast<int>(sourceState.getY()) +
                                          action.relativeY());
    const char* newOrientation = action.getOrientation(sourceState.getTheta());

    State newState = State(newX, newY, newOrientation);

    if (isLegalLocation(newState)) {
      return newState;
    }

    return {};
  }

  /*
   * maxActions <- maximum number of actions
   * width/height <- internal size representation of world
   * obstacles <- stores locations of the objects in world
   * startLocation <- where the agent begins
   * goalLocation <- where the agent needs to end up
   * actionDuration <- constant cost value for 1 "action unit"
   * costValues <- Stores the actual cost values for actions of different types
   */
  unsigned int width;
  unsigned int height;
  std::unordered_set<State, LocationHash, LocationEquals>
      obstacles{};
  State startLocation{};
  State goalLocation{};
  const Cost actionDuration;
  Cost costValues[4];
  const double heuristicMultiplier;
};

}  // namespace metronome
