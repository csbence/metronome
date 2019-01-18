#pragma once

#include <MetronomeException.hpp>
#include <experiment/Configuration.hpp>
#include <limits>
#include <optional>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>
#include "SuccessorBundle.hpp"

namespace metronome {

template <std::size_t DIMENSION, bool HEAVY = false>
class SlidingTilePuzzle {
 public:
  using Cost = long long int;

  class Action {
   public:
    Action() : label{'~'} {};
    explicit Action(char label) : label{label} {}
    Action(const Action&) = default;
    Action(Action&&) = default;
    Action& operator=(const Action&) = default;
    ~Action() = default;

    static std::vector<Action>& getActions() {
      static std::vector<Action> actions{
          Action('N'), Action('S'), Action('W'), Action('E')};
      return actions;
    }

    int relativeX() const {
      if (label == 'N') return 0;
      if (label == 'S') return 0;
      if (label == 'W') return -1;
      if (label == 'E') return 1;
      if (label == '0') return 0;
      return 0;
    }

    int relativeY() const {
      if (label == 'N') return -1;
      if (label == 'S') return 1;
      if (label == 'W') return 0;
      if (label == 'E') return 0;
      if (label == '0') return 0;
      return 0;
    }

    Action inverse() const {
      if (label == 'N') return Action('S');
      if (label == 'S') return Action('N');
      if (label == 'W') return Action('E');
      if (label == 'E') return Action('W');
      if (label == '0') return Action('0');

      throw MetronomeException("Unknown action to invert: " +
                               std::to_string(label));
    }

    bool operator==(const Action& rhs) const { return label == rhs.label; }

    bool operator!=(const Action& rhs) const { return !(rhs == *this); }

    char toChar() const { return label; }

    friend std::ostream& operator<<(std::ostream& os, const Action& action) {
      os << action.label << " (dx: " << action.relativeX()
         << " dy: " << action.relativeY() << ")";
      return os;
    }

   private:
    char label;
  };

  /**
   * State representation of a 4x4 sliding tile puzzle.
   */
  class State {
   public:
    State() { tiles.resize(DIMENSION * DIMENSION); }

    bool operator==(const State& state) const {
      return state.zeroIndex() == zeroIndex() && state.tiles == tiles;
    }

    bool operator!=(const State& state) const { return !(*this == state); }

    std::size_t hash() const {
      std::size_t hash = 0;

      for (const uint8_t tile : tiles) {
        hash = hash << 1;
        hash ^= tile;
      }

      return hash;
    }

    uint8_t& operator[](std::size_t index) { return tiles[index]; }
    const uint8_t& operator[](std::size_t index) const { return tiles[index]; }

    uint8_t& zeroIndex() { return zeroTileIndex; }
    const uint8_t& zeroIndex() const { return zeroTileIndex; }

    friend std::ostream& operator<<(std::ostream& os, const State& state) {
      os << "zero: " << state.zeroIndex() << " tiles: ";

      for (std::size_t i = 0; i < DIMENSION * DIMENSION; ++i) {
        if (i % DIMENSION == 0) os << "\n";
        os << i << (i < 10 ? "  " : " ");
      }

      os << "\n";
      return os;
    }

    // To ensure compatibility with the visualizer
    unsigned int getX() const { return 0; }
    unsigned int getY() const { return 0; }

   private:
    uint8_t zeroTileIndex = 0;
    std::vector<uint8_t> tiles;
  };

  SlidingTilePuzzle(const Configuration& configuration, std::istream& input)
      : actionDuration(configuration.getLong(ACTION_DURATION)) {
    using namespace std;

    string line;

    // Read dimensions
    getline(input, line);

    if (line.empty()) {
      throw MetronomeException(
          "First line of the domain description can't be empty. Path might be "
          "invalid.");
    }

    std::istringstream firstLine(line);

    std::size_t width;
    std::size_t height;
    firstLine >> width;
    firstLine >> height;

    if (width != height) {
      throw MetronomeException("Only square puzzles are supported.");
    }

    if (DIMENSION != width) {
      throw MetronomeException("Invalid dimension!");
    }

    getline(input, line);  // skip one line

    // Read start state
    for (unsigned char i = 0; i < DIMENSION * DIMENSION; ++i) {
      getline(input, line);
      const int intValue = stoi(line);
      startState[i] = static_cast<uint8_t>(intValue);

      if (static_cast<uint8_t>(intValue) == 0) {
        startState.zeroIndex() = i;
      }
    }
  }

  /**
   * Transition to a new state from the given state applying the given action.
   *
   * @return the original state if the transition is not possible
   */
  std::optional<State> transition(const State& sourceState,
                                  const Action& action) const {
    return getSuccessor(sourceState, action.relativeX(), action.relativeY());
  }

  bool isGoal(const State& state) const { return distance(state) == 0; }

  Cost distance(const State& state) const {
    int manhattanSum{0};

    for (int x = 0; x < DIMENSION; ++x) {
      for (int y = 0; y < DIMENSION; ++y) {
        auto value = state[getIndex(x, y)];
        if (value == 0) {
          continue;
        }

        const int endX = value % DIMENSION;
        const int endY = value / DIMENSION;

        manhattanSum += std::abs(endY - y) + std::abs(endX - x);
      }
    }

    return manhattanSum;
  }

  Cost heuristic(const State& state, const State& otherState) const {
    throw MetronomeException("State-to-state heuristic not implemented in Sliding Tile");
  }

  Cost heuristic(const State& state) const {
    Cost manhattanSum{0};

    for (int x = 0; x < DIMENSION; ++x) {
      for (int y = 0; y < DIMENSION; ++y) {
        auto value = state[getIndex(x, y)];
        if (value == 0) {
          continue;
        }

        const int endX = value % DIMENSION;
        const int endY = value / DIMENSION;

        const int manhattanDistance = std::abs(endY - y) + std::abs(endX - x);

        if (HEAVY) {
          manhattanSum += manhattanDistance * value;
        } else {
          manhattanSum += manhattanDistance;
        }
      }
    }

    return manhattanSum * actionDuration;
  }

  bool safetyPredicate(const State&) const { return true; }

  std::vector<SuccessorBundle<SlidingTilePuzzle>> successors(
      const State& state) const {
    std::vector<SuccessorBundle<SlidingTilePuzzle>> successors;
    successors.reserve(4);

    for (auto& action : Action::getActions()) {
      addValidSuccessor(
          successors, state, action.relativeX(), action.relativeY(), action);
    }

    return successors;
  }

  const State getStartState() const { return startState; }

  Cost getActionDuration() const { return actionDuration; }

  Action getIdentityAction() const { return Action('0'); }

 private:
  int getIndex(int x, int y) const {
    return static_cast<int>(DIMENSION * y + x);
  }

  void addValidSuccessor(
      std::vector<SuccessorBundle<SlidingTilePuzzle>>& successors,
      const State& sourceState,
      int relativeX,
      int relativeY,
      Action action) const {
    auto successor = getSuccessor(sourceState, relativeX, relativeY);
    
    if (successor.has_value()) {
      const auto successorState = successor.value();
      Cost actionCost;
      
      if(HEAVY) {
        actionCost = successorState[sourceState.zeroIndex()] * actionDuration;
      } else {
        actionCost = actionDuration;
      }

      successors.emplace_back(successorState, action, actionDuration);
    }
  }

  std::optional<State> getSuccessor(const State& sourceState,
                                    int relativeX,
                                    int relativeY) const {
    const auto sourceZeroIndex = sourceState.zeroIndex();

    int targetZeroIndex = sourceZeroIndex + getIndex(relativeX, relativeY);

    const auto sourceZeroX = sourceZeroIndex % DIMENSION;

    // Bound checks
    // Left
    if (sourceZeroX == 0 && relativeX == -1) return {};
    // Right
    if (sourceZeroX == DIMENSION - 1 && relativeX == 1) return {};
    // Up & Down
    if (targetZeroIndex < 0 || targetZeroIndex >= DIMENSION * DIMENSION)
      return {};

    State targetState{sourceState};

    // Move tile
    assert(targetState[sourceZeroIndex] >= 0 &&
           targetState[sourceZeroIndex] <= DIMENSION * DIMENSION - 1);
    assert(targetState[targetZeroIndex] >= 0 &&
           targetState[targetZeroIndex] <= DIMENSION * DIMENSION - 1);
    std::swap(targetState[sourceZeroIndex], targetState[targetZeroIndex]);
    targetState.zeroIndex() = static_cast<uint8_t>(targetZeroIndex);

    return targetState;
  }

  const Cost actionDuration;
  State startState;
};

}  // namespace metronome
