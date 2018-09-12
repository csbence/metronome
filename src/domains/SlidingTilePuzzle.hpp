#pragma once

#include <MetronomeException.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/optional.hpp>
#include <experiment/Configuration.hpp>
#include <limits>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>
#include "SuccessorBundle.hpp"

namespace metronome {

class SlidingTilePuzzle {
 public:
  typedef long long int Cost;

  class Action {
   public:
    Action() : label{'~'} {}

    Action(char value) : label{value} {}

    static std::vector<Action>& getActions() {
      static std::vector<Action> actions{
          Action('N'), Action('E'), Action('W'), Action('S')};
      return actions;
    }

    bool operator==(const Action& rhs) const { return label == rhs.label; }

    bool operator!=(const Action& rhs) const { return !(rhs == *this); }

    char toChar() const { return label; }

    static Action getIdentity() { return Action('0'); }

    std::string toString() const { return std::string{1, label}; }

    friend std::ostream& operator<<(std::ostream& os, const Action& action) {
      os << action.label;
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
    bool operator==(const State& state) const {
      return state.zeroIndex == zeroIndex && state.tiles == tiles;
    }

    bool operator!=(const State& state) const { return state.tiles != tiles; }

    std::size_t hash() const { return (size_t)tiles >> 32 ^ tiles; }

    std::string toString() const {
      std::ostringstream stream;

      const int totalSize = size * size;

      for (int i = 0; i < totalSize; ++i) {
        stream << (*this)[i] << " ";
      }

      return stream.str();
    }

    unsigned char operator[](const unsigned char index) const {
      return static_cast<unsigned char>(tiles >> (index * 4) & 0xF);
    }

    void set(const unsigned char index, const unsigned char value) {
      tiles = (tiles & ~(15ULL << (index * 4))) |
              static_cast<unsigned long long int>(value) << (index * 4);
    }

    unsigned char getZeroIndex() const { return zeroIndex; }

    void setZeroIndex(unsigned char zeroIndex) { this->zeroIndex = zeroIndex; }

    unsigned long long int getTiles() const { return tiles; }

    void setTiles(unsigned long long int tiles) { this->tiles = tiles; }

    friend std::ostream& operator<<(std::ostream& os, const State& state) {
      os << "zero: " << static_cast<int>(state.zeroIndex)
         << " tiles: " << state.tiles;
      return os;
    }

   private:
    unsigned char zeroIndex{0};
    unsigned long long int tiles{0};
  };

  SlidingTilePuzzle(const Configuration& configuration, std::istream& input)
      : actionDuration(configuration.getLongOrThrow(ACTION_DURATION)) {
    using namespace std;

    unsigned char dimension{0};
    string line;

    // Read dimensions
    getline(input, line);
    std::vector<std::string> dimensions;

    if (line.empty()) {
      throw MetronomeException(
          "First line of the domain description can't be empty. Path might be "
          "invalid.");
    }

    boost::split(dimensions, line, boost::is_any_of(" "));

    if (dimensions.size() != 2) {
      throw MetronomeException("Only two dimensional puzzles are supported.");
    }

    if (dimensions[0] != dimensions[1]) {
      throw MetronomeException("Only square puzzles are supported.");
    }

    dimension = static_cast<unsigned char>(stoi(dimensions[0]));

    if (dimension != 4) {
      throw MetronomeException("Only 4 by 4 puzzles are supported.");
    }

    getline(input, line);  // skip one line

    // Read start state
    for (unsigned char i = 0; i < size * size; ++i) {
      getline(input, line);
      const int intValue = stoi(line);
      unsigned char value = static_cast<unsigned char>(intValue);
      startState.set(i, value);

      if (value == 0) {
        startState.setZeroIndex(i);
      }
    }
  }

  boost::optional<State> transition(const State& state,
                                    const Action& action) const {
    switch (action.toChar()) {
      case 'N':
        return getSuccessor(state, 0, -1);
      case 'E':
        return getSuccessor(state, 1, 0);
      case 'W':
        return getSuccessor(state, -1, 0);
      case 'S':
        return getSuccessor(state, 0, 1);
      case '0':
        return boost::make_optional(state);
      default:
        return boost::none;
    }
  }

  bool isGoal(const State& state) const { return distance(state) == 0; }

  Cost distance(const State& state) const {
    int manhattanSum{0};

    for (unsigned char x = 0; x < size; ++x) {
      for (unsigned char y = 0; y < size; ++y) {
        auto value = state[getIndex(x, y)];
        if (value == 0) {
          continue;
        }

        manhattanSum += abs(value / size - y) + abs(value % size - x);
      }
    }

    return manhattanSum;
  }

  Cost heuristic(const State& state) const {
    return distance(state) * actionDuration;
  }

  bool safetyPredicate(const State& state) const { return true; }

  std::vector<SuccessorBundle<SlidingTilePuzzle>> successors(
      const State& state) const {
    std::vector<SuccessorBundle<SlidingTilePuzzle>> successors;

    addValidSuccessor(successors, state, 0, -1, Action::getActions()[0]);
    addValidSuccessor(successors, state, 0, 1, Action::getActions()[3]);
    addValidSuccessor(successors, state, -1, 0, Action::getActions()[2]);
    addValidSuccessor(successors, state, 1, 0, Action::getActions()[1]);

    return successors;
  }

  const State getStartState() const { return startState; }

  Cost getActionDuration() const { return actionDuration; }

 private:
  static signed char getIndex(signed char x, signed char y) {
    return size * y + x;
  }

  void addValidSuccessor(
      std::vector<SuccessorBundle<SlidingTilePuzzle>>& successors,
      const State& sourceState,
      signed char relativeX,
      signed char relativeY,
      Action action) const {
    const boost::optional<State>& successor =
        getSuccessor(sourceState, relativeX, relativeY);
    if (successor.is_initialized()) {
      successors.emplace_back(successor.get(), action, actionDuration);
    }
  }

  boost::optional<State> getSuccessor(const State& sourceState,
                                      signed char relativeX,
                                      signed char relativeY) const {
    signed char targetZeroIndex =
        static_cast<signed char>(sourceState.getZeroIndex()) +
        getIndex(relativeX, relativeY);

    if (targetZeroIndex >= 0 && targetZeroIndex < size * size) {
      State targetState{sourceState};

      // Move tile
      targetState.set(targetState.getZeroIndex(),
                      targetState[static_cast<unsigned char>(targetZeroIndex)]);
      targetState.set(static_cast<unsigned char>(targetZeroIndex), 0);
      targetState.setZeroIndex(static_cast<unsigned char>(targetZeroIndex));

      return boost::optional<State>{targetState};
    }

    return boost::none;
  }

  const Cost actionDuration;
  static const unsigned char size{4};
  State startState;
};

}  // namespace metronome
