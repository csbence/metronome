#include "catch.hpp"
#include "domains/SlidingTilePuzzle.hpp"
#include "easylogging++.h"
#include "experiment/ConfigurationExecutor.hpp"

namespace {

using namespace metronome;


const std::string json = "{\"actionDuration\" : 6000000}";

const std::string map{"4 4\n"
                      "starting positions for each tile:\n"
                      "9\n"
                      "11\n"
                      "10\n"
                      "15\n"
                      "12\n"
                      "7\n"
                      "8\n"
                      "3\n"
                      "13\n"
                      "6\n"
                      "14\n"
                      "4\n"
                      "5\n"
                      "1\n"
                      "0\n"
                      "2\n"
                      "goal positions:\n"
                      "0\n"
                      "1\n"
                      "2\n"
                      "3\n"
                      "4\n"
                      "5\n"
                      "6\n"
                      "7\n"
                      "8\n"
                      "9\n"
                      "10\n"
                      "11\n"
                      "12\n"
                      "13\n"
                      "14\n"
                      "15"};

const std::string mapKorf3{"4 4\n"
                      "starting positions for each tile:\n"
                      "14\n"
                      "7\n"
                      "8\n"
                      "2\n"
                      "13\n"
                      "11\n"
                      "10\n"
                      "4\n"
                      "9\n"
                      "12\n"
                      "5\n"
                      "0\n"
                      "3\n"
                      "6\n"
                      "1\n"
                      "15\n"
                      "goal positions:\n"
                      "0\n"
                      "1\n"
                      "2\n"
                      "3\n"
                      "4\n"
                      "5\n"
                      "6\n"
                      "7\n"
                      "8\n"
                      "9\n"
                      "10\n"
                      "11\n"
                      "12\n"
                      "13\n"
                      "14\n"
                      "15"};

const std::string mapKorf100{"4 4\n"
                           "starting positions for each tile:\n"
                           "11\n"
                           "4\n"
                           "0\n"
                           "8\n"
                           "6\n"
                           "10\n"
                           "5\n"
                           "13\n"
                           "12\n"
                           "7\n"
                           "14\n"
                           "3\n"
                           "1\n"
                           "2\n"
                           "9\n"
                           "15\n"
                           "goal positions:\n"
                           "0\n"
                           "1\n"
                           "2\n"
                           "3\n"
                           "4\n"
                           "5\n"
                           "6\n"
                           "7\n"
                           "8\n"
                           "9\n"
                           "10\n"
                           "11\n"
                           "12\n"
                           "13\n"
                           "14\n"
                           "15"};

const std::string mapTopLeft{"4 4\n"
                             "starting positions for each tile:\n"
                             "0\n"
                             "1\n"
                             "2\n"
                             "3\n"
                             "4\n"
                             "5\n"
                             "6\n"
                             "7\n"
                             "8\n"
                             "9\n"
                             "10\n"
                             "11\n"
                             "12\n"
                             "13\n"
                             "14\n"
                             "15\n"
                             "goal positions:\n"
                             "0\n"
                             "1\n"
                             "2\n"
                             "3\n"
                             "4\n"
                             "5\n"
                             "6\n"
                             "7\n"
                             "8\n"
                             "9\n"
                             "10\n"
                             "11\n"
                             "12\n"
                             "13\n"
                             "14\n"
                             "15"};

const std::string mapBottomRight{"4 4\n"
                             "starting positions for each tile:\n"
                             "15\n"
                             "1\n"
                             "2\n"
                             "3\n"
                             "4\n"
                             "5\n"
                             "6\n"
                             "7\n"
                             "8\n"
                             "9\n"
                             "10\n"
                             "11\n"
                             "12\n"
                             "13\n"
                             "14\n"
                             "0\n"
                             "goal positions:\n"
                             "0\n"
                             "1\n"
                             "2\n"
                             "3\n"
                             "4\n"
                             "5\n"
                             "6\n"
                             "7\n"
                             "8\n"
                             "9\n"
                             "10\n"
                             "11\n"
                             "12\n"
                             "13\n"
                             "14\n"
                             "15"};

TEST_CASE("SlidingTilePuzzle state manipulation test", "[SlidingTilePuzzle]") {
    SlidingTilePuzzle<4>::State state;

    for (unsigned char i = 0; i < 16; ++i) {
      for (unsigned char j = 0; j < 16; ++j) {
        state[i] = j;
        REQUIRE(state[i] == j);
      }
    }

    for (unsigned char i = 0; i < 16; ++i) {
      state[i] = 16;
    }

    for (unsigned char i = 0; i < 16; ++i) {
      REQUIRE(state[i] == 16);
    }
}

TEST_CASE("SlidingTilePuzzle heuristic tests", "[SlidingTilePuzzle]") {
  SECTION("Korf 4x4 instance 4") {
    Configuration configuration{json};

    std::istringstream stringStream{mapKorf3};
    std::istream inputStream{stringStream.rdbuf()};
    SlidingTilePuzzle<4> tiles{configuration, inputStream};

    REQUIRE(tiles.distance(tiles.getStartState()) == 41);
  }
  
  SECTION("Korf 4x4 instance 100") {
    Configuration configuration{json};

    std::istringstream stringStream{mapKorf100};
    std::istream inputStream{stringStream.rdbuf()};
    SlidingTilePuzzle<4> tiles{configuration, inputStream};

    REQUIRE(tiles.distance(tiles.getStartState()) == 38);
  }
}

TEST_CASE("SlidingTilePuzzle successor tests", "[SlidingTilePuzzle]") {
  SECTION("TopLeft") {
    Configuration configuration{json};

    std::istringstream stringStream{mapTopLeft};
    std::istream inputStream{stringStream.rdbuf()};
    SlidingTilePuzzle<4> tiles{configuration, inputStream};

    auto startState = tiles.getStartState();

    REQUIRE(startState.zeroIndex() == 0);
    auto successors = tiles.successors(startState);
    REQUIRE(successors.size() == 2);
    
    for (const auto successor : successors) {
      const uint8_t zeroIndex = successor.state.zeroIndex();

      const bool is1 = zeroIndex == 1;
      const bool is4 = zeroIndex == 4;
      const bool valid = is1 || is4;
      
      REQUIRE(valid);
    }
  }
  
  SECTION("BottomRight") {
    Configuration configuration{json};

    std::istringstream stringStream{mapBottomRight};
    std::istream inputStream{stringStream.rdbuf()};
    SlidingTilePuzzle<4> tiles{configuration, inputStream};

    auto startState = tiles.getStartState();

    REQUIRE(startState.zeroIndex() == 15);
    auto successors = tiles.successors(startState);
    REQUIRE(successors.size() == 2);

    for (const auto successor : successors) {
      const uint8_t zeroIndex = successor.state.zeroIndex();

      const bool is11 = zeroIndex == 11;
      const bool is14 = zeroIndex == 14;
      const bool valid = is11 || is14;

      REQUIRE(valid);
    }
  }
}

TEST_CASE("SlidingTilePuzzle domain tests", "[SlidingTilePuzzle]") {
    Configuration configuration{json};

    std::istringstream stringStream{map};
    std::istream inputStream{stringStream.rdbuf()};
    SlidingTilePuzzle<4> tiles{configuration, inputStream};

    SECTION("Successor function test") {
        SlidingTilePuzzle<4>::State state = tiles.getStartState();

        auto successors = tiles.successors(state);
        REQUIRE(successors.size() == 3);

//        REQUIRE(successors[0].state.getTiles() != 0ULL);
//        REQUIRE(successors[1].state.getTiles() != 0ULL);
    }

    SECTION("Goal state test") {
        SlidingTilePuzzle<4>::State state;

        for (uint8_t i = 0; i < 16; ++i) {
            state[i] = i;
        }

        state.zeroIndex() = 0;

        REQUIRE(tiles.isGoal(state));
    }
}

}
