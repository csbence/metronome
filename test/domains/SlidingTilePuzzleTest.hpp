#include "catch.hpp"
#include "domains/SlidingTilePuzzle.hpp"
#include "easylogging++.h"
#include "experiment/ConfigurationExecutor.hpp"

namespace {
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

TEST_CASE("SlidingTilePuzzle State manipulation test ", "[SlidingTilePuzzle]") {
    using namespace metronome;

    SlidingTilePuzzle::State state;

    for (unsigned char i = 0; i < 16; ++i) {
        for (unsigned char j = 0; j < 16; ++j) {
            state.set(i, j);
            REQUIRE(state[i] == j);
        }
    }

    for (unsigned char i = 0; i < 16; ++i) {
        state.set(i, 0xF);
    }

    for (unsigned char i = 0; i < 16; ++i) {
        REQUIRE(state[i] == 0xF);
    }

}

TEST_CASE("SlidingTilePuzzle domain tests", "[SlidingTilePuzzle]") {
    using namespace metronome;

    Configuration configuration{json};

    std::istringstream stringStream{map};
    std::istream inputStream{stringStream.rdbuf()};
    SlidingTilePuzzle tiles{configuration, inputStream};

    SECTION("Successor function test") {
        SlidingTilePuzzle::State state = tiles.getStartState();

        auto successors = tiles.successors(state);
        REQUIRE(successors.size() == 3);

        REQUIRE(successors[0].state.getTiles() != 0ULL);
        REQUIRE(successors[1].state.getTiles() != 0ULL);
    }

    SECTION("Goal state test") {
        SlidingTilePuzzle::State state;

        for (unsigned char i = 0; i < 16; ++i) {
            state.set(i, i);
        }

        state.setZeroIndex(0);

        REQUIRE(tiles.isGoal(state));
    }
}
}
