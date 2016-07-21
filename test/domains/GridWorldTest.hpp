#include <utils/Visualizer.hpp>
#include "catch.hpp"
#include "domains/GridWorld.hpp"
#include "easylogging++.h"
#include "experiment/ConfigurationExecutor.hpp"

namespace {
const char* json = "{\"timeLimit\" : 150000000000,\n"
                   "\"domainPath\" : "
                   "\"/input/vacuum/slalom.vw\",\n"
                   "\"domainInstanceName\" : \"Manual test instance\",\n"
                   "\"actionDuration\" : 6000000,\n"
                   "\"domainName\" : \"GRID_WORLD\",\n"
                   "\"terminationType\" : \"time\",\n"
                   "\"list\" : [1, 2],\n"
                   "\"objects\" : {\"a\": [], \"b\": {}},\n"
                   "\"algorithmName\" : \"A_STAR\"}";

const std::string resourceDir = "/home/aifs2/doylew/Public/metronome/resources";
metronome::GridWorld testGrid =
        metronome::ConfigurationExecutor::extractDomain<metronome::GridWorld>(metronome::Configuration(json),
                resourceDir);

TEST_CASE("GridWorld creation test", "[GridWorld]") {
    metronome::GridWorld gridWorld = testGrid;

    REQUIRE(gridWorld.getWidth() == 13);
    REQUIRE(gridWorld.getHeight() == 9);
}

TEST_CASE("GridWorld::State = operator", "[GridWorld]") {
    metronome::GridWorld::State s = metronome::GridWorld::State(0, 0);
    metronome::GridWorld::State t = metronome::GridWorld::State(9, 9);

    REQUIRE(s.getX() == 0);
    REQUIRE(s.getY() == 0);
    REQUIRE(t.getX() == 9);
    REQUIRE(t.getY() == 9);

    s = t;

    REQUIRE(s.getX() == 9);
    REQUIRE(s.getY() == 9);
}

TEST_CASE("GridWorld setting variables", "[GridWorld]") {
    metronome::GridWorld gridWorld = testGrid;

    metronome::GridWorld::State pair1 = metronome::GridWorld::State(3, 5);
    metronome::GridWorld::State pair2 = metronome::GridWorld::State(1, 3);

    REQUIRE(gridWorld.getNumberObstacles() == 32);
    REQUIRE(gridWorld.getStartState().getX() == 6);
    REQUIRE(gridWorld.getStartState().getY() == 0);

    REQUIRE(pair1.getX() == 3);
    REQUIRE(pair1.getY() == 5);
    REQUIRE(pair2.getX() == 1);
    REQUIRE(pair2.getY() == 3);
}

TEST_CASE("GridWorld getters", "[GridWorld]") {
    metronome::GridWorld gridWorld = testGrid;

    for (int i = 0; i < gridWorld.getHeight(); ++i) {
        for (int j = 0; j < gridWorld.getWidth(); ++j) {
            metronome::GridWorld::State testState = metronome::GridWorld::State(i, j);
            LOG(INFO) << "R_LOC: " << testState.getX() << " " << testState.getY() << std::endl;
//            if (gridWorld.isObstacle(testState)) {
//                REQUIRE(!gridWorld.isLegalLocation(testState));
//            } else {
//                REQUIRE(gridWorld.isLegalLocation(testState));
//            }
        }
    }
}
    TEST_CASE("GridWorld visualization", "[GridWorld]") {
        metronome::GridWorld gridWorld = testGrid;
        metronome::Visualizer<metronome::GridWorld> visualizer(gridWorld);
        visualizer.visualize(std::cout);
    }
}
