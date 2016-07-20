#include "catch.hpp"
#include "domains/GridWorld.hpp"
#include "easylogging++.h"

namespace {

TEST_CASE("GridWorld creation", "[GridWorld]") {
/*    metronome::GridWorld vacuumWorld;

    REQUIRE(vacuumWorld.getWidth() == 5);
    REQUIRE(vacuumWorld.getHeight() == 5);
*/
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
    /*metronome::GridWorld vacuumWorld;

    vacuumWorld.setWidth(10);
    vacuumWorld.setHeight(13);
    REQUIRE(vacuumWorld.getWidth() == 10);
    REQUIRE(vacuumWorld.getHeight() == 13);

    metronome::GridWorld::State pair1 = metronome::GridWorld::State::newState(3, 5);
    metronome::GridWorld::State pair2 = metronome::GridWorld::State::newState(1, 3);
    REQUIRE(vacuumWorld.getNumberObstacles() == 0);
    REQUIRE(vacuumWorld.getNumberDirtyCells() == 1);
    REQUIRE(vacuumWorld.getStartState().getX() == 0);
    REQUIRE(vacuumWorld.getStartState().getY() == 0);

    REQUIRE(pair1.getX() == 3);
    REQUIRE(pair1.getY() == 5);
    REQUIRE(pair2.getX() == 1);
    REQUIRE(pair2.getY() == 3);

//    REQUIRE(vacuumWorld.changeStartLocation(pair1));
//    REQUIRE(vacuumWorld.getStartLocation().getX() == 3);
//    REQUIRE(vacuumWorld.getStartLocation().getY() == 5);

//   REQUIRE(vacuumWorld.changeStartLocation(pair2));
//   REQUIRE(vacuumWorld.getStartLocation().getX() == 1);
//    REQUIRE(vacuumWorld.getStartLocation().getY() == 3);
*/
}

TEST_CASE("GridWorld getters", "[GridWorld]") {
 /*   metronome::GridWorld vacuumWorld;

    vacuumWorld.setWidth(13);
    vacuumWorld.setHeight(9);

    for (int i = 0; i < 10; i++) {
        std::pair<unsigned int, unsigned int> q = vacuumWorld.randomLocation();
        metronome::GridWorld::State _t = metronome::GridWorld::State::newState(q.first, q.second);

//        LOG(INFO) << "R_LOC: " << _t.getX() << " " << _t.getY() << std::endl;
        REQUIRE(vacuumWorld.isLegalLocation(_t));
    }
*/
}
}
