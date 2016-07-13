#include "catch.hpp"
#include "domains/GridWorld.hpp"
#include "easylogging++.h"

namespace {

TEST_CASE("GridWorld creation", "[GridWorld]") {
    GridWorld vacuumWorld;

    REQUIRE(vacuumWorld.getWidth() == 5);
    REQUIRE(vacuumWorld.getHeight() == 5);
}

TEST_CASE("GridWorld::State = operator", "[GridWorld]") {
    GridWorld::State s = GridWorld::State::newState(0, 0);
    GridWorld::State t = GridWorld::State::newState(9, 9);

    REQUIRE(s.getX() == 0);
    REQUIRE(s.getY() == 0);
    REQUIRE(t.getX() == 9);
    REQUIRE(t.getY() == 9);

    s = t;

    REQUIRE(s.getX() == 9);
    REQUIRE(s.getY() == 9);
}

TEST_CASE("GridWorld setting variables", "[GridWorld]") {
    GridWorld vacuumWorld;

    vacuumWorld.setWidth(10);
    vacuumWorld.setHeight(13);
    REQUIRE(vacuumWorld.getWidth() == 10);
    REQUIRE(vacuumWorld.getHeight() == 13);

    GridWorld::State pair1 = GridWorld::State::newState(3, 5);
    GridWorld::State pair2 = GridWorld::State::newState(1, 3);
    REQUIRE(vacuumWorld.getNumberBlockedCells() == 0);
    REQUIRE(vacuumWorld.getNumberDirtyCells() == 1);
    REQUIRE(vacuumWorld.getStartLocation().getX() == 0);
    REQUIRE(vacuumWorld.getStartLocation().getY() == 0);

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
}

TEST_CASE("GridWorld getters", "[GridWorld]") {
    GridWorld vacuumWorld;

    vacuumWorld.setWidth(13);
    vacuumWorld.setHeight(9);

    for (int i = 0; i < 10; i++) {
        std::pair<unsigned int, unsigned int> q = vacuumWorld.randomLocation();
        GridWorld::State _t = GridWorld::State::newState(q.first, q.second);

//        LOG(INFO) << "R_LOC: " << _t.getX() << " " << _t.getY() << std::endl;
        REQUIRE(vacuumWorld.isLegalLocation(_t));
    }
}
}
