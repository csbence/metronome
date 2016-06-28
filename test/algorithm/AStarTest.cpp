#include "algorithm/AStar.hpp"
#include <catch.hpp>
#include <domains/TestDomain.hpp>
#include <domains/VacuumWorld.hpp>

namespace {

TEST_CASE("AStar test", "[AStar]") {

    VacuumWorld testDomain;
    metronome::AStar<VacuumWorld> aStar(testDomain);

    aStar.plan(testDomain.getStartLocation());
}
}
