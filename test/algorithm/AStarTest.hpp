#include "algorithm/AStar.hpp"
#include <catch.hpp>
#include <domains/TestDomain.hpp>

namespace {

TEST_CASE("AStar test", "[AStar]") {
    TestDomain testDomain;
    metronome::AStar<TestDomain> aStar(testDomain);

    aStar.plan(TestDomain::State(0));
}
}
