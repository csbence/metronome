#include "algorithm/AStar.hpp"
#include <catch.hpp>

namespace {

class TestDomain {
public:
    class State {
    public:
        std::size_t hash() const {
            return 0;
        }
    };

    class Action {};
    typedef unsigned long Cost;

    Cost heuristic(State&) {
        return 0;
    }
};

TEST_CASE("AStar test", "[AStar]") {
    TestDomain testDomain;
//    metronome::AStar<TestDomain> aStar(testDomain);

 //   aStar.plan(TestDomain::State());
}
}
