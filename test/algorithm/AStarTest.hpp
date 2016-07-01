#include "algorithm/AStar.hpp"
#include <catch.hpp>
#include <domains/VacuumWorld.hpp>
#include <easylogging++.h>

namespace {

class TestDomain {
public:
    class State {
    public:
        State() {
        }

        std::size_t hash() const {
            return 0;
        }

        bool operator==(const State& state) const {
            return true;
        }
    };

    class Action {};
    typedef unsigned long Cost;

    Cost heuristic(State&) {
        return 0;
    }

    bool isGoal() {
        return false;
    }
};

//TEST_CASE("AStar test", "[AStar]") {
//    TestDomain testDomain;
//    metronome::AStar<TestDomain> aStar(testDomain);
//
//    TestDomain::State state;
//
//    aStar.plan(state);
//}

TEST_CASE("AStar - VacuumWorld test", "[VacuumWorld]") {
    VacuumWorld testDomain;
    metronome::AStar<VacuumWorld> aStar(testDomain);

    std::vector<VacuumWorld::Action> ret = aStar.plan(testDomain.getStartLocation());
    LOG(INFO) << "Solution: " << std::endl;
    for (auto i : ret) {
        LOG(INFO) << i.evaluate() << std::endl;
    }
}
}
