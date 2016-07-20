#include "algorithms/AStar.hpp"
#include "catch.hpp"
#include "domains/GridWorld.hpp"
#include "domains/Vehicle.hpp"
#include "easylogging++.h"
#include "experiment/Configuration.hpp"

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

// TEST_CASE("AStar test", "[AStar]") {
//    TestDomain testDomain;
//    metronome::AStar<TestDomain> aStar(testDomain);
//
//    TestDomain::State state;
//
//    aStar.plan(state);
//}

TEST_CASE("AStar - GridWorld test", "[GridWorld]") {
    /*    metronome::GridWorld testDomain;
        metronome::Configuration config;
        metronome::AStar<metronome::GridWorld> aStar(testDomain, config);

        std::vector<metronome::GridWorld::Action> ret = aStar.plan(testDomain.getStartState());
        LOG(INFO) << "Solution: " << std::endl;
        for (auto i : ret) {
            LOG(INFO) << i.toString() << std::endl;
        }
    */
}

/*TEST_CASE("AStar - Vehicle test", "[Vehicle]") {
    Vehicle testDomain;
    metronome::AStar<Vehicle> aStar(testDomain);

    std::vector<GridWorld::Action> ret = aStar.plan(testDomain.getStartLocation());
    LOG(INFO) << "Solution: " << std::endl;
    for (auto i : ret) {
        LOG(INFO) << i.evaluate() << std::endl;
    }
}$A*/
}
