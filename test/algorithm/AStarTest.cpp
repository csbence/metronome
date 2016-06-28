#include "algorithm/AStar.hpp"
#include <catch.hpp>
#include <domains/TestDomain.hpp>
#include <domains/VacuumWorld.hpp>
#include <easylogging++.h>

namespace {

    TEST_CASE("AStar test", "[AStar]") {

        VacuumWorld testDomain;
        metronome::AStar<VacuumWorld> aStar(testDomain);

        std::vector<VacuumWorld::Action> ret = aStar.plan(testDomain.getStartLocation());
        LOG(INFO) << "Solution: " << std::endl;
        for (auto i : ret) {
            LOG(INFO) << i.evaluate() << std::endl;
        }

    }
}
