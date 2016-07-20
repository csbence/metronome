#ifndef METRONOME_TIMETERMINATIONCHECKERTEST_HPP
#define METRONOME_TIMETERMINATIONCHECKERTEST_HPP

#include <chrono>
#include <thread>
#include "experiment/termination/TimeTerminationChecker.hpp"
namespace {

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace metronome;

TEST_CASE("Termination test", "[TimeTerminationChecker]") {
    TimeTerminationChecker terminationChecker;

    SECTION("Check before termination") {
        terminationChecker.resetTo(1h);
        REQUIRE_FALSE(terminationChecker.reachedTermination());
    }

    SECTION("Check after termination") {
        terminationChecker.resetTo(0ns);
        REQUIRE(terminationChecker.reachedTermination());

        terminationChecker.resetTo(1ns);
        std::this_thread::sleep_for(2ns);

        REQUIRE(terminationChecker.reachedTermination());
    }
}
}

#endif // METRONOME_TIMETERMINATIONCHECKERTEST_HPP
