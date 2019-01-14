#include "catch.hpp"
#include "domains/VacuumWorld.hpp"
#include "easylogging++.h"

namespace {
using namespace metronome;

const std::string json = "{\"actionDuration\" : 6000000}";

const std::string map{
    "3\n3\n"
    "@__\n"
    "##*\n"
    "__*\n"};

TEST_CASE("GridWorld", "[GridWorld]") {
  Configuration configuration{json};

  std::istringstream stringStream{map};
  std::istream inputStream{stringStream.rdbuf()};
  VacuumWorld vacuumWorld{configuration, inputStream};

  REQUIRE(vacuumWorld.getWidth() == 3);
  REQUIRE(vacuumWorld.getHeight() == 3);

  SECTION("State copy test") {
    auto startState = vacuumWorld.getStartState();
    auto startStateCopy(startState);

    REQUIRE(startState == startStateCopy);
    REQUIRE(startState.getDirtLocations().size() ==
            startStateCopy.getDirtLocations().size());

    bool removed = startStateCopy.removeDirtCell(VacuumWorld::Location(2, 2));
    REQUIRE(removed);

    REQUIRE(startState.getDirtLocations().size() !=
            startStateCopy.getDirtLocations().size());
    REQUIRE(startState.getDirtLocations() != startStateCopy.getDirtLocations());
    REQUIRE(startState != startStateCopy);
  }
}

}  // namespace
