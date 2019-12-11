<<<<<<< HEAD

#include "domains/DynamicGridWorld.hpp"

#include "algorithms/AStar.hpp"
#include "utils/TimeMeasurement.hpp"

#include "instances/GridInstance.hpp"

#include <iostream>
#include "catch.hpp"

namespace metronome {
namespace {

const std::string configurationJson =
    "{\"actionDuration\" : 6000000,"
    "\"obstacleCount\" : 10}";

TEST_CASE("GridWorld creation", "[DynamicGridWorld]") {
  el::Loggers::reconfigureAllLoggers(el::ConfigurationType::Format,
                                     "%level: %msg");

  Configuration configuration{configurationJson};

  std::istringstream stringStream{test::slalom};
  std::istream inputStream{stringStream.rdbuf()};

  logTimeDifference("start");
  DynamicGridWorld dynamicGridWorld(configuration, inputStream);
  logTimeDifference("init");

  dynamicGridWorld.expandObstacleDistributionHorizon();
  logTimeDifference("expansion");
  dynamicGridWorld.expandObstacleDistributionHorizon();
  logTimeDifference("expansion");
  dynamicGridWorld.expandObstacleDistributionHorizon();
  logTimeDifference("expansion");

  std::cout << "test" << std::endl;

  REQUIRE(true);
}

TEST_CASE("A* validation", "[DynamicGridWorld]") {
  el::Loggers::reconfigureAllLoggers(el::ConfigurationType::Format,
                                     "%level: %msg");

  Configuration configuration{configurationJson};

  std::istringstream stringStream{test::blocks};
  std::istream inputStream{stringStream.rdbuf()};

  logTimeDifference("start");
  DynamicGridWorld dynamicGridWorld(configuration, inputStream);
  logTimeDifference("init");

  AStar<DynamicGridWorld> aStar(dynamicGridWorld, configuration);
  auto startState = dynamicGridWorld.getStartState();

  const std::vector<GridWorld::Action> plan = aStar.plan(startState);

  std::cout << "test" << std::endl;

  REQUIRE(true);
}

TEST_CASE("print example dynamicGridWorld expansion from start state", "[DynamicGridWorld]") {
  el::Loggers::reconfigureAllLoggers(el::ConfigurationType::Format,
                                     "%level: %msg");

  Configuration configuration{configurationJson};

  std::istringstream stringStream{test::slalom};
  std::istream inputStream{stringStream.rdbuf()};

  logTimeDifference("start");
  DynamicGridWorld const domain(configuration, inputStream);
  auto const start_state = domain.getStartState();
  std::cout << start_state;

  auto const successors = domain.successors(start_state);
  std::cout << successors[0];

}

}  // namespace
}  // namespace metronome
=======

#include <iostream>

#include "algorithms/AStar.hpp"
#include "catch.hpp"
#include "domains/DynamicGridWorld.hpp"
#include "instances/GridInstance.hpp"
#include "utils/TimeMeasurement.hpp"

namespace metronome {
namespace {

const std::string configurationJson =
    "{\"actionDuration\" : 6000000,"
    "\"obstacleCount\" : 10}";

TEST_CASE("GridWorld creation", "[DynamicGridWorld]") {
  el::Loggers::reconfigureAllLoggers(el::ConfigurationType::Format,
                                     "%level: %msg");

  Configuration configuration{configurationJson};

  std::istringstream stringStream{test::slalom};
  std::istream inputStream{stringStream.rdbuf()};

  logTimeDifference("start");
  DynamicGridWorld dynamicGridWorld(configuration, inputStream);
  logTimeDifference("init");

  dynamicGridWorld.expandObstacleDistributionHorizon();
  logTimeDifference("expansion");
  dynamicGridWorld.expandObstacleDistributionHorizon();
  logTimeDifference("expansion");
  dynamicGridWorld.expandObstacleDistributionHorizon();
  logTimeDifference("expansion");

  std::cout << "test" << std::endl;

  REQUIRE(true);
}

TEST_CASE("A* validation", "[DynamicGridWorld]") {
  el::Loggers::reconfigureAllLoggers(el::ConfigurationType::Format,
                                     "%level: %msg");

  Configuration configuration{configurationJson};

  std::istringstream stringStream{test::blocks};
  std::istream inputStream{stringStream.rdbuf()};

  std::cout << "test";

  DynamicGridWorld dynamicGridWorld(configuration, inputStream);

  AStar<DynamicGridWorld> aStar(dynamicGridWorld, configuration);
  auto startState = dynamicGridWorld.getStartState();

  const std::vector<GridWorld::Action> plan = aStar.plan(startState);

  std::cout << "end" << std::endl;

  REQUIRE(true);
}

}  // namespace
}  // namespace metronome
>>>>>>> e7ae9d261c41215579b1d4306888ffcb46caded9
