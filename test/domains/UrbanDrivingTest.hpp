#include "catch.hpp"
#include "domains/UrbanDriving.hpp"
#include "easylogging++.h"

namespace metronome {
namespace {

TEST_CASE("Successor generation", "[UrbanDriving]") {
  Configuration configuration;
  std::fstream fileInputStream;

  UrbanDriving urbanDriving(configuration, fileInputStream);

  auto startState = urbanDriving.getStartState();

  REQUIRE(urbanDriving.successors(startState).size() > 0);
}

TEST_CASE("Physics test", "[UrbanDriving]") {
  const char* json = "{\"spatialDistances\" : [0, 1, 2]}";
  
  Configuration configuration(json);
  std::fstream fileInputStream;

  UrbanDriving urbanDriving(configuration, fileInputStream);

  auto startState = urbanDriving.getStartState();
  UrbanDriving::Action positive(1, true);
  UrbanDriving::Action negative(-1, true);
  UrbanDriving::Action zero(0, true);

  auto positiveSuccessor = urbanDriving.transition(startState, positive);
  auto negativeSuccessor = urbanDriving.transition(startState, negative);
  auto zeroSuccessor = urbanDriving.transition(startState, zero);

  REQUIRE(positiveSuccessor.has_value());
  // 
  REQUIRE_FALSE(negativeSuccessor.has_value());
  REQUIRE_FALSE(zeroSuccessor.has_value());

  std::cout << positiveSuccessor.value() << std::endl;

  auto positiveSuccessor2 =
      urbanDriving.transition(positiveSuccessor.value(), positive);
  auto negativeSuccessor2 =
      urbanDriving.transition(positiveSuccessor.value(), negative);
  auto zeroSuccessor2 = urbanDriving.transition(positiveSuccessor.value(), 
      zero);

  REQUIRE(positiveSuccessor2.has_value());
  REQUIRE(negativeSuccessor2.has_value());
  REQUIRE(zeroSuccessor2.has_value());
  
  std::cout << positiveSuccessor2.value() << std::endl;
  std::cout << negativeSuccessor2.value() << std::endl;
  std::cout << zeroSuccessor2.value() << std::endl;
}

}  // namespace
}  // namespace metronome
