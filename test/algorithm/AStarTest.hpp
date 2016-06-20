#ifndef METRONOME_ASTARTEST_HPP
#define METRONOME_ASTARTEST_HPP

#include "algorithm/AStar.hpp"
#include <catch.hpp>

namespace {

class TestDomain {
 public:
  class State { };
  class Action { };
  typedef unsigned long Cost;

  Cost heuristic(State&) {
    return 0;
  }

};

TEST_CASE("AStar test", "[AStar]") {
  TestDomain testDomain;
  metronome::AStar<TestDomain> aStar(testDomain);

  aStar.plan(TestDomain::State());
}

}
#endif //METRONOME_ASTARTEST_HPP
