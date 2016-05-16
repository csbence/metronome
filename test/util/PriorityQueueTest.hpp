#define CATCH_CONFIG_COLOUR_NONE

#include "util/PriorityQueue.hpp"
#include "catch.hpp"
#include "easylogging++.h"

namespace {

class TestNode {};

int nodeCompare(const TestNode &, const TestNode &) {
  return 1; // TODO: actually compare test nodes
}

auto queue = PriorityQueue<TestNode>(100, nodeCompare);

TEST_CASE("PriorityQueue test", "[PriorityQueue]") {
  REQUIRE(queue.getCapacity() == 100);
}
}
