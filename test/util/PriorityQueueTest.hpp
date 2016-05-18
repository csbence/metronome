#define CATCH_CONFIG_COLOUR_NONE

#include "util/PriorityQueue.hpp"
#include "catch.hpp"

namespace {
class TestNode {
 public:
  mutable unsigned int index;
};

int nodeCompare(const TestNode&, const TestNode&) {
  return 1; // TODO: actually compare test nodes
}

auto queue = PriorityQueue<TestNode>(100, nodeCompare);

TEST_CASE("PriorityQueue add/clear test", "[PriorityQueue]") {
  REQUIRE(queue.getCapacity() == 100);

  SECTION("Add items to queue") {
    auto node1 = TestNode();
    auto node2 = TestNode();

    REQUIRE(queue.getSize() == 0);

    queue.push(node1);
    REQUIRE(queue.getSize() == 1);
    REQUIRE(node1.index == 0);

    queue.push(node2);
    REQUIRE(queue.getSize() == 2);
  }

  SECTION("Clear queue") {
    queue.clear();
    REQUIRE(queue.getSize() == 0);
  }

  REQUIRE(queue.getCapacity() == 100);
}

TEST_CASE("PriorityQueue order test", "[PriorityQueue]") {
  REQUIRE(queue.getCapacity() == 100);

  SECTION("Add items to queue") {
    auto node1 = TestNode();
    auto node2 = TestNode();

    REQUIRE(queue.getSize() == 0);

    queue.push(node1);
    REQUIRE(queue.getSize() == 1);
    REQUIRE(node1.index == 0);

    queue.push(node2);
    REQUIRE(queue.getSize() == 2);
  }

  SECTION("Clear queue") {
    queue.clear();
    REQUIRE(queue.getSize() == 0);
  }

  REQUIRE(queue.getCapacity() == 100);
}
}
