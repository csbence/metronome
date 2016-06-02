#include "catch.hpp"
#include "easylogging++.h"
#include "util/PriorityQueue.hpp"

namespace {

struct TestNode {
  TestNode(int value) : value(value) {}
  mutable unsigned int index;
  int value;
};

int nodeCompare(const TestNode &lhs, const TestNode &rhs) {
  if (lhs.value < rhs.value)
    return -1;
  if (lhs.value > rhs.value)
    return 1;
  return 0;
}

TEST_CASE("PriorityQueue add/clear test", "[PriorityQueue]") {
  auto queue = PriorityQueue<TestNode>(100, nodeCompare);

  REQUIRE(queue.getCapacity() == 100);

  SECTION("Add items to queue") {
    auto node1 = TestNode(1);
    auto node2 = TestNode(2);

    REQUIRE(queue.getSize() == 0);
    REQUIRE(queue.top() == nullptr);
    REQUIRE(queue.isEmpty() == true);

    queue.push(node1);

    REQUIRE(queue.getSize() == 1);
    REQUIRE(node1.index == 0);
    REQUIRE(queue.top() == &node1);
    REQUIRE(queue.top() != &node2);
    REQUIRE(queue.isEmpty() == false);

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
  auto queue = PriorityQueue<TestNode>(100, nodeCompare);

  REQUIRE(queue.getCapacity() == 100);

  SECTION("Add items to queue") {
    auto node1 = TestNode(1);
    auto node2 = TestNode(2);
    auto node0 = TestNode(0);

    std::stringstream ss;
    ss << &node2;
    LOG(INFO) << "Address of node2: " + ss.str();

    REQUIRE(queue.getSize() == 0);

    queue.push(node1);
    REQUIRE(queue.getSize() == 1);
    REQUIRE(node1.index == 0);

    queue.push(node2);
    queue.push(node0);

    REQUIRE(queue.getSize() == 3);

    REQUIRE(queue.pop() == &node0);
    REQUIRE(queue.pop() == &node1);
    REQUIRE(queue.pop() == &node2);
  }

  SECTION("Add several items") {
    auto node3 = TestNode(12);
    auto node4 = TestNode(16);
    auto node5 = TestNode(-1);
    auto node6 = TestNode(5);
    auto node7 = TestNode(9);
    auto node8 = TestNode(9);

    queue.push(node3);
    queue.push(node4);
    queue.push(node5);
    queue.push(node6);
    queue.push(node7);
    queue.push(node8);

    int value = -10;
    while (!queue.isEmpty()) {
      REQUIRE(queue.top()->value >= value);
      value = queue.pop()->value;
    }
  }

  SECTION("Update item") {
    auto node3 = TestNode(12);
    auto node4 = TestNode(16);
    auto node5 = TestNode(-1);
    auto node6 = TestNode(5);
    auto node7 = TestNode(9);
    auto node8 = TestNode(9);

    queue.push(node3);
    queue.push(node4);
    queue.push(node5);
    queue.push(node6);
    queue.push(node7);
    queue.push(node8);

    node4.value = -2;
    REQUIRE(node4.index == 3);
    queue.update(node4);
    REQUIRE(node4.index == 0);

    int value = -10;
    while (!queue.isEmpty()) {
      REQUIRE(queue.top()->value >= value);
      value = queue.pop()->value;
    }
  }

  SECTION("Clear queue") {
    queue.clear();
    REQUIRE(queue.getSize() == 0);
  }

  REQUIRE(queue.getCapacity() == 100);
}
}
