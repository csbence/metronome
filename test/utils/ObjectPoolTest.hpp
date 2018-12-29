#include "catch.hpp"
#include "easylogging++.h"
#include "utils/ObjectPool.hpp"
#include "utils/PriorityQueue.hpp"

namespace {

using namespace metronome;

struct TestNode {
  TestNode(int value) : value(value) {}
  mutable unsigned int index;
  int value;
};

TEST_CASE("ObjectPool Allocation", "[ObjectPool]") {
  ObjectPool<TestNode, 300000000> nodePool;
  
  std::cout << nodePool.empty();
}

}  // namespace
