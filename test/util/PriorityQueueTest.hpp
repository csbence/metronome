#define CATCH_CONFIG_COLOUR_NONE

#include "../../src/util/PriorityQueue.hpp"
#include "../dependencies/catch.hpp"
#include "easylogging++.h"

INITIALIZE_EASYLOGGINGPP

namespace {
class TestNode {
 public:

};


auto queue = PriorityQueue<TestNode>(100, );

REQUIRE(queue.getCapacity() == 100);

TEST_CASE("PriorityQueue test", "[PriorityQueue]") {}
}