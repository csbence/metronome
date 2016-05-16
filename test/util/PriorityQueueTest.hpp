#define CATCH_CONFIG_COLOUR_NONE

#include "../../src/util/PriorityQueue.hpp"
#include "../dependencies/catch.hpp"
#include "easylogging++.h"

INITIALIZE_EASYLOGGINGPP

namespace {
class TestNode {
 public:

};

int nodeCompare(const TestNode&, const TestNode&) {
    return 1;  // TODO: actually compare test nodes
}

auto queue = PriorityQueue<TestNode>(100, nodeCompare);


TEST_CASE("PriorityQueue test", "[PriorityQueue]") {
    REQUIRE(queue.getCapacity() == 100); 
}

}
