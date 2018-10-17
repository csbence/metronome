#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do
                           // this in one cpp file
#define CATCH_CONFIG_COLOUR_NONE

#include "../dependencies/catch.hpp"
//#include "algorithms/AStarTest.hpp"
//#include "algorithms/LssLrtaStarTest.hpp"
#include "easylogging++.h"
//#include "experiment/TimeTerminationCheckerTest.hpp"
//#include "utils/PriorityQueueTest.hpp"
//#include "domains/GridWorldTest.hpp"
#include "domains/TrafficTest.hpp"
//#include "domains/SlidingTilePuzzleTest.hpp"
INITIALIZE_EASYLOGGINGPP

TEST_CASE("Main test", "[main]") {}
