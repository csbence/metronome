#include <domains/Vehicle.hpp>
#include "catch.hpp"
#include "easylogging++.h"
#include "experiment/ConfigurationExecutor.hpp"

namespace {
const char* json = "{\"timeLimit\" : 150000000000,\n"
                   "\"domainPath\" : "
                   "\"/input/vacuum/slalom.vw\",\n"
                   "\"domainInstanceName\" : \"Manual test instance\",\n"
                   "\"actionDuration\" : 6000000,\n"
                   "\"domainName\" : \"GRID_WORLD\",\n"
                   "\"terminationType\" : \"time\",\n"
                   "\"list\" : [1, 2],\n"
                   "\"objects\" : {\"a\": [], \"b\": {}},\n"
                   "\"algorithmName\" : \"A_STAR\"}";

const std::string resourceDir = "/home/aifs2/doylew/Public/metronome/resources";
metronome::Vehicle testGrid =
        metronome::ConfigurationExecutor::extractDomain<metronome::Vehicle>(metronome::Configuration(json),
                resourceDir);
TEST_CASE("Vehicle basic creation test", "[Vehicle]") {
    metronome::Vehicle vehicle = testGrid;
    REQUIRE(vehicle.getStartLocation() == metronome::Vehicle::State(6, 0));
}
TEST_CASE("Vehicle object movements test", "[Vehicle]") {
        metronome::Vehicle vehicle = testGrid;


}
}
