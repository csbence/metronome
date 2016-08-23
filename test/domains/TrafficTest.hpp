#include "catch.hpp"
#include "domains/Traffic.hpp"
#include "easylogging++.h"
#include "experiment/ConfigurationExecutor.hpp"

namespace {
const char* json = "{\"timeLimit\" : 150000000000,\n"
                   "\"domainPath\" : "
                   "\"/input/vacuum/smallwall.vw\",\n"
                   "\"domainInstanceName\" : \"Manual test instance\",\n"
                   "\"actionDuration\" : 6000000,\n"
                   "\"domainName\" : \"TRAFFIC\",\n"
                   "\"terminationType\" : \"time\",\n"
                   "\"list\" : [1, 2],\n"
                   "\"objects\" : {\"a\": [], \"b\": {}},\n"
                   "\"algorithmName\" : \"A_STAR\"}";

const std::string resourceDir = "/home/aifs2/doylew/Public/metronome/resources";
metronome::Traffic testGrid =
        metronome::ConfigurationExecutor::extractDomain<metronome::Traffic>(metronome::Configuration(json),
                resourceDir);

TEST_CASE("Traffic basic creation test", "[Traffic]") {
    metronome::Traffic traffic = testGrid;
    std::cout << testGrid.getStartLocation().getX() << "," << testGrid.getStartLocation().getY() << std::endl;
    std::cout << testGrid.getGoalState().getX() << "," << testGrid.getGoalState().getY() << std::endl;
    std::cout << testGrid.isGoal(testGrid.getStartLocation()) << std::endl;
    std::cout << testGrid.isGoal(testGrid.getGoalState()) << std::endl;

    testGrid.visualize(std::cout, testGrid.getStartLocation(), metronome::Traffic::Action(5));

    std::vector<SuccessorBundle<metronome::Traffic>> test = testGrid.successors(testGrid.getStartLocation());

    std::cout << "SUCCESSORS: \n" << std::endl;
    for (SuccessorBundle<metronome::Traffic> bundle : test) {
        testGrid.visualize(std::cout, bundle.state, bundle.action);
        std::cout << "SUCCESSORS OF SUCCESSORS" << std::endl;
        std::vector<SuccessorBundle<metronome::Traffic>> test1 = testGrid.successors(bundle.state);

        for(SuccessorBundle<metronome::Traffic> bundle1 : test1) {
            testGrid.visualize(std::cout, bundle1.state, bundle1.action);
            std::cout << "SUCCESSORS OF SUCCESSORS SUCCESSORS" << std::endl;
            std::vector<SuccessorBundle<metronome::Traffic>> test2 = testGrid.successors(bundle1.state);

            for(SuccessorBundle<metronome::Traffic> bundle2 : test2) {
                testGrid.visualize(std::cout, bundle2.state, bundle2.action);
                std::cout << "SUCCESSORS OF SUCCESSORS SUCCESSORS of successors" << std::endl;
                std::vector<SuccessorBundle<metronome::Traffic>> test3 = testGrid.successors(bundle2.state);

                for(SuccessorBundle<metronome::Traffic> bundle3 : test3) {
                    testGrid.visualize(std::cout, bundle3.state, bundle3.action);
                }
                std::cout << "END S-S-S-s" << std::endl;
            }
            std::cout << "END S-S-S" << std::endl;
        }
        std::cout << "LOOP FOR" << std::endl;
    }


    // REQUIRE(traffic.getStartLocation() == metronome::Traffic::State(6, 0));
}
TEST_CASE("Traffic object movements test", "[Traffic]") {
    metronome::Traffic traffic = testGrid;
}
}
