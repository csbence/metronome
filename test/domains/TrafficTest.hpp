#include "catch.hpp"
#include "domains/Traffic.hpp"
#include "easylogging++.h"
#include "experiment/ConfigurationExecutor.hpp"

namespace {
const char* json = "{\"timeLimit\" : 150000000000,\n"
                   "\"domainPath\" : "
                   "\"/input/vacuum/larges.vw\",\n"
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
//    metronome::Traffic traffic = testGrid;
//    std::cout << testGrid.getStartState().getX() << "," << testGrid.getStartState().getY() << std::endl;
//    std::cout << testGrid.getGoalState().getX() << "," << testGrid.getGoalState().getY() << std::endl;
//    std::cout << testGrid.isGoal(testGrid.getStartState()) << std::endl;
//    std::cout << testGrid.isGoal(testGrid.getGoalState()) << std::endl;
//
//    testGrid.visualize(std::cout, testGrid.getStartState(), metronome::Traffic::Action(5));
//
//    std::vector<SuccessorBundle<metronome::Traffic>> test = testGrid.successors(testGrid.getStartState());
//
//    std::cout << "SUCCESSORS: \n" << std::endl;
//    for (SuccessorBundle<metronome::Traffic> bundle : test) {
//        testGrid.visualize(std::cout, bundle.state, bundle.action);
//        std::cout << "SUCCESSORS OF SUCCESSORS" << std::endl;
//        std::vector<SuccessorBundle<metronome::Traffic>> test1 = testGrid.successors(bundle.state);
//
//        for (SuccessorBundle<metronome::Traffic> bundle1 : test1) {
//            testGrid.visualize(std::cout, bundle1.state, bundle1.action);
//            std::cout << "SUCCESSORS OF SUCCESSORS SUCCESSORS" << std::endl;
//            std::vector<SuccessorBundle<metronome::Traffic>> test2 = testGrid.successors(bundle1.state);
//
//            for (SuccessorBundle<metronome::Traffic> bundle2 : test2) {
//                testGrid.visualize(std::cout, bundle2.state, bundle2.action);
//                std::cout << "SUCCESSORS OF SUCCESSORS SUCCESSORS of successors" << std::endl;
//                std::vector<SuccessorBundle<metronome::Traffic>> test3 = testGrid.successors(bundle2.state);
//
//                for (SuccessorBundle<metronome::Traffic> bundle3 : test3) {
//                    testGrid.visualize(std::cout, bundle3.state, bundle3.action);
//                }
//                std::cout << "END S-S-S-s" << std::endl;
//            }
//            std::cout << "END S-S-S" << std::endl;
//        }
//        std::cout << "LOOP FOR" << std::endl;
//    }

    // REQUIRE(traffic.getStartState() == metronome::Traffic::State(6, 0));
}
TEST_CASE("Traffic equals operator test", "[Traffic]") {
    metronome::Traffic::Obstacle o1{0, 1, 0, 0};
    metronome::Traffic::Obstacle o2{0, 2, 1, 1};

    REQUIRE(o1 != o2);
    REQUIRE(o1 == o1);
    REQUIRE(o2 == o2);

    metronome::Traffic::Obstacle o3{0, 1, 1, 1};
    metronome::Traffic::Obstacle o4{0, 2, 0, 0};

    REQUIRE(o3 != o4);
    REQUIRE(o3 == o3);
    REQUIRE(o4 == o4);

    REQUIRE(o1 != o3);
    REQUIRE(o2 != o3);
    REQUIRE(o1 != o4);
    REQUIRE(o2 != o4);

    std::vector<metronome::Traffic::Obstacle> vo;
    vo.push_back(o1);
    LOG(INFO) << "\n" << o1 << "\n" << o2 << "\n" << o3 << "\n" << o4 << std::endl;
    vo.push_back(o2);

    std::vector<metronome::Traffic::Obstacle> voo;
    voo.push_back(o3);
    voo.push_back(o4);
    metronome::Traffic::State s{1, 1, vo};
    metronome::Traffic::State ss{1, 1, voo};

    REQUIRE(s != ss);
    REQUIRE(s == s);
    REQUIRE(ss == ss);
}

TEST_CASE("Traffic simulation", "[Traffic]") {
    metronome::Traffic traffic = testGrid;

    std::vector<std::string> actions{"E",
            "E",
            "E",
            "E",
            "E",
            "E",
            "E",
            "E",
            "S",
            "S",
            "E",
            "E",
            "S",
            "S",
            "S",
            "S",
            "S",
            "S",
            "S",
            "S",
            "S",
            "S",
            "S",
            "E",
            "E",
            "E"};

    metronome::Traffic::State currentState = testGrid.getStartState();
    for (auto action : actions) {
        testGrid.visualize(std::cout,
                currentState,
                metronome::Traffic::Action{metronome::Traffic::Action::toValue(action.c_str())});

        boost::optional<metronome::Traffic::State> candidateState =
                testGrid.transition(currentState, metronome::Traffic::Action::toValue(action.c_str()));

        if(candidateState.is_initialized()) {
            currentState = candidateState.get();
        }
    }
}
}
