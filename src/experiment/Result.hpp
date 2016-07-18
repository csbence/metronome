#ifndef METRONOME_RESULT_HPP
#define METRONOME_RESULT_HPP

#include <string>
#include <vector>
#include <utils/TimeMeasurement.hpp>
#include "Configuration.hpp"
namespace metronome {
class Result {
public:
    Result(const Configuration& configuration, std::string errorMessage)
            : configuration{configuration},
              errorMessage{std::move(errorMessage)},
              success{false},
              expandedNodes{0},
              generatedNodes{0},
              planningTime{0},
              actionExecutionTime{0},
              goalAchievementTime{0},
              idlePlanningTime{0},
              pathLength{0},
              actions{std::vector<std::string>()},
              timestamp{currentNanoTime()} {};

    Result(const Configuration& configuration,
            const int expandedNodes,
            const int generatedNodes,
            const long long planningTime,
            const long long actionExecutionTime,
            const long long goalAchievementTime,
            const long long idlePlanningTime,
            const long long pathLength,
            const std::vector<std::string> actions,
            const long long timestamp = currentNanoTime())
            : configuration{configuration},
              errorMessage{""},
              success{true},
              expandedNodes{expandedNodes},
              generatedNodes{generatedNodes},
              planningTime{planningTime},
              actionExecutionTime{actionExecutionTime},
              goalAchievementTime{goalAchievementTime},
              idlePlanningTime{idlePlanningTime},
              pathLength{pathLength},
              actions{actions},
              timestamp{timestamp} {};

    const Configuration& configuration;
    const std::string errorMessage;
    const bool success;
    const int expandedNodes;
    const int generatedNodes;
    const long long planningTime;
    const long long actionExecutionTime;
    const long long goalAchievementTime;
    const long long idlePlanningTime;
    const long long pathLength;
    const std::vector<std::string> actions;
    const long long timestamp;
};
}

#endif // METRONOME_RESULT_HPP
