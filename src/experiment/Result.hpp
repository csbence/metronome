#ifndef METRONOME_RESULT_HPP
#define METRONOME_RESULT_HPP

#include <iostream>
#include <string>
#include <utils/TimeMeasurement.hpp>
#include <vector>
#include "Configuration.hpp"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

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

    std::string getJsonString() const {
        using namespace rapidjson;

        GenericDocument<ASCII<>> resultDocument;
        resultDocument.SetObject();

        auto& allocator = resultDocument.GetAllocator();

        //        resultDocument.AddMember("configuration", Value{configuration}, allocator);

        GenericValue<ASCII<>> test;
        std::string testString("asdf");
        test.SetString(testString.c_str(), testString.size());
        resultDocument.AddMember("test", test, allocator);

        GenericValue<ASCII<>> errorMessageValue;
        errorMessageValue.SetString(errorMessage.c_str(), errorMessage.size());
        resultDocument.AddMember("errorMessage", errorMessageValue, allocator);

        resultDocument.AddMember("success", success, allocator);
        resultDocument.AddMember("expandedNodes", expandedNodes, allocator);
        resultDocument.AddMember("generatedNodes", generatedNodes, allocator);
        resultDocument.AddMember("planningTime", planningTime, allocator);
        resultDocument.AddMember("actionExecutionTime", actionExecutionTime, allocator);
        resultDocument.AddMember("goalAchievementTime", goalAchievementTime, allocator);
        resultDocument.AddMember("idlePlanningTime", idlePlanningTime, allocator);
        resultDocument.AddMember("pathLength", pathLength, allocator);

//        GenericValue<ASCII<>> actionsArray{kArrayType};
//        for (std::string action : actions) {
//            actionsArray.PushBack(GenericValue<ASCII<>>{}.SetString(action.c_str(), action.size(), allocator), allocator);
//        }
//
//        resultDocument.AddMember("actions", actionsArray, allocator);
        resultDocument.AddMember("timestamp", timestamp, allocator);

        StringBuffer buffer;
        Writer<StringBuffer, Document::EncodingType, ASCII<>> writer(buffer);
        resultDocument.Accept(writer);
        return buffer.GetString();
    }

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
