#pragma once

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
        normalizedGoalAchievementTime{0},
        idlePlanningTime{0},
        pathLength{0},
        actions{std::vector<std::string>()},
        iterationCount{0},
        idleIterationCount{0},
        goalFirstFoundIteration{0},
        identityActions{0},
        timestamp{currentNanoTime()} {};

  Result(const Configuration& configuration,
         const int expandedNodes,
         const int generatedNodes,
         const long long planningTime,
         const long long actionExecutionTime,
         const long long goalAchievementTime,
         const long long normalizedGoalAchievementTime,
         const long long idlePlanningTime,
         const long long pathLength,
         const std::vector<std::string> actions,
         const int iterationCount = 0,
         const int idleIterationCount = 0,
         const int goalFirstFoundIteration = 0,
         const int identityActions = 0,
         const long long timestamp = currentNanoTime())
      : configuration{configuration},
        errorMessage{""},
        success{true},
        expandedNodes{expandedNodes},
        generatedNodes{generatedNodes},
        planningTime{planningTime},
        actionExecutionTime{actionExecutionTime},
        goalAchievementTime{goalAchievementTime},
        normalizedGoalAchievementTime{normalizedGoalAchievementTime},
        idlePlanningTime{idlePlanningTime},
        pathLength{pathLength},
        actions{actions},
        iterationCount{iterationCount},
        idleIterationCount{idleIterationCount},
        goalFirstFoundIteration{goalFirstFoundIteration},
        identityActions{identityActions},
        timestamp{timestamp} {};

  std::string getJsonString() const {
    using namespace rapidjson;

    Document resultDocument;
    resultDocument.SetObject();

    auto& allocator = resultDocument.GetAllocator();

    Value errorMessageValue;
    errorMessageValue.SetString(errorMessage.c_str(), errorMessage.size());
    resultDocument.AddMember("errorMessage", errorMessageValue, allocator);

    resultDocument.AddMember("success", success, allocator);
    resultDocument.AddMember(
        "expandedNodes", Value{}.SetInt(expandedNodes), allocator);
    resultDocument.AddMember(
        "generatedNodes", Value{}.SetInt(generatedNodes), allocator);
    resultDocument.AddMember(
        "planningTime", Value{}.SetInt64(planningTime), allocator);
    resultDocument.AddMember("actionExecutionTime",
                             Value{}.SetInt64(actionExecutionTime),
                             allocator);
    resultDocument.AddMember("goalAchievementTime",
                             Value{}.SetInt64(goalAchievementTime),
                             allocator);
    resultDocument.AddMember("normalizedGoalAchievementTime",
                             Value{}.SetInt64(normalizedGoalAchievementTime),
                             allocator);
    resultDocument.AddMember(
        "idlePlanningTime", Value{}.SetInt64(idlePlanningTime), allocator);
    resultDocument.AddMember(
        "pathLength", Value{}.SetInt64(pathLength), allocator);

    //        GenericValue<ASCII<>> actionsArray{kArrayType};
    //        for (std::string action : actions) {
    //            actionsArray.PushBack(GenericValue<ASCII<>>{}.SetString(action.c_str(),
    //            action.size(), allocator), allocator);
    //        }
    //
    //        resultDocument.AddMember("actions", actionsArray, allocator);
    resultDocument.AddMember(
        "iterationCount", Value{}.SetInt(iterationCount), allocator);
    resultDocument.AddMember(
        "idleIterationCount", Value{}.SetInt(idleIterationCount), allocator);
    resultDocument.AddMember("goalFirstFoundIteration",
                             Value{}.SetInt(goalFirstFoundIteration),
                             allocator);
    resultDocument.AddMember(
        "identityActions", Value{}.SetInt(identityActions), allocator);
    resultDocument.AddMember(
        "timestamp", Value{}.SetInt64(timestamp), allocator);
    
    for (const auto& attribute : attributes) {
        Value key(attribute.first.c_str(), allocator);
        Value value(attribute.second);
        resultDocument.AddMember(key, value, allocator);
    }

    const rapidjson::Document& document = configuration.getJsonDocument();
    rapidjson::Value configurationValue;
    configurationValue.CopyFrom(document, allocator);
    resultDocument.AddMember("configuration", configurationValue, allocator);

    StringBuffer buffer;
    Writer<StringBuffer> writer(buffer);
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
  const long long normalizedGoalAchievementTime;
  const long long idlePlanningTime;
  const long long pathLength;
  const std::vector<std::string> actions;
  const int iterationCount;
  const int idleIterationCount;
  const int goalFirstFoundIteration;
  const int identityActions;
  const long long timestamp;
  std::vector<std::pair<std::string, std::int64_t>> attributes;
};

}  // namespace metronome
