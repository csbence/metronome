#pragma once

#include <sstream>
#include "Experiment.hpp"
#include "utils/TimeMeasurement.hpp"

namespace metronome {
template <typename Domain, typename Planner>
class OfflineExperiment : Experiment<Domain, Planner> {
 public:
  Result plan(const Configuration& configuration,
              const Domain& domain,
              Planner& planner) {
    std::vector<typename Domain::Action> actions;
    //            long long int start = currentNanoTime();
    //        logTime();
    auto planningTime = measureNanoTime(
        [&] { actions = planner.plan(domain.getStartState()); });
    //        long long int end = currentNanoTime();
    //        LOG(DEBUG) << (end - start) / 1000000;

    auto pathLength = actions.size();

    std::vector<std::string> actionStrings;
    typename Domain::State currentState = domain.getStartState();

    for (auto& action : actions) {
      std::ostringstream stringStream;
      stringStream << action;
      actionStrings.push_back(stringStream.str());
      
      auto candidateState = domain.transition(currentState, action);
      if (!candidateState.has_value()) {
        throw MetronomeException("Invalid path.");
      }
      currentState = candidateState.value();
    }
    if (!domain.isGoal(currentState)) {
      throw MetronomeException("Goal is not reached!");
    }

    const std::string terminationCheckerType{
        configuration.getString(TERMINATION_CHECKER_TYPE)};
    long long int calculatedPlanningTime = planningTime;

    if (terminationCheckerType == TERMINATION_CHECKER_EXPANSION) {
      calculatedPlanningTime = planner.getExpandedNodeCount();
    }

    // Normalize GAT
      long long int actionDuration = configuration.getLong(ACTION_DURATION, 1);

      auto normalizedGat = calculatedPlanningTime / actionDuration + pathLength;

    return Result(configuration,
                  planner.getExpandedNodeCount(),
                  planner.getGeneratedNodeCount(),
                  planningTime,
                  pathLength * actionDuration,
                  calculatedPlanningTime +
                  pathLength * actionDuration,
                  normalizedGat,
                  calculatedPlanningTime,
                  pathLength,
                  actionStrings);
  }
};

}  // namespace metronome
