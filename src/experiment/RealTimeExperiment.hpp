#pragma once

#include <optional>
#include <sstream>
#include "Experiment.hpp"
#include "MetronomeException.hpp"
#include "easylogging++.h"
#include "utils/TimeMeasurement.hpp"

namespace metronome {
template <typename Domain, typename Planner, typename TerminationChecker>
class RealTimeExperiment : Experiment<Domain, Planner> {
 public:
  using State = typename Domain::State;
  using Action = typename Domain::Action;
  using Cost = typename Domain::Cost;

  RealTimeExperiment(const Configuration& configuration) {
    std::string lookaheadType = configuration.getString(LOOKAHEAD_TYPE);

    if (lookaheadType == LOOKAHEAD_STATIC) {
      dynamicLookahead = false;
    } else if (lookaheadType == LOOKAHEAD_DYNAMIC) {
      dynamicLookahead = true;
    } else {
      throw MetronomeException("Unknown lookaheadType: " + lookaheadType);
    }

    std::string commitmentStrategy =
        configuration.getString(COMMITMENT_STRATEGY);

    if (commitmentStrategy == COMMITMENT_SINGLE) {
      singleStep = true;
    } else if (commitmentStrategy == COMMITMENT_MULTIPLE) {
      singleStep = false;
    } else {
      throw MetronomeException("Unknown commitment strategy: " + lookaheadType);
    }
  }

  Result plan(const Configuration& configuration,
              const Domain& domain,
              Planner& planner) {
    using namespace std::chrono;
    //    LOG(INFO) << "Begin real-time planning iterations";

    std::vector<typename Domain::Action> actions;

    std::size_t planningTime = 0;
    std::size_t executionTime = 0;
    std::size_t planningAndExecutionTime = 0;

    const auto actionDuration = configuration.getLong(ACTION_DURATION);
    auto currentState = domain.getStartState();

    TerminationChecker terminationChecker;

    auto timeBound = static_cast<std::size_t>(actionDuration);

    while (!domain.isGoal(currentState)) {
      if (dynamicLookahead) {
        terminationChecker.resetTo(timeBound);
      } else {
        terminationChecker.resetTo(actionDuration);
      }

      const auto iterationStartTime = currentNanoTime();
      auto actionBundles =
          planner.selectActions(currentState, terminationChecker);
      const auto iterationEndTime = currentNanoTime();

      if (singleStep && actionBundles.size() > 1) {
        actionBundles.resize(1);
      }

      const std::size_t iterationDuration =
          iterationEndTime - iterationStartTime;

      // Increase the total planning time after each planning iteration
      planningTime += iterationDuration;

      // Planning might take longer/shorter than the allocated time.
//      std::cout << iterationDuration << ',';
      planningAndExecutionTime += std::max(iterationDuration, timeBound);

      timeBound = 0;

      for (auto& actionBundle : actionBundles) {
        currentState =
            validateAction(domain, currentState, actionBundle.action);

        actions.emplace_back(actionBundle.action);
        timeBound += actionBundle.actionDuration;
      }

      executionTime += timeBound;
    }
    
    std::cout << std::endl;

    LOG(INFO) << "Planning: Done";

    auto pathLength = actions.size();

    std::vector<std::string> actionStrings;

    for (auto& action : actions) {
      std::ostringstream stringStream;
      stringStream << action;

      actionStrings.push_back(stringStream.str());
    }

    auto result = Result(configuration,
                         planner.getExpandedNodeCount(),
                         planner.getGeneratedNodeCount(),
                         planningTime,   // Planning time
                      executionTime,  // Execution time
                      planningAndExecutionTime,
                         planner.getIterationCount(), // Normalized goal achievement - should be comparable across configs
                      domain.getActionDuration(),  // Idle planning time
                      pathLength,                  // Path length
                      actionStrings,
                         planner.getIterationCount(),
                         planner.getIdleIterationCount(),
                         planner.getGoalFirstFoundIteration());
    
    result.attributes = planner.getAttributes();
    return result;
  }

 private:
  State validateAction(const Domain& domain,
                       const State& state,
                       const Action& action) {
    auto nextState = domain.transition(state, action);

    if (!nextState.has_value()) {
      LOG(ERROR) << "Invalid action " << action << " from: " << state;
      throw MetronomeException("Invalid action. Partial plan is corrupt.");
    }
    LOG(INFO) << "> action from: " << state;

    return nextState.value();
  }

  bool dynamicLookahead;
  bool singleStep;
};

}  // namespace metronome
