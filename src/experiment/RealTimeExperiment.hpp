#pragma once

#include <boost/optional.hpp>
#include <chrono>
#include "Experiment.hpp"
#include "MetronomeException.hpp"
#include "easylogging++.h"
#include "utils/TimeMeasurement.hpp"

namespace metronome {
template <typename Domain, typename Planner, typename TerminationChecker>
class RealTimeExperiment : Experiment<Domain, Planner> {
 public:

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
    long long int planningTime{0};
    const auto actionDuration = configuration.getLong(ACTION_DURATION);
    auto currentState = domain.getStartState();

    TerminationChecker terminationChecker;

    long long int timeBound = actionDuration;
    long long int previousTimeBound;

    while (!domain.isGoal(currentState)) {
      auto planningIterationTime = measureNanoTime([&] {
        if (dynamicLookahead) {
          terminationChecker.resetTo(timeBound);
        } else {
          terminationChecker.resetTo(actionDuration);
        }

        auto actionBundles =
            planner.selectActions(currentState, terminationChecker);
        
        if (singleStep && actionBundles.size() > 1) {
          actionBundles.resize(1);
        }

        previousTimeBound = timeBound;
        timeBound = 0;
        for (auto& actionBundle : actionBundles) {
          boost::optional<typename Domain::State> nextState =
              domain.transition(currentState, actionBundle.action);

          if (!nextState.is_initialized()) {
            LOG(ERROR) << "Invalid action from: " << currentState
                       << "Expected target: "
                       << actionBundle.expectedTargetState;
            throw MetronomeException(
                "Invalid action. Partial plan is corrupt.");
          }
          LOG(INFO) << "> action from: " << currentState
                    << " expected target: " << actionBundle.expectedTargetState;

          currentState = nextState.get();
          //                    LOG(INFO) << actionBundle.action << std::endl;
          actions.emplace_back(actionBundle.action);
          timeBound += actionBundle.actionDuration;
        }
      });

      //            LOG(INFO) << planningIterationTime - previousTimeBound;

      //            if (planningIterationTime > previousTimeBound + 100000) {
      //                LOG(INFO) << "Time bount violated: " <<
      //                planningIterationTime - previousTimeBound << " bound: "
      //                << previousTimeBound;
      //            } else {
      //                LOG(INFO) << "Fine";
      //            }

      // Increase the total planning time after each planning iteration
      planningTime += planningIterationTime;
    }

    LOG(INFO) << "Planning: Done";

    auto pathLength = actions.size();

    std::vector<std::string> actionStrings;

    for (auto& action : actions) {
      actionStrings.push_back(action.toString());
    }

    return Result(
        configuration,
        planner.getExpandedNodeCount(),
        planner.getGeneratedNodeCount(),
        planningTime,                                          // Planning time
        pathLength * configuration.getLong("actionDuration"),  // Execution time
        domain.getActionDuration() +
            pathLength * configuration.getLong("actionDuration"),  // GAT
        domain.getActionDuration(),  // Idle planning time
        pathLength,                  // Path length
        actionStrings);
  }

 private:
  bool dynamicLookahead;
  bool singleStep;
};

}  // namespace metronome
