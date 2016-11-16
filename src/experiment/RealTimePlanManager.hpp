#ifndef METRONOME_REALTIMEPLANMANAGER_HPP
#define METRONOME_REALTIMEPLANMANAGER_HPP

#include <boost/optional.hpp>
#include <chrono>
#include "MetronomeException.hpp"
#include "PlanManager.hpp"
#include "utils/TimeMeasurement.hpp"

namespace metronome {
template <typename Domain, typename Planner, typename TerminationChecker>
class RealTimePlanManager : PlanManager<Domain, Planner> {
public:
    Result plan(const Configuration& configuration, const Domain& domain, Planner& planner) {
        using namespace std::chrono;

        std::string lookaheadType{configuration.getStringOrThrow(LOOKAHEAD_TYPE)};

        bool dynamicLookahead;
        if (lookaheadType == LOOKAHEAD_STATIC) {
            dynamicLookahead = false;
        } else if (lookaheadType == LOOKAHEAD_DYNAMIC) {
            dynamicLookahead = true;
        } else {
            LOG(ERROR) << "Unknown lookahead type: " << lookaheadType << std::endl;
            return Result(configuration, "Unknown lookaheadType: " + lookaheadType);
        }

        std::vector<typename Domain::Action> actions;
        long long int planningTime{0};
        const auto firstIterationDuration = getFirstIterationDuration(configuration);
        const auto actionExecutionTime = configuration.getLong("actionExecutionTime", 1);
        auto currentState = domain.getStartState();

        TerminationChecker terminationChecker;

        long long int timeBound = firstIterationDuration;
        long long int previousTimeBound;

        while (!domain.isGoal(currentState)) {
            auto planningIterationTime = measureNanoTime([&] {
                if (dynamicLookahead) {
                    terminationChecker.resetTo(timeBound / actionExecutionTime);
                } else {
                    terminationChecker.resetTo(firstIterationDuration);
                }

                auto actionBundles{planner.selectActions(currentState, terminationChecker)};

                previousTimeBound = timeBound;
                timeBound = 0;

                LOG(INFO) << "Number of actions: " << actionBundles.size();
                for (auto& actionBundle : actionBundles) {
                    boost::optional<typename Domain::State> nextState =
                            domain.transition(currentState, actionBundle.action);
                    if (!nextState.is_initialized()) {
                        throw MetronomeException("Invalid action. Partial plan is corrupt.");
                    }
                    currentState = nextState.get();
//                    LOG(INFO) << actionBundle.actionDuration << std::endl;
                    actions.emplace_back(actionBundle.action);
                    timeBound += actionBundle.actionDuration;
                }

            });

            //            LOG(INFO) << planningIterationTime - previousTimeBound;

            //            if (planningIterationTime > previousTimeBound + 100000) {
            //                LOG(INFO) << "Time bount violated: " << planningIterationTime - previousTimeBound << "
            //                bound: " << previousTimeBound;
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
            LOG(INFO) << action.toString();
        }

        return Result(configuration,
                planner.getExpandedNodeCount(),
                planner.getGeneratedNodeCount(),
                planningTime, // Planning time
                pathLength * configuration.getLong("actionDuration"), // Execution time
                domain.getActionDuration() + pathLength * configuration.getLong("actionDuration"), // GAT
                domain.getActionDuration(), // Idle planning time
                pathLength, // Path length
                actionStrings,
                planner.getIdentityActionCount());
    }

private:
    long long int getFirstIterationDuration(const Configuration& configuration) const {
        if (configuration.hasMember(FIRST_ITERATION_DURATION)) {
            return configuration.getLong(FIRST_ITERATION_DURATION);
        }

        return configuration.getLongOrThrow(ACTION_DURATION);
    }
};
}

#endif // METRONOME_REALTIMEPLANMANAGER_HPP
