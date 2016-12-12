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

        // Initialize parameters from configuration
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
        
        const bool singleStepCommitment{isSingleStepCommitment(configuration)};

        std::vector<typename Domain::Action> actions;
        long long int planningTime{0};
        const auto firstIterationDuration = this->getFirstIterationDuration(configuration);
        const auto actionExecutionTime = configuration.getLong("actionExecutionTime", 1);

        auto currentState = domain.getStartState();
        TerminationChecker terminationChecker;

        long long int timeBound = firstIterationDuration;
        long long int previousTimeBound;
        long long int totalActionExecutionTime{0};

        // Construct plan incrementally
        while (!domain.isGoal(currentState)) {
            auto planningIterationTime = measureNanoTime([&] {
                if (dynamicLookahead) {
                    terminationChecker.resetTo(timeBound / actionExecutionTime);
//                    LOG(INFO) << "Available steps: " << timeBound / actionExecutionTime;
                } else {
                    terminationChecker.resetTo(firstIterationDuration);
                }

                auto actionBundles{planner.selectActions(currentState, 
                                                                                      terminationChecker)};
                previousTimeBound = timeBound;
                timeBound = 0;

                if (actionBundles.empty()) {
                    throw MetronomeException("Solution not found");
                }
                
                if (singleStepCommitment && !actionBundles.empty()) {
                    // Limit the number of actions to one
                    actionBundles = std::vector<typename Planner::ActionBundle>{actionBundles[0]};
                }
                
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
                
                totalActionExecutionTime += timeBound; // Calculate total execution time

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
//            LOG(INFO) << action.toString();
        }

        return Result(configuration,
                planner.getExpandedNodeCount(),
                planner.getGeneratedNodeCount(),
                planningTime, // Planning time
                totalActionExecutionTime, // Execution time
                firstIterationDuration + totalActionExecutionTime, // GAT
                firstIterationDuration, // Idle planning time
                pathLength, // Path length
                actionStrings,
                planner.getIdentityActionCount());
    }

private:
    bool isSingleStepCommitment(const Configuration& configuration) const {
        if (configuration.hasMember(COMMITMENT_TYPE)) {
            return configuration.getString(COMMITMENT_TYPE) == COMMITMENT_SINGLE;
        }
        
        return false;
    }
};
}

#endif // METRONOME_REALTIMEPLANMANAGER_HPP
