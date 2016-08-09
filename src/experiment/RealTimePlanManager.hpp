#ifndef METRONOME_REALTIMEPLANMANAGER_HPP
#define METRONOME_REALTIMEPLANMANAGER_HPP

#include <chrono>
#include <boost/optional.hpp>
#include "MetronomeException.hpp"
#include "experiment/termination/TimeTerminationChecker.hpp"
#include "PlanManager.hpp"
#include "utils/TimeMeasurement.hpp"

namespace metronome {
template <typename Domain, typename Planner>
class RealTimePlanManager : PlanManager<Domain, Planner> {
public:
    Result plan(const Configuration& configuration, const Domain& domain, Planner& planner) {
        using namespace std::chrono;

        std::vector<typename Domain::Action> actions;
        long long int planningTime{0};
        const auto actionDuration = configuration.getLong(ACTION_DURATION);
        auto currentState = domain.getStartState();

        TimeTerminationChecker terminationChecker;

        long long int timeBound = actionDuration;
        long long int previousTimeBound;

        while (!domain.isGoal(currentState)) {
            auto planningIterationTime = measureNanoTime([&] {
                terminationChecker.resetTo(nanoseconds(static_cast<nanoseconds>(timeBound)));

                auto actionBundles{planner.selectActions(currentState, terminationChecker)};

                previousTimeBound = timeBound;
                timeBound = 0;
                for (auto& actionBundle : actionBundles) {
                    boost::optional<typename Domain::State> nextState = domain.transition(currentState, actionBundle
                        .action);
                    if (!nextState.is_initialized()) {
                        throw MetronomeException("Invalid action. Partial plan is corrupt.");
                    }
                    currentState = nextState.get();
                    actions.emplace_back(actionBundle.action);
                    timeBound += actionBundle.actionDuration;
                }

            });

//            LOG(INFO) << planningIterationTime - previousTimeBound;

//            if (planningIterationTime > previousTimeBound + 100000) {
//                LOG(INFO) << "Time bount violated: " << planningIterationTime - previousTimeBound << " bound: " << previousTimeBound;
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

        return Result(configuration,
                planner.getExpandedNodeCount(),
                planner.getGeneratedNodeCount(),
                planningTime,
                pathLength * configuration.getLong("actionDuration"),
                planningTime + pathLength * configuration.getLong("actionDuration"),
                planningTime,
                pathLength,
                actionStrings);
    }
};

}

#endif // METRONOME_REALTIMEPLANMANAGER_HPP
