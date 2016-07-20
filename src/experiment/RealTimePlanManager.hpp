#ifndef METRONOME_REALTIMEPLANMANAGER_HPP
#define METRONOME_REALTIMEPLANMANAGER_HPP

#include <chrono>
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

        unsigned long long int timeBound = static_cast<unsigned long long int>(actionDuration);

        while (!domain.isGoal(currentState)) {
            auto planningIterationTime = measureNanoTime([&] {
                terminationChecker.resetTo(nanoseconds(static_cast<nanoseconds>(timeBound)));

                auto actionBundles{planner.selectActions(currentState, terminationChecker)};

                timeBound = 0;
                for (auto& actionBundle : actionBundles) {
                    currentState = domain.transition(currentState, actionBundle.action);
                    actions.emplace_back(actionBundle.action);
                    timeBound += actionBundle.actionDuration;
                }

            });

            // Increase the total planning time after each planning iteration
            planningTime += planningIterationTime;
        }

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
