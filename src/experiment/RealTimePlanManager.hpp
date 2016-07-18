#ifndef METRONOME_REALTIMEPLANMANAGER_HPP
#define METRONOME_REALTIMEPLANMANAGER_HPP

#include "PlanManager.hpp"
#include "utils/TimeMeasurement.hpp"
#include <MetronomeException.hpp>
#include <experiment/termination/TimeTerminationChecker.hpp>
namespace metronome {
template <typename Domain, typename Planner>
class RealTimePlanManager : PlanManager<Domain, Planner> {
public:
    Result plan(const Configuration& configuration, const Domain& domain, Planner& planner) {
        std::vector<typename Domain::Action> actions;
        long long int planningTime{0};
        typename Domain::State currentState = domain.getStartState();
        TimeTerminationChecker terminationChecker;

        while (!domain.isGoal(currentState)) {
            auto planningIterationTime = measureNanoTime([&] {
                auto actionList{planner.selectActions(currentState, terminationChecker)};

                for (auto& action : actionList) {
                    currentState = domain.transition(currentState, action);
                    actions.emplace_back(action);
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

template <typename Domain>
class ActionBundle {
public:
    ActionBundle(typename Domain::Action action, typename Domain::Cost cost) : action{action}, cost{cost} {
    }
    const typename Domain::Action action;
    const typename Domain::Cost cost;
};
}

#endif // METRONOME_REALTIMEPLANMANAGER_HPP
