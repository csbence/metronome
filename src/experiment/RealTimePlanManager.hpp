#ifndef METRONOME_REALTIMEPLANMANAGER_HPP
#define METRONOME_REALTIMEPLANMANAGER_HPP

#include "PlanManager.hpp"
#include "util/TimeMeasurement.hpp"
#include <MetronomeException.hpp>
namespace metronome {
template <typename Domain, typename Planner>
class RealTimePlanManager : PlanManager<Domain, Planner> {
public:
    Result plan(const Configuration& configuration, const Domain& domain, Planner& planner) {
        std::vector<typename Domain::Action> actions;
        long long int planningTime{0};
        Domain::State currentState = domain.getStartState();

        while (!domain.isGoal(currentState)) {
            auto planningIterationTime = measureNanoTime([&] {
                auto actionList{planner.plan(currentState)};

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
    ActionBundle(Domain::Action action, Domain::Cost cost) : action{action}, cost{cost} {
    }
    const Domain::Action action;
    const Domain::Cost cost;
};
}

#endif // METRONOME_REALTIMEPLANMANAGER_HPP
