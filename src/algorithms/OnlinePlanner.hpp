#ifndef METRONOME_ONLINEPLANNER_HPP
#define METRONOME_ONLINEPLANNER_HPP

#include <vector>
#include "Planner.hpp"
#include "experiment/termination/TimeTerminationChecker.hpp"
namespace metronome {

template <typename Domain>
class OnlinePlanner : public Planner {
public:
    class ActionBundle final {
    public:
        ActionBundle(typename Domain::Action action, typename Domain::Cost actionDuration)
                : action{action}, actionDuration{actionDuration} {
        }

        void swap(ActionBundle& other) {

        }

        typename Domain::Action action;
        typename Domain::Cost actionDuration;
    };

    virtual ~OnlinePlanner() = default;

    virtual std::vector<ActionBundle> selectActions(const typename Domain::State& startState,
            const TimeTerminationChecker& terminationChecker) = 0;

    void swap(ActionBundle& lhs, ActionBundle& rhs) {
        lhs.swap(rhs);
    }
};
}

#endif // METRONOME_ONLINEPLANNER_HPP
