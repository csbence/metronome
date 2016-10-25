#ifndef METRONOME_ONLINEPLANNER_HPP
#define METRONOME_ONLINEPLANNER_HPP

#include <vector>
#include "Planner.hpp"
#include "experiment/termination/TimeTerminationChecker.hpp"
namespace metronome {

template <typename Domain, typename TerminationChecker>
class OnlinePlanner : public Planner {
public:
    class ActionBundle final {
    public:
        ActionBundle(typename Domain::Action action, typename Domain::Cost actionDuration)
                : action{action}, actionDuration{actionDuration} {}

        typename Domain::Action action;
        typename Domain::Cost actionDuration;
    };

    virtual ~OnlinePlanner() = default;

    virtual std::vector<ActionBundle> selectActions(const typename Domain::State& startState,
            TerminationChecker& terminationChecker) = 0;

    virtual unsigned long long getIdentityActionCount() const final {
        return identityActionCount;
    }

    virtual inline void incrementIdentityActionCount() final {
        ++identityActionCount;
    }

private:
    int identityActionCount{0};


};
}

#endif // METRONOME_ONLINEPLANNER_HPP
