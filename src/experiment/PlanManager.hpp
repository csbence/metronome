#ifndef METRONOME_PLANMANAGER_HPP
#define METRONOME_PLANMANAGER_HPP

#include "Configuration.hpp"
#include "Result.hpp"
namespace metronome {

template <typename Domain, typename Planner>
class PlanManager {
public:
    virtual Result plan(const Configuration&, const Domain&, Planner&) = 0; //

protected:
    long long int getFirstIterationDuration(const Configuration& configuration) const {
        if (configuration.hasMember(FIRST_ITERATION_DURATION)) {
            return configuration.getLong(FIRST_ITERATION_DURATION);
        }

        return configuration.getLongOrThrow(ACTION_DURATION);
    }
};
}

#endif // METRONOME_PLANMANAGER_HPP
