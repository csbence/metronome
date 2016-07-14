#ifndef METRONOME_REALTIMEPLANMANAGER_HPP
#define METRONOME_REALTIMEPLANMANAGER_HPP

#include "util/TimeMeasurement.hpp"
#include "PlanManager.hpp"
namespace metronome {
template <typename Domain, typename Planner>
class RealTimePlanManager : PlanManager<Domain, Planner> {
public:
    Result plan(const Configuration& configuration, const Domain& domain, Planner& planner) {
        auto executionTime = measureNanoTime([&] {

            // TODO start state
//            planner.plan(configuration);

        });

        return Result();
    }
};
}

#endif // METRONOME_REALTIMEPLANMANAGER_HPP
