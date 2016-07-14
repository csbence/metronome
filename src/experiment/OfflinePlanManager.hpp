#ifndef METRONOME_OFFLINEPLANMANAGER_HPP
#define METRONOME_OFFLINEPLANMANAGER_HPP

#include "util/TimeMeasurement.hpp"
#include "PlanManager.hpp"
namespace metronome {
template <typename Domain, typename Planner>
class OfflinePlanManager : PlanManager<Domain, Planner> {
public:
    Result plan(const Configuration& configuration, const Domain& domain, Planner& planner) {
        auto executionTime = measureNanoTime([&] {

            planner.plan(domain.getStartState());

        });

        return Result();
    }
};
}

#endif // METRONOME_OFFLINEPLANMANAGER_HPP
