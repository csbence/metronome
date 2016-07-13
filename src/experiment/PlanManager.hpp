#ifndef METRONOME_PLANMANAGER_HPP
#define METRONOME_PLANMANAGER_HPP

#include "Configuration.hpp"
#include "Result.hpp"
#include <chrono>
#include <functional>
namespace metronome {

template <typename Domain, typename Planner>
class PlanManager {
public:
    static virtual Result plan(const Configuration&, const Domain&, Planner) = 0; //
};
}

#endif // METRONOME_PLANMANAGER_HPP
