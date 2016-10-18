#ifndef METRONOME_PLANMANAGER_HPP
#define METRONOME_PLANMANAGER_HPP

#include "Configuration.hpp"
#include "Result.hpp"
namespace metronome {

template <typename Domain, typename Planner>
class PlanManager {
public:
    virtual Result plan(const Configuration&, const Domain&, Planner, bool b) = 0; //
};
}

#endif // METRONOME_PLANMANAGER_HPP
