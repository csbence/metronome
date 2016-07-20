#ifndef METRONOME_OFFLINEPLANNER_HPP
#define METRONOME_OFFLINEPLANNER_HPP

#include <vector>
#include "Planner.hpp"
namespace metronome {

template <typename Domain>
class OfflinePlanner : public Planner {
public:
    virtual ~OfflinePlanner() = default;
    virtual std::vector<typename Domain::Action> plan(const typename Domain::State& startState) = 0;
};

}

#endif //METRONOME_OFFLINEPLANNER_HPP
