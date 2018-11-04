#pragma once

#include <vector>
#include "Planner.hpp"
namespace metronome {

template <typename Domain>
class OfflinePlanner : public Planner<Domain> {
 public:
  virtual ~OfflinePlanner() = default;
  virtual std::vector<typename Domain::Action> plan(
      const typename Domain::State& startState) = 0;
};

}  // namespace metronome
