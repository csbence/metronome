#pragma once

#include "Configuration.hpp"
#include "Result.hpp"
namespace metronome {

template <typename Domain, typename Planner>
class PlanManager {
 public:
  virtual Result plan(const Configuration&, const Domain&, Planner&) = 0;  //
};

}  // namespace metronome
