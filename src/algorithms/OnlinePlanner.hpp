#pragma once

#include <vector>
#include "Planner.hpp"
#include "experiment/termination/TimeTerminationChecker.hpp"

namespace metronome {

template <typename Domain, typename TerminationChecker>
class OnlinePlanner : public Planner<Domain> {
 public:
  ~OnlinePlanner() override = default;

  virtual std::vector<typename Planner<Domain>::ActionBundle> selectActions(
      const typename Domain::State& startState,
      TerminationChecker& terminationChecker) = 0;

  virtual void incrementIdleIterationCount() final {
    ++idleIterationCount;
  }

  virtual std::size_t getIdleIterationCount() const final {
    return idleIterationCount;
  }
  
 private:
  std::size_t idleIterationCount = 0;
};

}  // namespace metronome
