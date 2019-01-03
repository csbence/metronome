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

  virtual void incrementIterationCount() final {
    ++iterationCount;
  }

  virtual std::size_t getIterationCount() const final {
    return iterationCount;
  }
  
  virtual void incrementIdleIterationCount() final {
    ++idleIterationCount;
  }

  virtual std::size_t getIdleIterationCount() const final {
    return idleIterationCount;
  }
  
  virtual void goalFound() final {
    if (goalNodeFoundIteration == 0)
      goalNodeFoundIteration = iterationCount;
  }

  virtual std::size_t getGoalFirstFoundIteration() const final {
    return goalNodeFoundIteration;
  }

  
 private:
  std::size_t iterationCount = 0;
  std::size_t idleIterationCount = 0;
  std::size_t goalNodeFoundIteration = 0;
};

}  // namespace metronome
