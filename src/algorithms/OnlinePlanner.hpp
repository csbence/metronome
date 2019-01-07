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

  virtual void beginIteration() final {
    ++iterationCount;

    iterationAttributes.emplace_back();
  }

  virtual std::size_t getIterationCount() const final { return iterationCount; }

  virtual void incrementIdleIterationCount() final { ++idleIterationCount; }

  virtual std::size_t getIdleIterationCount() const final {
    return idleIterationCount;
  }

  virtual void goalFound() final {
    if (goalNodeFoundIteration == 0) goalNodeFoundIteration = iterationCount;
  }

  virtual std::size_t getGoalFirstFoundIteration() const final {
    return goalNodeFoundIteration;
  }

  virtual std::vector<std::pair<std::string, std::int64_t>> getAttributes()
      const override {
    // Calculate sum
    std::unordered_map<std::string, std::int64_t> attributeSums;
    std::unordered_map<std::string, std::int64_t> attributeCount;

    for (const auto& attributes : iterationAttributes) {
      for (const auto& attribute : attributes) {
        attributeSums[attribute.first] += attribute.second;
        ++attributeCount[attribute.first];
      }
    }

    auto attributes = Planner<Domain>::getAttributes();
    // Calculate average
    for (auto& attributeSum : attributeSums) {
      const auto frequency = attributeCount[attributeSum.first];
      const auto sum = attributeSum.second;
      attributes.emplace_back(attributeSum.first, sum / frequency);
    }

    return attributes;
  }

  virtual void recordAttribute(const std::string key,
                               std::int64_t value) final {
    assert(iterationAttributes.size() > iterationCount - 1);
    iterationAttributes[iterationCount - 1][key] = value;
  }

 private:
  std::vector<std::unordered_map<std::string, std::int64_t>>
      iterationAttributes;

  std::size_t iterationCount = 0;
  std::size_t idleIterationCount = 0;
  std::size_t goalNodeFoundIteration = 0;
};

}  // namespace metronome
