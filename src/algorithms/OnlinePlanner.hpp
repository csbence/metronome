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
    auto strategies = getAttributeAggregationStrategies();
    // Calculate aggregation
    for (auto& attributeSum : attributeSums) {
      const auto sum = attributeSum.second;
      std::string strategy = strategies[attributeSum.first];

      std::int64_t agg{0};

      if (strategy == "sum") {
        agg = sum;
      } else { // Default strategy is average
        const auto frequency = attributeCount[attributeSum.first];
        agg = sum / frequency;
      }

      attributes.emplace_back(attributeSum.first, agg);
    }

    return attributes;
  }

  /**
   * Override to provide strategies for aggregations.
   * Current supported strategies are "average" and "sum"
   * Default is average, which averages over the number of times the attribute
   * was reported
   * @return map of attribute names to strategies
   */
  virtual std::unordered_map<std::string, std::string> getAttributeAggregationStrategies() const {
    return {};
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
