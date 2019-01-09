#pragma once

#include <ostream>
#include <string>

namespace metronome {

template <typename Domain>
class Planner {
 public:
  class ActionBundle final {
   public:
    ActionBundle() = default;
    ActionBundle(typename Domain::Action action,
                 typename Domain::Cost actionDuration)
        : action{action}, actionDuration{actionDuration} {}
    ActionBundle(const ActionBundle&) = default;
    ActionBundle(ActionBundle&&) = default;
    ActionBundle& operator=(const ActionBundle&) = default;
    ActionBundle& operator=(ActionBundle&&) = default;
    ~ActionBundle() = default;

    friend std::ostream& operator<<(std::ostream& os,
                                    const ActionBundle& bundle) {
      os << "Action: " << bundle.action
         << " expectedTargetState: " << bundle.expectedTargetState
         << " label: " << bundle.label;
      return os;
    }

    typename Domain::Action action;
    typename Domain::Cost actionDuration;
    typename Domain::State expectedTargetState;
    std::string label;
  };

  friend std::ostream& operator<<(
      std::ostream& os,
      const std::vector<typename Planner<Domain>::ActionBundle>&
          actionBundles) {
    for (const auto& actionBundle : actionBundles) {
      os << actionBundle << "\n";
    }
    return os;
  }

  virtual std::size_t getGeneratedNodeCount() const final {
    return generatedNodeCount;
  }

  virtual std::size_t getExpandedNodeCount() const final {
    return expandedNodeCount;
  }

  virtual ~Planner() = default;

  virtual inline void incrementGeneratedNodeCount() final {
    ++generatedNodeCount;
  }
  
  virtual std::vector<std::pair<std::string, std::int64_t>> getAttributes() 
  const {
    return {};
  }

 protected:
  virtual inline void incrementExpandedNodeCount() final {
    ++expandedNodeCount;
  }

 private:
  unsigned long long generatedNodeCount{0};
  unsigned long long expandedNodeCount{0};
};

}  // namespace metronome
