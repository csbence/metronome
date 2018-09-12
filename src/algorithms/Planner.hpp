#pragma once

namespace metronome {

class Planner {
 public:
  virtual unsigned long long getGeneratedNodeCount() const final {
    return generatedNodeCount;
  }

  virtual unsigned long long getExpandedNodeCount() const final {
    return expandedNodeCount;
  }

  virtual ~Planner() = default;

  virtual inline void incrementGeneratedNodeCount() final {
    ++generatedNodeCount;
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
