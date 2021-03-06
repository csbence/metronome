#pragma once

#include <chrono>

namespace metronome {

class TimeTerminationChecker {
 public:
  void resetTo(long long timeLimit) {
    this->timeLimit = static_cast<std::chrono::nanoseconds>(timeLimit);
    startTime = std::chrono::high_resolution_clock::now();
    expansionCount = 0;
  }

  void setRatio(double ratio) {
    assert(ratio > 0 && ratio <= 1);
    this->ratio = ratio;
  }

  bool reachedTermination() const { return getEllapsedTime() >= timeLimit * ratio; };

  void notifyExpansion() { ++expansionCount; }

  unsigned int expansionsPerAction(unsigned long long actionDuration) const {
    return static_cast<unsigned int>(expansionCount * actionDuration /
                                     getEllapsedTime().count());
  }

 private:
  std::chrono::nanoseconds getEllapsedTime() const {
    std::chrono::time_point<std::chrono::high_resolution_clock,
                            std::chrono::nanoseconds>
        currentTime = std::chrono::high_resolution_clock::now();

    return currentTime - startTime;
  };

  std::chrono::time_point<std::chrono::high_resolution_clock,
                          std::chrono::nanoseconds>
      startTime;
  std::chrono::nanoseconds timeLimit{0};
  unsigned int expansionCount{0};
  double ratio{1};
};

}  // namespace metronome
