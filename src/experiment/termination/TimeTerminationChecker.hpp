#ifndef METRONOME_TIMETERMINATIONCHECKER_HPP
#define METRONOME_TIMETERMINATIONCHECKER_HPP

#include <chrono>

namespace metronome {

class TimeTerminationChecker {
public:
    void resetTo(long long timeLimit) {
        this->timeLimit = static_cast<std::chrono::nanoseconds>(timeLimit);
        startTime = std::chrono::high_resolution_clock::now();
        expansionCount = 0;
    }

    void setRatio(double) {
        throw MetronomeException("Function not implemented");
    }

    bool reachedTermination() const { return getEllapsedTime() >= timeLimit; };

    void notifyExpansion() { ++expansionCount; }

    unsigned int expansionsPerAction(unsigned long long actionDuration) const {
        return static_cast<unsigned int>(expansionCount * actionDuration / getEllapsedTime().count());
    }

private:
    std::chrono::nanoseconds getEllapsedTime() const {
        std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::nanoseconds> currentTime =
                std::chrono::high_resolution_clock::now();

        return currentTime - startTime;
    };

    std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::nanoseconds> startTime;
    std::chrono::nanoseconds timeLimit{0};
    unsigned int expansionCount{0};
};
}

#endif // METRONOME_TIMETERMINATIONCHECKER_HPP
