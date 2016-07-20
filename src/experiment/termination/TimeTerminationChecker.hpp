#ifndef METRONOME_TIMETERMINATIONCHECKER_HPP
#define METRONOME_TIMETERMINATIONCHECKER_HPP

#include <chrono>

namespace metronome {

class TimeTerminationChecker {
public:
    void resetTo(std::chrono::nanoseconds timeLimit) {
        this->timeLimit = timeLimit;
        startTime = std::chrono::high_resolution_clock::now();
    }

    bool reachedTermination() const {
        std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::nanoseconds> currentTime =
                std::chrono::high_resolution_clock::now();

        std::chrono::nanoseconds ellapsedTime = currentTime - startTime;

        return ellapsedTime >= timeLimit;
    };

private:
    std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::nanoseconds> startTime;
    std::chrono::nanoseconds timeLimit;
};
}

#endif // METRONOME_TIMETERMINATIONCHECKER_HPP
