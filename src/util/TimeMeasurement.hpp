#ifndef METRONOME_TIMEMEASUREMENT_HPP
#define METRONOME_TIMEMEASUREMENT_HPP

#include <chrono>
#include <functional>
namespace metronome {

long long int measureNanoTime(std::function<void()> action) {
    auto begin = std::chrono::high_resolution_clock::now();

    action();

    auto end = std::chrono::high_resolution_clock::now();

    return std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count();
}

long long int currentNanoTime() {
    auto now = std::chrono::high_resolution_clock::now().time_since_epoch();

    return std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
}

void logTime(const std::string message = "") {
    static long long int startTime = currentNanoTime();
    LOG(INFO) << (currentNanoTime() - startTime) / 1000000 << message;
}

}

#endif // METRONOME_TIMEMEASUREMENT_HPP
