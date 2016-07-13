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
}

#endif // METRONOME_TIMEMEASUREMENT_HPP
