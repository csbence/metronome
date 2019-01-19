#pragma once

#include <chrono>
#include <functional>
#include <cstdio>
namespace metronome {

long long int measureNanoTime(std::function<void()> action) {
  auto begin = std::chrono::high_resolution_clock::now();

  action();

  auto end = std::chrono::high_resolution_clock::now();

  return std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin)
      .count();
}

long long int currentNanoTime() {
  auto now = std::chrono::high_resolution_clock::now().time_since_epoch();

  return std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
}

void logTime(const std::string message = "") {
  static long long int startTime = currentNanoTime();
  LOG(INFO) << (currentNanoTime() - startTime) / 1000000 << message;
}

// Macro wraps block to measure its execution time. Alternative to using lambda
// with measureNanoTime for non-function code
struct PrintCtrl {
  bool flag = false;
  std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();
};

#if CMAKE_BUILD_TYPE == Debug 
#define PRINT_NANO_TIME(msg) \
  for (PrintCtrl MACRO_PRINT_CTRL{}; \
      MACRO_PRINT_CTRL.flag == false; \
      MACRO_PRINT_CTRL.flag = true, \
        printf("%s: %lld ns\n", msg, std::chrono::high_resolution_clock::now() - MACRO_PRINT_CTRL.begin ))
#else
#define PRINT_NANO_TIME(msg)
#endif

}  // namespace metronome
