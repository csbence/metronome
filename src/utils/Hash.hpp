#pragma once

#include <cstdio>

namespace metronome {

template <typename T>
class Hash {
 public:
  std::size_t operator()(const T& value) const { return value.hash(); }

  std::size_t operator()(const T* value) const { return value->hash(); }
};

}  // namespace metronome
