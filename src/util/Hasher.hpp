#ifndef METRONOME_HASHER_HPP
#define METRONOME_HASHER_HPP

#include <cstdio>

namespace metronome {

template<typename T>
class Hasher {
 public:
  std::size_t operator()(const T& value) const { return value.hash(); }
};
}

#endif // METRONOME_HASHER_HPP
