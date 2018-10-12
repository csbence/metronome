#pragma once

/**
 *  Generic comparator functions
 */

#include <cstdint>

namespace metronome {

/**
 *  Requires that Node type have function f and member g
 */
template <typename Node>
int fComparator(const Node& lhs, const Node& rhs) {
  if (lhs.f() < rhs.f()) return -1;
  if (lhs.f() > rhs.f()) return 1;
  if (lhs.g > rhs.g) return -1;
  if (lhs.g < rhs.g) return 1;
  return 0;
}

/**
 *  Requires that Node type have member h
 */
template <typename Node>
int hComparator(const Node& lhs, const Node& rhs) {
  if (lhs.h < rhs.h) return -1;
  if (lhs.h > rhs.h) return 1;
  return 0;
}
} //namespace metronome
