#pragma once

/**
 *  Generic comparator functions
 */

#include <cstdint>
#include "Nodes.hpp"

namespace metronome {

/**
 *  Requires that Node type have function f and member g
 */
template <typename Domain>
int fComparator(const SearchNode<Domain>& lhs, const SearchNode<Domain>& rhs) {
  if (lhs.f() < rhs.f()) return -1;
  if (lhs.f() > rhs.f()) return 1;
  if (lhs.g > rhs.g) return -1;
  if (lhs.g < rhs.g) return 1;
  return 0;
}

/**
 *  Requires that Node type have member h
 */
template <typename Domain>
int hComparator(const SearchNode<Domain>& lhs, const SearchNode<Domain>& rhs) {
  if (lhs.h < rhs.h) return -1;
  if (lhs.h > rhs.h) return 1;
  return 0;
}
} //namespace metronome
