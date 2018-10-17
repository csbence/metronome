#pragma once

namespace metronome {

class TestDomain {
 public:
  class State {
   private:
    unsigned int x;

   public:
    bool operator==(const State &state) const { return x == state.x; }
    std::size_t hash() const { return 0; }
    State(unsigned int x) : x(x) {}
  };

  class Action {};
  typedef unsigned long Cost;

  Cost heuristic(State &) { return 0; }
};

}  // namespace metronome
