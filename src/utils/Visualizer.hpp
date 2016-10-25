#ifndef METRONOME_VISUALIZER_HPP
#define METRONOME_VISUALIZER_HPP

#include <iostream>
#include <vector>

namespace metronome {
template <typename Domain>
class Visualizer {
public:
    Visualizer(Domain& domain) : domain(domain) {}
    void visualize(std::ostream& display,
            const typename Domain::State& state,
            const typename Domain::Action& action) const {
        domain.visualize(display, state, action);
    }
    void animate(std::ostream& display, const std::vector<typename Domain::Action> actions) const {
        domain.animate(display, actions);
    }

private:
    Domain domain;
};
}

#endif // METRONOME_VISUALIZER_HPP
