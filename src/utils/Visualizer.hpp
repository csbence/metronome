#ifndef METRONOME_VISUALIZER_HPP
#define METRONOME_VISUALIZER_HPP

#include <iostream>
#include <vector>

namespace metronome {
template <typename Domain>
class Visualizer {
public:
    Visualizer(Domain& domain) : domain(domain) {
    }
    void visualize(std::ostream& display) const {
        domain.visualize(display);
    }
    void animate(std::ostream& display, const std::vector<Domain::Action> actions) const {
        domain.animate(display, actions);
    }

private:
    Domain domain;
};
}

#endif // METRONOME_VISUALIZER_HPP
