#ifndef METRONOME_TUPLE_HPP
#define METRONOME_TUPLE_HPP

namespace metronome {
class Location2D {
public:
    Location2D() : x(0), y(0) {

    }
    Location2D(unsigned int x, unsigned int y) : x(x), y(y) {
    }
    unsigned int x;
    unsigned int y;
};
}

#endif // METRONOME_TUPLE_HPP
