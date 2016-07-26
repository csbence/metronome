#ifndef METRONOME_TUPLE_HPP
#define METRONOME_TUPLE_HPP

namespace metronome {
class PointTuple {
public:
    PointTuple(unsigned int x, unsigned int y) : x(x), y(y) {
    }
    unsigned int x;
    unsigned int y;
};
}

#endif // METRONOME_TUPLE_HPP
