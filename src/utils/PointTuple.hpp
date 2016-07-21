#ifndef METRONOME_TUPLE_HPP
#define METRONOME_TUPLE_HPP

namespace metronome {
class Tuple {
public:
    Tuple(unsigned int x, unsigned int y) : x(x), y(y) {
    }
    unsigned int x;
    unsigned int y;
};
}

#endif // METRONOME_TUPLE_HPP
