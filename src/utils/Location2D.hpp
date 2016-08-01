#ifndef METRONOME_LOCATION2D_HPP
#define METRONOME_LOCATION2D_HPP

namespace metronome {
class Location2D {
public:
    Location2D() : x(0), y(0) {}
    Location2D(int x, int y) : x(x), y(y) {}
    int x;
    int y;
};
}

#endif // METRONOME_LOCATION2D_HPP
