#ifndef METRONOME_LOCATION2D_HPP
#define METRONOME_LOCATION2D_HPP

namespace metronome {
class Location2D {
public:
    Location2D() : x(0), y(0) {}
    Location2D(int x, int y) : x(x), y(y) {}
    Location2D operator=(Location2D toCopy) {
        swap(*this, toCopy);
        return *this;
    }
    int x;
    int y;
private:
    friend void swap(Location2D& first, Location2D& second) {
        using std::swap;
        swap(first.x,second.x);
        swap(first.x,second.y);
    }
};
}

#endif // METRONOME_LOCATION2D_HPP
