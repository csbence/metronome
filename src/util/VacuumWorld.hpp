#ifndef VACUUM_WORLD_HPP
#define VACUUM_WORLD_HPP

#include <boost/assert.hpp>
#include <cstdlib>
#include <functional>
#include <vector>

class VacuumWorld {
public:
    /***
    *** State <- location of the agent as a pair
    *** Action <- value representing the action taken {N,S,E,W,V} = {0,1,2,3,4}
    *** Cost <- value for taking action from a state
    ***/
    class State {
    private:
        unsigned int _x;
        unsigned int _y;
        State(unsigned int x, unsigned int y) : _x(x), _y(y) {
        }

    public:
        State& operator=(State toCopy) {
            std::cout << "THIS_X_Y: " << this->getX() << " " << this->getY()
                      << std::endl;
            std::cout << "COPY_X_Y: " << toCopy.getX() << " " << toCopy.getY()
                      << std::endl;
            swap(*this, toCopy);
            std::cout << "THIS_X_Y: " << this->getX() << " " << this->getY()
                      << std::endl;
            std::cout << "COPY_X_Y: " << toCopy.getX() << " " << toCopy.getY()
                      << std::endl;
            return *this;
        }
        friend void swap(State& first, State& second) {
            using std::swap;
            swap(first._x, second._x);
            swap(first._y, second._y);
        }
        static const State newState(
                const unsigned int x, const unsigned int y) {
            return State(x, y);
        }
        const unsigned int getX() const {
            const unsigned int ret = _x;
            return ret;
        }
        const unsigned int getY() const {
            const unsigned int ret = _y;
            return ret;
        }
        std::size_t hash() const {
            return _x + 0x9e3779b9 + (_y << 6) + (_y >> 2);
        }
        bool operator==(const State& state) const {
            return _x == state._x && _y == state._y;
        }
    };
    class Action {
    private:
        unsigned int _value;

    public:
        constexpr char evaluate() {
            return 'X';
        }
        constexpr unsigned int value() {
            return _value;
        }
        void setValue(const unsigned int toSet) {
            _value = toSet;
        }
    };
    typedef unsigned long Cost;

private:
    /***
    *** maxActions <- maximum number of actions
    *** width/height <- internal size representation of world
    *** blockedCells <- stores locations of the objects in world
    *** dirtyCells <- stores locations of dirt in the world
    *** startLocation <- where the agent begins
    *** goalLocation <- where the agent needs to end up
    *** initalAmountDirty <- how many cells are dirty
    *** initialCost <- constant cost value
    ***/
    const unsigned int maxActions = 5;
    unsigned int width = 0;
    unsigned int height = 0;
    std::vector<State> blockedCells;
    std::vector<State> dirtyCells;
    State startLocation = State::newState(0, 0);
    State goalLocation = State::newState(0, 0);
    unsigned int initialAmountDirty = 1;
    const unsigned long initialCost = 1.0;

public:
    /***
     *** Given a state and action pair give the cost
     *** for taking the action in the state
     *** TODO: make it take a cost function instead of constant
     ***/
    const Cost getCost(State s, Action a) {
        return initialCost;
    }

    const Action randomAction() {
        Action a;
        a.setValue(rand() % maxActions);
        return a;
    }

    const State randomLocation() {
        unsigned int x = rand() % width;
        unsigned int y = rand() % height;

        return State::newState(x, y);
    }

    const State getGoal() {
        return State::newState(goalLocation.getX(), goalLocation.getY());
    }

    const bool isGoal(State location) {
        return location.getX() == goalLocation.getX() &&
                location.getY() == goalLocation.getY();
    }

    const bool inBlockedCells(State location) {
        for (auto it : blockedCells) {
            if (it.getX() == location.getX() && it.getY() == location.getY()) {
                return true;
            }
        }
        return false;
    }

    const bool isLegalLocation(State location) {
        return location.getX() < width && location.getY() < height &&
                !inBlockedCells(location);
    }

    void setWidth(const int newWidth) {
        width = newWidth;
    }

    void setHeight(const int newHeight) {
        height = newHeight;
    }

    const int getWidth() {
        return width;
    }

    const int getHeight() {
        return height;
    }

    const bool addBlockedCell(State toAdd) {
        if (isLegalLocation(toAdd)) {
            blockedCells.push_back(toAdd);
            return true;
        }
        return false;
    }

    const bool addDirtyCell(const State toAdd) {
        if (isLegalLocation(toAdd)) {
            dirtyCells.push_back(toAdd);
            return true;
        }
        return false;
    }

    const bool changeStartLocation(const State location) {
        if (isLegalLocation(location)) {
            startLocation = State::newState(location.getX(), location.getY());
            return true;
        }
        return false;
    }

    const std::vector<State>::size_type getNumberBlockedCells() {
        return blockedCells.size();
    }

    const std::vector<State>::size_type getNumberDirtyCells() {
        return dirtyCells.size();
    }

    const State getStartLocation() {
        return startLocation;
    }
};
#endif
