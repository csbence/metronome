#ifndef VACUUM_WORLD_HPP
#define VACUUM_WORLD_HPP

#include "SuccessorBundle.hpp"
#include <boost/assert.hpp>
#include <cstdlib>
#include <experiment/Configuration.hpp>
#include <functional>
#include <util/Hasher.hpp>
#include <vector>

/*
 * NOTE: currently VWorld operates as GWorld
 * its assumed there is only one dirty cell
 * just for simplicity and getting stuff working
 */
namespace metronome {
class GridWorld {
public:
    /*
     * State <- location of the agent as a pair
     * Action <- value representing the action taken {N,S,E,W,V} = {0,1,2,3,4}
     * Cost <- value for taking action from a state
     */
    class Action {
    private:
        unsigned int value;

    public:
        Action() : value(0) {
        }
        Action(unsigned int v) : value(v) {
        }
        constexpr char evaluate() const {
            if (value == 1) {
                return 'N';
            } else if (value == 2) {
                return 'S';
            } else if (value == 3) {
                return 'E';
            } else if (value == 4) {
                return 'W';
            } else if (value == 5) {
                return 'V';
            } else {
                return '~';
            }
        }
        /*constexpr unsigned int getValue() {
            return value;
        }
        void setValue(const unsigned int toSet) {
            value = toSet;
        }*/
    };
    typedef unsigned long Cost;
    class State {
    private:
        unsigned int x;
        unsigned int y;
        //        unsigned long cost;
        //        GridWorld::Action action;

        friend void swap(State& first, State& second) {
            using std::swap;
            swap(first.x, second.x);
            swap(first.y, second.y);
        }

        State(unsigned int x, unsigned int y, unsigned int a) : x(x), y(y) {
        }

    public:
        State& operator=(State toCopy) {
            swap(*this, toCopy);
            return *this;
        }

        static const State newState(unsigned int x, unsigned int y, unsigned int a = 0) {
            return State(x, y, a);
        }
        const unsigned int getX() const {
            const unsigned int ret = x;
            return ret;
        }
        const unsigned int getY() const {
            const unsigned int ret = y;
            return ret;
        }
        std::size_t hash() const {
            return x + 0x9e3779b9 + (y << 6) + (y >> 2);
        }
        bool operator==(const State& state) const {
            return x == state.x && y == state.y;
        }
    };

private:
    /*
     * maxActions <- maximum number of actions
     * width/height <- internal size representation of world
     * blockedCells <- stores locations of the objects in world
     * dirtyCells <- stores locations of dirt in the world
     * startLocation <- where the agent begins
     * goalLocation <- where the agent needs to end up
     * initalAmountDirty <- how many cells are dirty
     * initialCost <- constant cost value
     * obstacles <- stores references to obstacles
     */
    const unsigned int maxActions = 5;
    unsigned int width;
    unsigned int height;
    std::vector<State> blockedCells;
    std::vector<State> dirtyCells;
    State startLocation = State::newState(0, 0, 0);
    State goalLocation = State::newState(width - 1, height - 1, 0);
    unsigned int initialAmountDirty = 1;
    const unsigned long initialCost = 1;
    // std::unordered_map<State, State*, typename metronome::Hasher<State>> nodes{};

    /*
      * Given a state and action pair give the cost
      * for taking the action in the state
      * TODO: make it take a cost function instead of constant


    const Cost getCost(const State& s, const Action& a) {
        return initialCost;
    }

    const Action randomAction() {
        return Action(rand() & maxActions);
    }

     TODO: make allow more than one dirty cell
    const bool addDirtyCell(const State& toAdd) {
        if (isLegalLocation(toAdd)) {
            dirtyCells.push_back(toAdd);
            return true;
        }
        return false;
    }
     */

public:
    GridWorld(Configuration config, std::fstream input) {
        GridWorld{};
    }
    GridWorld(State start = State::newState(0, 0, 0), State goal = State::newState(4, 4, 0), unsigned int width = 5,
            unsigned int height = 5, std::vector<State> objectStates = std::vector<State>{})
            : width(width), height(height), blockedCells(objectStates), startLocation(start), goalLocation(goal) {
    }

    /*
     * Calculate the transition state given
     * a state and action pair
     * TODO: make allow more than one dirty cell
     */
    const State transition(const State& s, const Action& a) const {
        if (a.evaluate() == 'N') {
            State n = s.newState(s.getX(), s.getY() - 1, 0);
            if (isLegalLocation(n)) {
                return n;
            }
        } else if (a.evaluate() == 'E') {
            State n = s.newState(s.getX() + 1, s.getY(), 1);
            if (isLegalLocation(n)) {
                return n;
            }
        } else if (a.evaluate() == 'S') {
            State n = s.newState(s.getX(), s.getY() + 1, 2);
            if (isLegalLocation(n)) {
                return n;
            }
        } else if (a.evaluate() == 'W') {
            State n = s.newState(s.getX() - 1, s.getY(), 3);
            if (isLegalLocation(n)) {
                return n;
            }
        }
        return s;
    }

    std::pair<unsigned int, unsigned int> randomLocation() {
        unsigned int x = rand() % width;
        unsigned int y = rand() % height;

        return std::pair<unsigned int, unsigned int>{x, y};
    }

    const bool isGoal(const State& location) const {
        return location.getX() == goalLocation.getX() && location.getY() == goalLocation.getY();
    }

    const bool inBlockedCells(const State& location) const {
        for (auto it = blockedCells.cbegin(); it != blockedCells.cend(); ++it) {
            if (it->getX() == location.getX() && it->getY() == location.getY()) {
                return true;
            }
        }
        return false;
    }

    const bool isLegalLocation(const State& location) const {
        return location.getX() < width && location.getY() < height && !inBlockedCells(location);
    }

    void setWidth(unsigned int newWidth) {
        width = newWidth;
    }

    void setHeight(unsigned int newHeight) {
        height = newHeight;
    }

    const int getWidth() {
        return width;
    }

    const int getHeight() {
        return height;
    }

    const bool addBlockedCell(const State& toAdd) {
        if (isLegalLocation(toAdd)) {
            blockedCells.push_back(toAdd);
            return true;
        }
        return false;
    }

    const std::vector<State>::size_type getNumberBlockedCells() {
        return blockedCells.size();
    }

    const std::vector<State>::size_type getNumberDirtyCells() {
        // return dirtyCells.size();
        return initialAmountDirty;
    }

    const State getStartState() const {
        return startLocation;
    }

    const bool isStart(const State& state) const {
        return state.getX() == startLocation.getX() && state.getY() == startLocation.getY();
    }

    Cost heuristic(const State& state) const {
        Cost manhattenDistance = 0;

        Cost horizontalDistance = this->goalLocation.getX() - state.getX();
        Cost verticalDistance = this->goalLocation.getY() - state.getY();

        manhattenDistance = horizontalDistance + verticalDistance;

        return manhattenDistance;
    }

    std::vector<SuccessorBundle<GridWorld>> successors(State state) const {
        std::vector<SuccessorBundle<GridWorld>> successors;

        unsigned int actions[] = {1, 2, 3, 4, 5};

        for (auto a : actions) {
            State newState = this->transition(state, Action(a));
            successors.push_back(SuccessorBundle<GridWorld>{newState, a, this->initialCost});
        }

        return successors;
    }
};
}
#endif
