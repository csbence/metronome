#ifndef VACUUM_WORLD_HPP
#define VACUUM_WORLD_HPP

#include <boost/assert.hpp>
#include <cstdlib>
#include <functional>
#include <vector>

/*
 * NOTE: currently VWorld operates as GWorld
 * its assumed there is only one dirty cell
 * just for simplicity and getting stuff working
 */

class VacuumWorld {
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
        Action(unsigned int v) : value(v) { }
        constexpr char evaluate() const {
            if(value == 0) {
                return 'N';
            }
            else if(value == 1) {
                return 'S';
            }
            else if(value == 2) {
                return 'E';
            }
            else if(value == 3) {
                return 'W';
            }
            else if (value == 4){
                return 'V';
            }
            else {
                return '~';
            }
        }
        constexpr unsigned int getValue() {
            return value;
        }
        void setValue(const unsigned int toSet) {
            value = toSet;
        }
    };
    typedef unsigned long Cost;
    class State {
    private:
        unsigned int x;
        unsigned int y;
        unsigned long cost;
        VacuumWorld::Action action;

        friend void swap(State first, State second) {
            using std::swap;
            swap(first.x, second.x);
            swap(first.y, second.y);
        }

        State(unsigned int x, unsigned int y, unsigned int a) : x(x), y(y), cost(1.0), action(a) { }

    public:
        const unsigned int getCost() const {
            return this->cost;
        }
        const VacuumWorld::Action getAction() const {
            return this->action;
        }
        State& operator=(State toCopy) {
            swap(*this, toCopy);
            return *this;
        }

        static const State newState(
                unsigned int x, unsigned int y, unsigned int a = -1) {
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
     */
    const unsigned int maxActions = 5;
    unsigned int width;
    unsigned int height;
    std::vector<State> blockedCells;
    std::vector<State> dirtyCells;
    State startLocation = State::newState(0, 0, -1);
    State goalLocation = State::newState(width-1, height-1, -1);
    unsigned int initialAmountDirty = 1;
    const unsigned long initialCost = 1.0;

public:

    VacuumWorld(State start = State::newState(0,0,-1), State goal = State::newState(4,4,-1),
                unsigned int width = 5, unsigned int height = 5,
                std::vector<State> objectStates = std::vector<State>{}) :
            width(width),
            height(height),
            blockedCells(objectStates),
            startLocation(start),
            goalLocation(goal)
    {}


    /*
     * Calculate the transition state given
     * a state and action pair
     * TODO: make allow more than one dirty cell
     */
    const State transition(const State &s, const Action &a) {
        if(a.evaluate() == 'N') {
            State n = s.newState(s.getX(),s.getY()-1,0);
            if(isLegalLocation(n)) {
                return n;
            }
        }
        else if(a.evaluate() == 'E') {
            State n = s.newState(s.getX()+1,s.getY(),1);
            if(isLegalLocation(n)) {
                return n;
            }
        }
        else if(a.evaluate() == 'S') {
            State n = s.newState(s.getX(),s.getY()+1,2);
            if(isLegalLocation(n)) {
                return n;
            }
        }
        else if(a.evaluate() == 'W') {
            State n = s.newState(s.getX()-1,s.getY(),3);
            if(isLegalLocation(n)) {
                return n;
            }
        }
        return s;
    }

    /*
     * Given a state and action pair give the cost
     * for taking the action in the state
     * TODO: make it take a cost function instead of constant
     */

    const Cost getCost(const State& s, const Action& a) {
        return initialCost;
    }

    const Action randomAction() {
        return Action(rand() & maxActions);
    }

    std::pair<unsigned int,unsigned int> randomLocation() {
        unsigned int x = rand() % width;
        unsigned int y = rand() % height;

        return std::pair<unsigned int, unsigned int>{x,y};
    }

    const State getGoal() {
        return goalLocation;
    }

    const bool isGoal(const State& location) {
        return location.getX() == goalLocation.getX() &&
                location.getY() == goalLocation.getY();
    }

    const bool inBlockedCells(const State& location) {
        for (auto it : blockedCells) {
            if (it.getX() == location.getX() && it.getY() == location.getY()) {
                return true;
            }
        }
        return false;
    }

    const bool isLegalLocation(const State& location) {
        return location.getX() < width && location.getY() < height &&
                !inBlockedCells(location);
    }

    void setWidth(int newWidth) {
        width = newWidth;
    }

    void setHeight(int newHeight) {
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

    const bool addDirtyCell(const State& toAdd) {
        if (isLegalLocation(toAdd)) {
            dirtyCells.push_back(toAdd);
            return true;
        }
        return false;
    }

    const bool changeStartLocation(const State& location) {
        if (isLegalLocation(location)) {
            startLocation = State::newState(location.getX(), location.getY(),-1);
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

    const State getStartLocation() {
        return startLocation;
    }

    Cost heuristic(State&) {
        return 0;
    }

    std::vector<State> successors(State q) {
        std::vector<State> ret;

        auto actions = {0,1,2,3,4};

        for (auto a : actions) {
            this->transition(q,Action(a));
        }

        return ret;
    }
};
#endif
