#ifndef VACUUM_WORLD_HPP
#define VACUUM_WORLD_HPP

#include "SuccessorBundle.hpp"
#include <boost/assert.hpp>
#include <boost/optional.hpp>
#include <cstdlib>
#include <experiment/Configuration.hpp>
#include <functional>
#include <limits>
#include <unordered_set>
#include <utils/Hasher.hpp>
#include <vector>

/*
 * NOTE: currently VWorld operates as GWorld
 * its assumed there is only one dirty cell
 * just for simplicity and getting stuff working
 */
namespace metronome {
class GridWorld {
public:
    typedef long long int Cost;
    static constexpr Cost COST_MAX = std::numeric_limits<Cost>::max();
    /*
     * State <- location of the agent as a pair
     * Action <- actionDuration representing the action taken {N,S,E,W,V} = {0,1,2,3,4}
     * Cost <- actionDuration for taking action from a state
     */
    class Action {
    public:
        Action() : actionDuration(0) {
        }
        Action(unsigned int actionDuration) : actionDuration(actionDuration) {
        }
        constexpr char toChar() const {
            if (actionDuration == 1) {
                return 'N';
            } else if (actionDuration == 2) {
                return 'E';
            } else if (actionDuration == 3) {
                return 'S';
            } else if (actionDuration == 4) {
                return 'W';
            } else {
                return '~';
            }
        }
        const std::string toString() const {
            std::string s;
            s.push_back(toChar());
            return s;
        }

    private:
        unsigned int actionDuration;
    };
    class State {
    public:
        State() : x(0), y(0) {
        }
        State(unsigned int x, unsigned int y) : x(x), y(y) {
        }
        State& operator=(State toCopy) {
            swap(*this, toCopy);
            return *this;
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
            return x ^ +y << 16 ^ y >> 16;
        }
        bool operator==(const State& state) const {
            return x == state.x && y == state.y;
        }

    private:
        unsigned int x;
        unsigned int y;

        friend void swap(State& first, State& second) {
            using std::swap;
            swap(first.x, second.x);
            swap(first.y, second.y);
        }
    };

public:
    GridWorld(const Configuration& configuration, std::istream& input) {
        if (!configuration.hasMember(ACTION_DURATION)) {
            throw MetronomeException("No actionDuration provided.");
        }
        actionDuration = configuration.getLong(ACTION_DURATION);
        obstacles = std::unordered_set<State, typename metronome::Hasher<State>>{};
        int currentHeight = 0;
        int currentWidth = 0;
        std::string line;
        char* end;
        getline(input, line); // get the width
        std::stringstream convertWidth(line);
        if (std::strtol(line.c_str(), &end, 10) == 0) {
            throw MetronomeException("GridWorld first line must be a number.");
        }
        convertWidth >> width;
        getline(input, line); // get the height
        if (std::strtol(line.c_str(), &end, 10) == 0) {
            throw MetronomeException("GridWorld second line must be a number.");
        }
        std::stringstream convertHeight(line);
        convertHeight >> height;

        boost::optional<State> tempStarState;
        boost::optional<State> tempGoalState;

        while (getline(input, line)) {
            for (auto it = line.cbegin(); it != line.cend(); ++it) {
                if (*it == '@') { // find the start location
                    tempStarState = State(currentWidth, currentHeight);
                } else if (*it == '*') { // find the goal location
                    tempGoalState = State(currentWidth, currentHeight);
                } else if (*it == '#') { // store the objects
                    State object = State(currentWidth, currentHeight);
                    obstacles.insert(object);
                } else {
                    // its an open cell nothing needs to be done
                }
                ++currentWidth; // at the end of the character parse move along
            }
            if (currentWidth != width) {
                throw MetronomeException("GridWorld is not complete. Width doesn't match input configuration.");
            }

            currentWidth = 0; // restart character parse at beginning of line
            ++currentHeight; // move down one line in charadter parse
        }

        if (currentHeight != height) {
            throw MetronomeException("GridWorld is not complete. Height doesn't match input configuration.");
        }

        if (!tempStarState.is_initialized() || !tempGoalState.is_initialized()) {
            throw MetronomeException("Unknown start or goal location. Start or goal location is not defined.");
        }

        startLocation = tempStarState.get();
        goalLocation = tempGoalState.get();
    }
    /*
     * Calculate the transition state given
     * a state and action pair
     */
    const State transition(const State& state, const Action& action) const {
        if (action.toChar() == 'N') {
            State newState = State(state.getX(), state.getY() - 1);
            if (isLegalLocation(newState)) {
                return newState;
            }
        } else if (action.toChar() == 'E') {
            State newState = State(state.getX() + 1, state.getY());
            if (isLegalLocation(newState)) {
                return newState;
            }
        } else if (action.toChar() == 'S') {
            State newState = State(state.getX(), state.getY() + 1);
            if (isLegalLocation(newState)) {
                return newState;
            }
        } else if (action.toChar() == 'W') {
            State newState = State(state.getX() - 1, state.getY());
            if (isLegalLocation(newState)) {
                return newState;
            }
        }

        return state;
    }

    const bool isGoal(const State& location) const {
        return location.getX() == goalLocation.getX() && location.getY() == goalLocation.getY();
    }

    const bool isObstacle(const State& location) const {
        return obstacles.find(location) != obstacles.end();
    }

    const bool isLegalLocation(const State& location) const {
        return location.getX() < width && location.getY() < height && !isObstacle(location);
    }

    const int getWidth() {
        return width;
    }

    const int getHeight() {
        return height;
    }

    const bool addObstacle(const State& toAdd) {
        if (isLegalLocation(toAdd)) {
            obstacles.insert(toAdd);
            return true;
        }
        return false;
    }

    const std::vector<State>::size_type getNumberObstacles() {
        return obstacles.size();
    }

    const State getStartState() const {
        return startLocation;
    }

    const bool isStart(const State& state) const {
        return state.getX() == startLocation.getX() && state.getY() == startLocation.getY();
    }

    Cost heuristic(const State& state) const {
        Cost horizontalDistance = goalLocation.getX() - state.getX();
        Cost verticalDistance = goalLocation.getY() - state.getY();
        return (std::abs(horizontalDistance) + std::abs(verticalDistance)) * actionDuration;
    }

    std::vector<SuccessorBundle<GridWorld>> successors(State state) const {
        std::vector<SuccessorBundle<GridWorld>> successors;

        unsigned int actions[] = {1, 2, 3, 4};

        for (auto a : actions) {

            State newState = transition(state, Action(a));
            if(!(newState == state)) {
                successors.push_back(SuccessorBundle<GridWorld>{newState, a, actionDuration});
            }
        }

        return successors;
    }

private:
    /*
     * maxActions <- maximum number of actions
     * width/height <- internal size representation of world
     * obstacles <- stores locations of the objects in world
     * dirtyCells <- stores locations of dirt in the world
     * startLocation <- where the agent begins
     * goalLocation <- where the agent needs to end up
     * initalAmountDirty <- how many cells are dirty
     * initialCost <- constant cost actionDuration
     * obstacles <- stores references to obstacles
     */
    unsigned int width;
    unsigned int height;
    std::unordered_set<State, typename metronome::Hasher<State>> obstacles;
    State startLocation = State();
    State goalLocation = State();
    Cost actionDuration;
};
}
#endif
