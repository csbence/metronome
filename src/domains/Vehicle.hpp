#ifndef METRONOME_VEHICLE_HPP
#define METRONOME_VEHICLE_HPP

#include <boost/assert.hpp>
#include <cstdlib>
#include <ctime>
#include <functional>
#include <unordered_map>
#include <utils/Location2D.hpp>
#include <vector>
#include "GridWorld.hpp"
#include "SuccessorBundle.hpp"

namespace metronome {
class Vehicle {
public:
    typedef long long int Cost;
    static constexpr Cost COST_MAX = std::numeric_limits<Cost>::max();

    class Action {
    public:
        Action() : value(0) {}
        Action(unsigned int actionDuration) : value(actionDuration) {}
        constexpr char toChar() const {
            if (value == 1) {
                return 'N';
            } else if (value == 2) {
                return 'E';
            } else if (value == 3) {
                return 'S';
            } else if (value == 4) {
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
        unsigned int value;
    };

    class State {
    public:
        State() : x(0), y(0), xVelocity(0), yVelocity(0), index(x + y) {}
        State(const unsigned int x, const unsigned int y) : x(x), y(y) {
            if (randomSeedFlag) {
                std::srand(randomSeed);
            } else {
                std::srand(std::time(0));
            }

            if (generatedStates[x][y] != nullptr) {
                xVelocity = std::rand() % 3;
                yVelocity = std::rand() % 3;
                index = x + y;
                generatedStates[x][y] = this;
            } else {
                xVelocity = generatedStates[x][y]->getXVelocity();
                yVelocity = generatedStates[x][y]->getYVelocity();
                index = generatedStates[x][y]->index;
            }
        }
        State& operator=(State toCopy) {
            swap(*this, toCopy);
            return *this;
        }

        unsigned int getX() const { return x; }
        unsigned int getY() const { return y; }
        int getXVelocity() const { return xVelocity; }
        int getYVelocity() const { return yVelocity; }
        std::size_t hash() const { return x ^ y << 16 ^ y >> 16; }
        bool operator=(const State& state) const {
            return x == state.x && y == state.y && xVelocity == state.xVelocity && yVelocity == state.yVelocity;
        }
        const std::string toString() const {
            std::string string("x: ");
            return string + std::to_string(x) + " y: " + std::to_string(y);
        }

    private:
        unsigned int x;
        unsigned int y;
        int xVelocity;
        int yVelocity;
        int index;

        friend void swap(State& first, State& second) {
            using std::swap;
            swap(first.x, second.x);
            swap(first.y, second.y);
            swap(first.xVelocity, second.xVelocity);
            swap(first.yVelocity, second.yVelocity);
        }
    };

    Vehicle(const Configuration& configuration, std::istream& input) {
        if (!configuration.hasMember(ACTION_DURATION)) {
            throw MetronomeException("No value provided.");
        }
        actionDuration = configuration.getLong(ACTION_DURATION);
        unsigned int currentHeight = 0;
        unsigned int currentWidth = 0;
        unsigned int currentIndex = 0;
        std::string line;
        char* end;
        getline(input, line);
        std::stringstream convertWidth(line);
        if (std::strtol(line.c_str(), &end, 10) == 0) {
            throw MetronomeException("Vehicle first line must be a number.");
        }
        convertWidth >> width;
        getline(input, line);
        if (std::strtol(line.c_str(), &end, 10) == 0) {
            throw MetronomeException("Vehicle second line must be a number.");
        }
        std::stringstream convertHeight(line);
        convertHeight >> height;

        for (auto i = 0; i < width; ++i) {
            for (auto j = 0; j < height; ++j) {
                obstacles[i][j] = false;
                bunkers[i][j] = false;
            }
        }

        boost::optional<State> tempStartState;
        boost::optional<State> tempGoalState;

        while (getline(input, line)) {
            for (auto it = line.cbegin(); it != line.cend(); ++it) {
                // do something for IO
                if (*it == '@') { // find the start location
                    tempStartState = State(currentWidth, currentHeight);
                } else if (*it == '*') { // find the goal location
                    tempGoalState = State(currentWidth, currentHeight);
                } else if (*it == '#') { // store the objects
                    obstacleIndices.push_back(metronome::Location2D(currentWidth, currentHeight));
                    obstacles[currentWidth][currentHeight] = true;
                } else if (*it == '$') {
                    bunkerIndices.push_back(metronome::Location2D(currentWidth, currentHeight));
                    bunkers[currentWidth][currentHeight] = true;
                } else {
                    // open cell!
                }
                ++currentWidth;
                ++currentIndex;
            }
            if (currentWidth != width) {
                throw MetronomeException("Vehicle is not complete. Widthd doesn't match the input configuration.");
            }
            currentWidth = 0;
            ++currentHeight;
        }

        if (currentHeight != height) {
            throw MetronomeException("Vehicle is not complete. Height doesn't match input configuration.");
        }

        if (!tempStartState.is_initialized() || !tempGoalState.is_initialized()) {
            throw MetronomeException("Vehicle unknown start or goal location. Start or goal location is not defined.");
        }

        startLocation = tempStartState.get();
        startLocation = tempGoalState.get();
    }
    const State transition(const State& state, const Action& action) {
        moveObstacles();

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
    const bool isObstacle(const State& location) const { return obstacles[location.getX()][location.getY()]; }
    const bool isLegalLocation(const State& location) const {
        return location.getX() < width && location.getY() < height && !isObstacle(location);
    }
    /*
     * this needs to be fixed....
     */
    std::vector<SuccessorBundle<Vehicle>> successors(State state) {
        std::vector<SuccessorBundle<Vehicle>> successors;

        unsigned int actions[] = {1, 2, 3, 4, 5};

        for (auto a : actions) {
            State newState = this->transition(state, Action(a));
            for (auto it = obstaclesLocations.begin(); it != obstaclesLocations.end(); ++it) {
                if (*it == newState) {
                    successors.push_back(SuccessorBundle<Vehicle>{newState, a, deadCost});
                } else {
                    successors.push_back(SuccessorBundle<Vehicle>{newState, a, actionDuration});
                }
            }
        }

        return successors;
    }
    //    const bool addDyanmicObject(const State& toAdd) {
    //        return this->addObstacle(toAdd);
    //    }
    //
    //
    //
    //    const State getStartLocation() {
    //        return GridWorld::getStartState();
    //    }
    //
    //    Cost heuristic(const State& state) {
    //        return GridWorld::heuristic(state);
    //    }
    //
    //    const bool isGoal(const State& location) {
    //        return GridWorld::isGoal(location);
    //    }
    //
    //    const bool isStart(const State& state) {
    //        return GridWorld::isStart(state);
    //    }
private:
    void moveObstacles() {
        for (auto obstacleIndex : obstacleIndices) {
            auto curState = generatedStates[obstacleIndex.x][obstacleIndex.y];
            obstacles[obstacleIndex.x][obstacleIndex.y] = false;
            int modX = curState->getXVelocity();
            int modY = curState->getYVelocity();

            if (obstacleIndex.x + modX > this->width || obstacleIndex.x + modX < 0) {
                modX *= -1; // hit the wall of the grid bounce off
            }
            if (obstacleIndex.y + modY > this->height || obstacleIndex.y + modY < 0) {
                modY *= -1; // hit the wall of the grid bounce off
            }
            auto inObstacle = bunkers[obstacleIndex.x + modX][obstacleIndex.y + modY];

            if (!inObstacle) { // if it is hitting an obstacle bounce
                obstacles[curState->getX() + (modX * -1)][curState->getY() + (modY * -1)] = true;
                State newState = State(curState->getX() + (modX * -1), curState->getY() + (modY * -1));
                curState = &newState;
                generatedStates[obstacleIndex.x][obstacleIndex.y] = curState;
                // need to update the new obstacleIndex don't know how
            } else { // otherwise just move to the new location
                obstacles[curState->getX() + modX][curState->getY() + modY] = true;
                State newState = State(curState->getX() + modX, curState->getY() + modY);
                curState = &newState;
                generatedStates[obstacleIndex.x][obstacleIndex.y] = curState;
                // need to update the new obstacleIndex don't know how
            }
        }
    }

    /*
     * randomSeedFlag <- notifies the ctor if we are using internal seed or user-defined
     * randomSeed <- the seed we use for generating the velocities of the obstacles
     * width <- how wide the world is
     * height <- how tall the world is
     * obstacleIndices <- where the obstacles are using direct addressing *NOTE: vector is a container for a
     * bunkerIndices <- where the bunkers are using direct addressing      *NOTE: dynamic array!!!!!!!!!!!!!!
     * obstacles <- bit vector using direct addressing of the obstacles
     * bunkers <- bit vector using direct addressing of the bunkers ** NOTE: are these redundant?***
     * startLocation <- where the agent starts
     * goalLocation <- where the agent needs to go
     * deadCost <- to calculate if we are in a dead state if the cost is twice the actionDuration then we pronounce
     * it
     * dead
     * generatedStates <- our cache trick to insure velocity consistency when generating states
     */
    static bool randomSeedFlag;
    static long randomSeed;
    unsigned int width;
    unsigned int height;
    std::vector<metronome::Location2D> obstacleIndices;
    std::vector<metronome::Location2D> bunkerIndices;
    std::vector<std::vector<bool>> obstacles;
    std::vector<std::vector<bool>> bunkers;
    State startLocation = State();
    State goalLocation = State();
    Cost actionDuration;
    Cost deadCost = actionDuration * 2;
    static std::vector<std::vector<State*>> generatedStates;
};
}
#endif // METRONOME_VEHICLE_HPP
