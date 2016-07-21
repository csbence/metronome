#ifndef METRONOME_VEHICLE_HPP
#define METRONOME_VEHICLE_HPP

#include <boost/assert.hpp>
#include <cstdlib>
#include <ctime>
#include <functional>
#include <unordered_map>
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
        Action() : value(0) {
        }
        Action(unsigned int actionDuration) : value(actionDuration) {
        }
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
        State() : x(0), y(0), xVelocity(0), yVelocity(0), index(x + y) {
        }
        State(unsigned int x, unsigned int y) : x(x), y(y) {
            if (randomSeedFlag) {
                std::srand(randomSeed);
            } else {
                std::srand(std::time(0));
            }
            xVelocity = std::rand() % 3;
            yVelocity = std::rand() % 3;

            index = x + y;
        }
        State& operator=(State toCopy) {
            swap(*this, toCopy);
            return *this;
        }

        unsigned int getX() const {
            return x;
        }
        unsigned int getY() const {
            return y;
        }
        int getXVelocity() const {
            return xVelocity;
        }
        int getYVelocity() const {
            return yVelocity;
        }
        std::size_t hash() const {
            return x ^ y << 16 ^ y >> 16;
        }
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
                    obstacles[currentWidth][currentHeight] = true;
                } else {
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
    //    const bool addDyanmicObject(const State& toAdd) {
    //        return this->addObstacle(toAdd);
    //    }
    //
    //    const State transition(const State& state, const Action& action) {
    //        moveObstacles();
    //        return this->GridWorld::transition(state, action);
    //    }
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
    //
    //    std::vector<SuccessorBundle<Vehicle>> successors(State state) {
    //        std::vector<SuccessorBundle<Vehicle>> successors;
    //
    //        unsigned int actions[] = {1, 2, 3, 4, 5};
    //
    //        for (auto a : actions) {
    //            State newState = this->transition(state, Action(a));
    //            for (auto it = obstaclesLocations.begin(); it != obstaclesLocations.end(); ++it) {
    //                if (*it == newState) {
    //                    successors.push_back(SuccessorBundle<Vehicle>{newState, a, this->deadCost});
    //                } else {
    //                    successors.push_back(SuccessorBundle<Vehicle>{newState, a, this->initialCost});
    //                }
    //            }
    //        }
    //
    //        return successors;
    //    }
    //
private:
    static bool randomSeedFlag;
    static long randomSeed;
    unsigned int width;
    unsigned int height;
    bool obstacles;
    bool bunkers;
    State startLocation = State();
    State goalLocation = State();
    Cost actionDuration;
    //    void moveObstacles() {
    //        for (auto it = obstaclesLocations.begin(); it != obstaclesLocations.end(); ++it) {
    //            int cur = 0;
    //            int modX = it->
    //            int modY = obstacleVelocity[cur].second;
    //
    //            if (it->getX() + modX > this->width) {
    //                modX *= -1;
    //            }
    //            if (it->getY() + modY > this->height) {
    //                modY *= -1;
    //            }
    //
    //            auto& testState = bunkerCells[State(it->getX() + modX, it->getY() + modY)];
    //
    //            if (nullptr != testState) {
    //                *it = State(it->getX() + (modX * -1), it->getY() + (modY * -1));
    //            }
    //
    //            *it = State(it->getX() + modX, it->getY() + modY);
    //        }
    //    }
    //
    //    State start;
    //    State goal;
    //    unsigned int height;
    //    unsigned int width;
    //    std::vector<State> obstaclesLocations;
    //    std::unordered_map<State, State*, typename metronome::Hasher<State>> bunkerCells;
    //    Cost initialCost = 1;
    //    Cost deadCost = 1000000;
};
}
#endif // METRONOME_VEHICLE_HPP
