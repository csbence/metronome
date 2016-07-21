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

    private:
        unsigned int x;
        unsigned int y;
        int xVelocity;
        int yVelocity;

        friend void swap(State& first, State& second) {
            using std::swap;
            swap(first.x, second,x);
            swap(first.y, second.y);
            swap(first.xVelocity, second.xVelocity);
            swap(first.yVelocity, second.yVelocity);
        }
    };

    Vehicle(const Configuration& config, std::istream& input) {
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
    // private:
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
