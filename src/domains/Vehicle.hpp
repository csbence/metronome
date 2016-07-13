#ifndef METRONOME_VEHICLE_HPP
#define METRONOME_VEHICLE_HPP

#include "GridWorld.hpp"
#include "SuccessorBundle.hpp"
#include <bits/unordered_map.h>
#include <boost/assert.hpp>
#include <cstdlib>
#include <ctime>
#include <functional>
#include <vector>

class Vehicle : GridWorld {
public:
    typedef GridWorld::State State;
    typedef GridWorld::Action Action;
    typedef GridWorld::Cost Cost;
    Vehicle(State start = State::newState(0, 0, 0), State goal = State::newState(4, 4, 0), unsigned int width = 5,
            unsigned int height = 5, std::vector<State> objectStates = std::vector<State>{},
            std::unordered_map<State, State*, typename metronome::Hasher<State>> safeStates =
                    std::unordered_map<State, State*, typename metronome::Hasher<State>>{},
            std::vector<std::pair<int, int>> velocities = std::vector<std::pair<int, int>>{})
            : width(width),
              height(height),
              obstaclesLocations(objectStates),
              start(start),
              goal(goal),
              bunkerCells(safeStates),
              obstacleVelocity(velocities) {
    }
    const bool addDyanmicObject(const State& toAdd) {
        return this->addBlockedCell(toAdd);
    }

    const State transition(const State& state, const Action& action) {
        moveObstacles();
        return this->GridWorld::transition(state, action);
    }

    const State getStartLocation() {
        return GridWorld::getStartLocation();
    }

    Cost heuristic(const State& state) {
        return GridWorld::heuristic(state);
    }

    const bool isGoal(const State& location) {
        return GridWorld::isGoal(location);
    }

    const bool isStart(const State& state) {
        return GridWorld::isStart(state);
    }

    std::vector<SuccessorBundle<Vehicle>> successors(State state) {
        std::vector<SuccessorBundle<Vehicle>> successors;

        unsigned int actions[] = {1, 2, 3, 4, 5};

        for(auto a : actions) {
            State newState = this->transition(state, Action(a));
            for(auto it = obstaclesLocations.begin(); it != obstaclesLocations.end(); ++it) {
                if(*it == newState){
                    successors.push_back(SuccessorBundle<Vehicle>{newState, a, this->deadCost});
                }
                else {
                    successors.push_back(SuccessorBundle<Vehicle>{newState, a, this->initialCost});
                }
            }

        }

        return successors;
    }

private:
    void moveObstacles() {
        for (auto it = obstaclesLocations.begin(); it != obstaclesLocations.end(); ++it) {
            int cur = 0;
            int modX = obstacleVelocity[cur].first;
            int modY = obstacleVelocity[cur].second;

            if (it->getX() + modX > this->width) {
                modX *= -1;
            }
            if (it->getY() + modY > this->height) {
                modY *= -1;
            }

            auto& testState = bunkerCells[State::newState(it->getX() + modX, it->getY() + modY)];

            if (nullptr != testState) {
                *it = State::newState(it->getX() + (modX*-1), it->getY() + (modY*-1));
            }

            *it = State::newState(it->getX() + modX, it->getY() + modY);
        }
    }

    State start;
    State goal;
    unsigned int height;
    unsigned int width;
    std::vector<State> obstaclesLocations;
    std::unordered_map<State, State*, typename metronome::Hasher<State>> bunkerCells;
    std::vector<std::pair<int, int>> obstacleVelocity;
    Cost initialCost = 1;
    Cost deadCost = 1000000;
};

#endif // METRONOME_VEHICLE_HPP
