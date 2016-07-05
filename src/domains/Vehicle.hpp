#ifndef METRONOME_VEHICLE_HPP
#define METRONOME_VEHICLE_HPP

#include "SuccessorBundle.hpp"
#include "VacuumWorld.hpp"
#include <boost/assert.hpp>
#include <cstdlib>
#include <ctime>
#include <functional>
#include <vector>

class Vehicle : VacuumWorld {
public:
    Vehicle(State start = State::newState(0, 0, 0), State goal = State::newState(4, 4, 0), unsigned int width = 5,
            unsigned int height = 5, std::vector<State> objectStates = std::vector<State>{},
            std::vector<State> safeStates = std::vector<State>{})
            : width(width),
              height(height),
              obstaclesLocations(objectStates),
              start(start),
              goal(goal),
              safeCells(safeStates) {
    }
    const bool addDyanmicObject(const State& toAdd) {
        return this->addBlockedCell(toAdd);
    }

    const State transition(const State& state, const Action& action) {
        moveObstacles();
        return this->VacuumWorld::transition(state, action);
    }

private:
    void moveObstacles() {
        for (auto it = obstaclesLocations.begin(); it != obstaclesLocations.end(); ++it) {
            std::srand(std::time(0));
            int randomInteger = std::rand() % 1;
            std::srand(std::time(0));
            int otherRandomInteger = std::rand() % 1;
            std::srand(std::time(0));
            int coinFlip = std::rand() % 100;

            if (coinFlip > 50) {
                randomInteger *= -1;
            } else {
                otherRandomInteger *= -1;
            }
            *it = State::newState(it->getX() + randomInteger, it->getY() + otherRandomInteger);
        }
    }

    State start;
    State goal;
    unsigned int height;
    unsigned int width;
    std::vector<State> obstaclesLocations;
    std::vector<State> safeCells;
};

#endif // METRONOME_VEHICLE_HPP
