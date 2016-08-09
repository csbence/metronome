#ifndef METRONOME_TRAFFIC_HPP
#define METRONOME_TRAFFIC_HPP

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

class Traffic {
public:
    typedef long long int Cost;
    static constexpr Cost COST_MAX = std::numeric_limits<Cost>::max();
    static const short MAX_VELOCITY = 4;

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
            } else if (value == 5) {
                return '0';
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

    class Obstacle {
    public:
        Obstacle(int x, int y, int xVelocity, int yVelocity, int index)
                : x{x}, y{y}, xVelocity{xVelocity}, yVelocity{yVelocity}, index{index} {}
        Obstacle() : x{-1}, y{-1}, xVelocity{0}, yVelocity{0}, index{0} {}
        //        static Obstacle createObstacle(int x, int y, int xVelocity, int yVelocity) {
        //            return Obstacle(x, y, xVelocity, yVelocity);
        //        }
        Obstacle& operator=(Obstacle toCopy) {
            swap(*this, toCopy);
            return *this;
        }
        int getX() const { return x; }

        int getY() const { return y; }

        std::size_t hash() const { return static_cast<unsigned int>(x ^ y << 16 ^ y >> 16); }

        int getXVelocity() const { return xVelocity; }

        int getYVelocity() const { return yVelocity; }

        bool operator==(const Obstacle& obstacle) const {
            return x == obstacle.x && y == obstacle.y && xVelocity == obstacle.xVelocity &&
                    yVelocity == obstacle.yVelocity;
        }

        bool isEmpty() { return x == -1 && y == -1; }

        friend std::ostream& operator<<(std::ostream& stream, const Traffic::Obstacle& obstacle) {
            stream << "pos: (" << obstacle.getX() << " , " << obstacle.getY() << ")\t|";
            stream << "\tvel: (" << obstacle.getXVelocity() << " , " << obstacle.getYVelocity() << ")";
            return stream;
        }

        int getIndex() { return index; }

    private:
        int x;
        int y;
        int xVelocity;
        int yVelocity;
        int index;

        friend void swap(Obstacle& first, Obstacle& second) {
            using std::swap;
            swap(first.x, second.x);
            swap(first.y, second.y);
            swap(first.xVelocity, second.xVelocity);
            swap(first.yVelocity, second.yVelocity);
        }
    };

    class State {
    public:
        State() : x(0), y(0) {}
        State(const unsigned int x, const unsigned int y) : x(x), y(y) {}
        State& operator=(State toCopy) {
            swap(*this, toCopy);
            return *this;
        }

        unsigned int getX() const { return x; }
        unsigned int getY() const { return y; }
        std::size_t hash() const { return x ^ y << 16 ^ y >> 16; }
        bool operator==(const State& state) const { return x == state.x && y == state.y; }
        const std::string toString() const {
            std::string string("x: ");
            return string + std::to_string(x) + " y: " + std::to_string(y);
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

    Traffic(const Configuration& configuration, std::istream& input)
            : actionDuration(configuration.getLongOrThrow(ACTION_DURATION)) {
        if (randomSeedFlag) {
            std::srand(randomSeed);
        } else {
            std::srand(std::time(0));
        }

        if (!configuration.hasMember(ACTION_DURATION)) {
            throw MetronomeException("No value provided.");
        }

        actionDuration = configuration.getLong(ACTION_DURATION);
        deadCost = actionDuration * 2;
        unsigned int currentHeight = 0;
        unsigned int currentWidth = 0;
        unsigned int currentIndex = 0;
        std::string line;
        char* end;
        getline(input, line);
        std::stringstream convertWidth(line);

        if (std::strtol(line.c_str(), &end, 10) == 0) {
            throw MetronomeException("Traffic first line must be a number.");
        }

        convertWidth >> width;
        getline(input, line);

        if (std::strtol(line.c_str(), &end, 10) == 0) {
            throw MetronomeException("Traffic second line must be a number.");
        }

        std::stringstream convertHeight(line);
        convertHeight >> height;

        obstacles = std::vector<std::vector<bool>>{width, std::vector<bool>(height)};
        bunkers = std::vector<std::vector<bool>>{width, std::vector<bool>(height)};
        generatedObstacles = std::vector<std::vector<Obstacle>>{width, std::vector<Obstacle>(height)};

        for (auto i = 0; i < width; ++i) {
            for (auto j = 0; j < height; ++j) {
                obstacles[i][j] = false;
                bunkers[i][j] = false;
                //                generatedObstacles[i][j] = Obstacle::createObstacle(-1, -1, 0, 0);
            }
        }

        boost::optional<State> tempStartState;
        boost::optional<State> tempGoalState;

        int obstacleIndex = 0;

        while (getline(input, line)) {
            for (auto it = line.cbegin(); it != line.cend(); ++it) {
                // do something for IO
                if (*it == '@') { // find the start location
                    tempStartState = State(currentWidth, currentHeight);
                } else if (*it == '*') { // find the goal location
                    tempGoalState = State(currentWidth, currentHeight);
                } else if (*it == '#') { // store the objects
                    addObstacle(currentWidth, currentHeight, obstacleIndex);
                    obstacleIndices.push_back(metronome::Location2D(currentWidth, currentHeight));
                    obstacles[currentWidth][currentHeight] = true;
                    obstacleIndex++;
                } else if (*it == '$') {
                    bunkers[currentWidth][currentHeight] = true;
                } else {
                    // open cell!
                }
                ++currentWidth;
                ++currentIndex;
            }
            if (currentWidth != width) {
                throw MetronomeException("Traffic is not complete. Width doesn't match the input configuration.");
            }
            currentWidth = 0;
            ++currentHeight;
        }

        if (currentHeight != height) {
            throw MetronomeException("Traffic is not complete. Height doesn't match input configuration.");
        }

        if (!tempStartState.is_initialized() || !tempGoalState.is_initialized()) {
            throw MetronomeException("Traffic unknown start or goal location. Start or goal location is not defined.");
        }

        startLocation = tempStartState.get();
        goalLocation = tempGoalState.get();
    }

    void visualize(std::ostream& display) const {
        display << "WORLD\n";
        for (auto i = 0; i < height; ++i) {
            for (auto j = 0; j < width; ++j) {
                if (startLocation.getX() == j && startLocation.getY() == i) {
                    display << '@';
                } else if (goalLocation.getX() == j && goalLocation.getY() == i) {
                    display << '*';
                } else if (isObstacle(j, i)) {
                    display << '#';
                } else if (isBunker(j, i)) {
                    display << '$';
                } else {
                    display << '_';
                }
            }
            display << "\n";
        }
        display << "\n";
        display << "OBSTACLES:\n";
        for (auto index : obstacleIndices) {
            int x = index.x;
            int y = index.y;
            display << generatedObstacles[x][y] << "\n";
        }
        display << "\n";
    }

    void addObstacle(int x, int y, int index) const {
        if (generatedObstacles[x][y].isEmpty()) {
            int xVelocity = 1; // std::rand() % MAX_VELOCITY;
            int yVelocity = 1; // std::rand() % MAX_VELOCITY;
            Obstacle addObstacle = Obstacle{x, y, xVelocity, yVelocity, index};
            generatedObstacles[x][y] = addObstacle;
        } else {
            int xVelocity = generatedObstacles[x][y].getXVelocity();
            int yVelocity = generatedObstacles[x][y].getYVelocity();
            Obstacle addObstacle = Obstacle{x, y, xVelocity, yVelocity, index};
            generatedObstacles[x][y] = addObstacle;
        }
    }

    const State transition(const State& state, const Action& action) const {
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
        } else if (action.toChar() == '0' && bunkers[state.getX()][state.getY()]) {
            State newState = State(state.getX(), state.getY());
            if (isLegalLocation(newState)) {
                return newState;
            }
        }

        return state;
    }

    const bool isObstacle(int x, int y) const { return obstacles[x][y]; }
    const bool isBunker(int x, int y) const { return bunkers[x][y]; }

    const bool isLegalLocation(const State& location) const {
        return location.getX() < width && location.getY() < height && !isObstacle(location.getX(), location.getY());
    }
    /*
     * this needs to be fixed....
     */
    std::vector<SuccessorBundle<Traffic>> successors(State state) const {
        std::vector<SuccessorBundle<Traffic>> successors;

        unsigned int actions[] = {5, 4, 3, 2, 1};

        for (auto a : actions) {
            State newState = transition(state, Action(a));
            for (auto obstacleIndex : obstacleIndices) {
                if (obstacleIndex.x == newState.getX() && obstacleIndex.y == newState.getY()) {
                    successors.push_back(SuccessorBundle<Traffic>{newState, a, deadCost});
                } else {
                    successors.push_back(SuccessorBundle<Traffic>{newState, a, actionDuration});
                }
            }
        }

        return successors;
    }

    Cost distance(const State& state) const {
        unsigned int verticalDistance =
                std::max(goalLocation.getY(), state.getY()) - std::min(goalLocation.getY(), state.getY());
        unsigned int horizontalDistance =
                std::max(goalLocation.getX(), state.getX()) - std::min(goalLocation.getX(), state.getX());
        unsigned int totalDistance = verticalDistance + horizontalDistance;
        Cost manhattanDistance = static_cast<Cost>(totalDistance);
        return manhattanDistance;
    }

    const State getStartState() const { return startLocation; }
    const State getStartLocation() const { return startLocation; }

    Cost heuristic(const State& state) const { return distance(state) * actionDuration; }

    const bool isGoal(const State& location) const {
        return goalLocation.getX() == location.getX() && goalLocation.getY() == location.getY();
    }

    const bool isStart(const State& location) const {
        return startLocation.getX() == location.getX() && startLocation.getY() == location.getY();
    }

    void testMove() const { moveObstacles(); }

private:
    void moveObstacles() const {
        std::vector<metronome::Location2D> newObstacleIndices;
        for (auto obstacleIndex : obstacleIndices) {
            Obstacle curObstacle = generatedObstacles[obstacleIndex.x][obstacleIndex.y];
            int xVelocity = curObstacle.getXVelocity();
            int yVelocity = curObstacle.getYVelocity();

            int index = curObstacle.getIndex();

            int newXLocation = curObstacle.getX() + xVelocity;
            int newYLocation = curObstacle.getY() + yVelocity;

            // make sure new location is on the grid otherwise bounce
            if (newXLocation - 1 > width) {
                xVelocity *= xVelocity;
                newXLocation = curObstacle.getX() + xVelocity;
            }
            if (newYLocation - 1 > height) {
                yVelocity *= yVelocity;
                newYLocation = curObstacle.getY() + yVelocity;
            }
            // if the new location is a bunker bounce or in another obstacle
            if (bunkers[newXLocation][newYLocation] || obstacles[newXLocation][newYLocation]) {
                xVelocity *= xVelocity;
                yVelocity *= yVelocity;
                newXLocation = curObstacle.getX() + xVelocity;
                newYLocation = curObstacle.getY() + yVelocity;
            }

            std::cout << "new loc: (" << newXLocation << "," << newYLocation << ")" << std::endl;
            // update the obstacle bit
            // old location off
            obstacles[curObstacle.getX()][curObstacle.getY()] = false;
            // new location on
            obstacles[newXLocation][newYLocation] = true;

            // update the generatedObstacles
            // old location is now default Obstacle object
            generatedObstacles[curObstacle.getX()][curObstacle.getY()] = Obstacle{};
            generatedObstacles[newXLocation][newYLocation] =
                    Obstacle{newXLocation, newYLocation, xVelocity, yVelocity, index};
            //            addObstacle(newXLocation, newYLocation);
            // update the obstacleIndices
            newObstacleIndices.push_back(Location2D{newXLocation, newYLocation});
            //            obstacleIndex.x = newXLocation;
            //            obstacleIndex.y = newYLocation;
        }
        obstacleIndices.clear();

        for (auto a : newObstacleIndices) {
            obstacleIndices.push_back(a);
            //            std::cout << a.x << "," <<a.y << std::endl;
        }
        //        for(auto a : obstacleIndices) {
        //            std::cout << a.x << "," << a.y << std::endl;
        //        }
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
     * generatedObstacles <- our cache trick to insure velocity consistency when generating obstacles
     */
    const Cost actionDuration;
    bool randomSeedFlag{true};
    std::time_t randomSeed{1};
    unsigned int width;
    unsigned int height;
    mutable std::vector<metronome::Location2D> obstacleIndices;
    mutable std::vector<std::vector<bool>> obstacles;
    std::vector<std::vector<bool>> bunkers;
    State startLocation{};
    State goalLocation{};
    Cost deadCost;
    mutable std::vector<std::vector<Obstacle>> generatedObstacles;
};

std::ostream& operator<<(std::ostream& stream, const Traffic::Action& action) {
    stream << "action: " << action.toChar();
    return stream;
}

std::ostream& operator<<(std::ostream& stream, const Traffic::State& state) {
    stream << "x: " << state.getX() << " y: " << state.getY();
    return stream;
}
}
#endif // METRONOME_TRAFFIC_HPP
