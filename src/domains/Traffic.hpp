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
        Obstacle(int x, int y, int xVelocity, int yVelocity)
                : x{x}, y{y}, xVelocity{xVelocity}, yVelocity{yVelocity} {}
        Obstacle() : x{-1}, y{-1}, xVelocity{0}, yVelocity{0} {}
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

    private:
        int x;
        int y;
        int xVelocity;
        int yVelocity;

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
        State() : x{0}, y{0} {}
        State(const unsigned int x, const unsigned int y, std::vector<metronome::Traffic::Obstacle> obstacleMap)
                : x{x}, y{y}, obstacleMap{obstacleMap} {}
        State& operator=(State toCopy) {
            swap(*this, toCopy);
            return *this;
        }

        unsigned int getX() const { return x; }
        unsigned int getY() const { return y; }
        void remapObstacles(std::vector<Obstacle> toMap) { obstacleMap = toMap; }
        std::vector<metronome::Traffic::Obstacle> getObstacleMap() const { return obstacleMap; }
        std::size_t hash() const { return x ^ y << 16 ^ y >> 16; }
        bool operator==(const State& state) const { return x == state.x && y == state.y; }
        const std::string toString() const {
            std::string string("x: ");
            return string + std::to_string(x) + " y: " + std::to_string(y);
        }

        bool operator!=(const State& state) const { return x != state.x || y != state.y; }

    private:
        unsigned int x;
        unsigned int y;

        std::vector<metronome::Traffic::Obstacle> obstacleMap;

        friend void swap(State& first, State& second) {
            using std::swap;
            swap(first.x, second.x);
            swap(first.y, second.y);
            swap(first.obstacleMap, second.obstacleMap);
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

        // actionDuration = configuration.getLong(ACTION_DURATION);
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
//        originalObstacles = std::vector<std::vector<bool>>{width, std::vector<bool>(height)};
//        generatedObstacles = std::vector<std::vector<Obstacle>>{width, std::vector<Obstacle>(height)};

        for (auto i = 0; i < width; ++i) {
            for (auto j = 0; j < height; ++j) {
                obstacles[i][j] = false;
                bunkers[i][j] = false;
//                originalObstacles[i][j] = false;
                //                generatedObstacles[i][j] = Obstacle::createObstacle(-1, -1, 0, 0);
            }
        }

        boost::optional<State> tempStartState;
        boost::optional<State> tempGoalState;

        std::vector<Obstacle> startObstacles{};

        int obstacleIndex = 0;

        while (getline(input, line)) {
            for (auto it = line.cbegin(); it != line.cend(); ++it) {
                // do something for IO
                if (*it == '@') { // find the start location
                    tempStartState = State(currentWidth, currentHeight, std::vector<Obstacle>{});
                } else if (*it == '*') { // find the goal location
                    tempGoalState = State(currentWidth, currentHeight, std::vector<Obstacle>{});
                } else if (*it == '#') { // store the objects
                    //                    addObstacle(currentWidth, currentHeight, obstacleIndex);
                    //                    obstacleIndices.push_back(metronome::Location2D(currentWidth, currentHeight));
                    //                    originalObstacleIndices.push_back(metronome::Location2D(currentWidth,
                    //                    currentHeight));
                    //                    originalObstacles[currentWidth][currentHeight] = true;
                    obstacles[currentWidth][currentHeight] = true;
                    startObstacles.push_back(Obstacle(currentWidth,currentHeight,1,0));
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

        startLocation.remapObstacles(startObstacles);

    }

    void visualize(std::ostream& display, const State& state, const Action& action) const {
        display << "WORLD\n";
        int dx = 0;
        int dy = 0;
        if(action.toChar() == 'N') { dy -= 1;}
        else if(action.toChar() == 'E') {dx += 1; }
        else if(action.toChar() == 'S') { dy += 1; }
        else if(action.toChar() == 'W') { dx -= 1; }

        for (auto i = 0; i < height; ++i) {
            for (auto j = 0; j < width; ++j) {
                if (startLocation.getX() == j+dx && startLocation.getY() == i+dy) {
                    display << '@';
                } else if (goalLocation.getX() == j && goalLocation.getY() == i) {
                    display << '*';
                } else if (isObstacle(state, j, i)) {
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
        for (auto index : state.getObstacleMap()) {
            display << index << "\n";
        }
        display << "\n";
    }
    //
    //    void addObstacle(int x, int y, int index) const {
    //        if (generatedObstacles[x][y].isEmpty()) {
    //            int flip = (std::rand() % 2) + 1;
    //            flip = 0;
    //            int xVelocity = -1;
    //            int yVelocity = 0;
    //            if (flip == 1) {
    //                xVelocity = 0; // std::rand() % MAX_VELOCITY;
    //                yVelocity = 1; // std::rand() % MAX_VELOCITY;
    //            }
    //            Obstacle addObstacle = Obstacle{x, y, xVelocity, yVelocity, index};
    //            generatedObstacles[x][y] = addObstacle;
    //        } else {
    //            int xVelocity = generatedObstacles[x][y].getXVelocity();
    //            int yVelocity = generatedObstacles[x][y].getYVelocity();
    //            Obstacle addObstacle = Obstacle{x, y, xVelocity, yVelocity, index};
    //            generatedObstacles[x][y] = addObstacle;
    //        }
    //    }

    const State transition(const State& state, const Action& action) const {
        std::vector<metronome::Traffic::Obstacle> obstacleMap = moveObstacles(state);
        if (action.toChar() == 'N') {
            State newState = State(state.getX(), state.getY() - 1, obstacleMap);
            if (isLegalLocation(newState)) {
                return newState;
            }
        } else if (action.toChar() == 'E') {
            State newState = State(state.getX() + 1, state.getY(), obstacleMap);
            if (isLegalLocation(newState)) {
                return newState;
            }
        } else if (action.toChar() == 'S') {
            State newState = State(state.getX(), state.getY() + 1, obstacleMap);
            if (isLegalLocation(newState)) {
                return newState;
            }
        } else if (action.toChar() == 'W') {
            State newState = State(state.getX() - 1, state.getY(), obstacleMap);
            if (isLegalLocation(newState)) {
                return newState;
            }
        } else if (action.toChar() == '0' && bunkers[state.getX()][state.getY()]) {
            State newState = State(state.getX(), state.getY(), obstacleMap);
            if (isLegalLocation(newState)) {
                return newState;
            }
        }

        return state;
    }

    const bool isObstacle(const State& state, int x, int y) const {
        for (auto obstacle : state.getObstacleMap()) {
            if(obstacle.getX() == x && obstacle.getY() == y) {
                return true;
            }
        }
        return false;
    }
    const bool isBunker(int x, int y) const { return bunkers[x][y]; }

    const bool isLegalLocation(const State& location) const {
        return location.getX() < width && location.getY() < height &&
                !isObstacle(location, location.getX(), location.getY());
    }
    /*
     * this needs to be fixed....
     */
    std::vector<SuccessorBundle<Traffic>> successors(State state) const {
        // load in the current state of the obstacles
        obstacles.clear();
        for (auto obstacle : state.getObstacleMap()) {
            obstacles[obstacle.getX()][obstacle.getY()] = true;
        }
        // now we are using the correct obstacle locations for the
        // current state we need to expand

        // generate successors as normal
        std::vector<SuccessorBundle<Traffic>> successors;

        unsigned int actions[] = {5, 4, 3, 2, 1};

        for (auto a : actions) {
            State newState = transition(state, Action(a));
            //            std::cout << "newState: " << newState.getX() << "," << newState.getY() << std::endl;
            //            std::cout << obstacles[newState.getX()][newState.getY()] << std::endl;
            //            for (int i = 0; i < width; ++i) {
            //                for (int j = 0; j < height; ++j) {
            //                    std::cout << obstacles[i][j] << "\t";
            //                }
            //                std::cout << std::endl;
            //            }
            if (!obstacles[newState.getX()][newState.getY()]) {
                successors.push_back(SuccessorBundle<Traffic>{newState, a, actionDuration});
            } else {
                //
            }
        }
        //        visualize(std::cout);
        //        std::cout << "successor bundle:\n" << std::endl;
        //        for (auto state : successors) {
        //            std::cout << "\tstate: " << state.state.getX() << "," << state.state.getY() << std::endl;
        //            std::cout << "\taction: " << state.action.toChar() << std::endl;
        //            std::cout << "\tcost: " << state.actionCost << std::endl;
        //        }
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

    const State& getStartState() const { return startLocation; }
    const State& getGoalState() const { return goalLocation; }
    const State& getStartLocation() const { return startLocation; }

    Cost heuristic(const State& state) const { return distance(state) * actionDuration; }

    const bool isGoal(const State& location) const {
        return goalLocation.getX() == location.getX() && goalLocation.getY() == location.getY();
    }

    const bool isStart(const State& location) const {
        return startLocation.getX() == location.getX() && startLocation.getY() == location.getY();
    }

private:
    std::vector<metronome::Traffic::Obstacle> moveObstacles(const State& toMove) const {
        // load the current state of the obstacles
        obstacles.clear();
        for (auto obstacle : toMove.getObstacleMap()) {
            obstacles[obstacle.getX()][obstacle.getY()] = true;
        }
        // calculate where the new obstacles are going
        std::vector<metronome::Traffic::Obstacle> newObstacles;
        for (auto curObstacle : toMove.getObstacleMap()) {
            int xVelocity = curObstacle.getXVelocity();
            int yVelocity = curObstacle.getYVelocity();

            int newXLocation = curObstacle.getX() + xVelocity;
            int newYLocation = curObstacle.getY() + yVelocity;

            if (newXLocation > width - 1) {
                xVelocity *= -1;
                newXLocation = curObstacle.getX(); // + xVelocity;
            }
            if (newYLocation > height - 1) {
                yVelocity *= -1;
                newYLocation = curObstacle.getY(); // + yVelocity;
            }

            if (newXLocation == -1 || newYLocation == -1) {
                throw MetronomeException("Obstacle data structure corrupted...");
            }

            if (newXLocation != curObstacle.getX() || newYLocation != curObstacle.getY()) {
                //                std::cout << "IT IN A NEW LOCAITON" << std::endl;
                if (bunkers[newXLocation][newYLocation] || obstacles[newXLocation][newYLocation]) {
                    xVelocity *= -1;
                    yVelocity *= -1;
                    newXLocation = curObstacle.getX(); // + xVelocity;
                    newYLocation = curObstacle.getY(); // + yVelocity;
                    //                    std::cout << "ITS COLLIDED WITH SOMETHING" << std::endl;

                    // add logic for obstacle precedence when obstacles[newXLocation][newYLocation] is true
                }
            }

            Obstacle newObstacle = Obstacle{newXLocation,newYLocation,xVelocity,yVelocity};
            newObstacles.push_back(newObstacle);
            //            std::cout << newXLocation << "." << newYLocation << ":" << xVelocity << "." << yVelocity <<
            //            std::endl;
            //            if (obstacles[newXLocation][newYLocation]) {
            //                if (newXLocation < width - 1) {
            //                    if (!obstacles[newXLocation + 1][newYLocation]) {
            //                        newXLocation += 1;
            //                    }
            //                } else if (newXLocation > 0) {
            //                    if (!obstacles[newXLocation - 1][newYLocation]) {
            //                        newXLocation -= 1;
            //                    }
            //                } else if (newYLocation < height - 1) {
            //                    if (!obstacles[newXLocation][newYLocation + 1]) {
            //                        newYLocation += 1;
            //                    }
            //                } else if (newYLocation > 0) {
            //                    if (!obstacles[newXLocation][newYLocation - 1]) {
            //                        newYLocation -= 1;
            //                    }
            //                } else {
            //                     can not move this block set velocity to zero?
            //                     or just place it back where it came from
            //                    newXLocation = curObstacle.getX();
            //                    newYLocation = curObstacle.getY();
            //                }
            //            }

            //            if (newXLocation > width - 1) {
            //                xVelocity *= -1;
            //                newXLocation = curObstacle.getX(); // + xVelocity;
            //            }
            //            if (newYLocation > height - 1) {
            //                yVelocity *= -1;
            //                newYLocation = curObstacle.getY(); // + yVelocity;
            //            }

            //            std::cout << "new loc: (" << newXLocation << "," << newYLocation << ")" << std::endl;
            // update the obstacle bit
            // old location off
            //            bool stillMoreObstacles = false;
            //
            //            for(auto a : obstacleIndices) {
            //                int counts = 0;
            //                if(curObstacle.getX() == a.x && curObstacle.getY() == a.y) {
            //                   ++counts;
            //                }
            //                if(counts > 2) {
            //                    stillMoreObstacles = true;
            //                }
            //            }

            //            if (!stillMoreObstacles) {
            //            obstacles[curObstacle.getX()][curObstacle.getY()] = false;
            // new location on
            //            }
            //            std::cout << "?" << std::endl;
            //            obstacles[newXLocation][newYLocation] = true;

            //            std::cout << "?" << std::endl;
            // update the generatedObstacles
            // old location is now default Obstacle object
            //            if(!stillMoreObstacles) {
            //            generatedObstacles[curObstacle.getX()][curObstacle.getY()] = Obstacle{};
            //            }
            //            std::cout << "(X,Y): (" << newXLocation << "," << newYLocation << ") (xV,yV): (" << xVelocity
            //            << ","
            //                      << yVelocity << ")" << std::endl;
            //            generatedObstacles[newXLocation][newYLocation] =
            //                    Obstacle{newXLocation, newYLocation, xVelocity, yVelocity, index};

        }

        return newObstacles;
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
    //    mutable std::vector<metronome::Location2D> obstacleIndices;

    mutable std::vector<std::vector<bool>> obstacles;
    std::vector<std::vector<bool>> bunkers;
    State startLocation{};
    State goalLocation{};
    Cost deadCost;
    //    mutable std::vector<std::vector<Obstacle>> generatedObstacles;
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
