#ifndef METRONOME_SLIDINGTILEPUZZLE_HPP
#define METRONOME_SLIDINGTILEPUZZLE_HPP

namespace metronome {

class SlidingTilePuzzle {
    typedef long long int Cost;
    static constexpr Cost COST_MAX = std::numeric_limits<Cost>::max();

    class Action {

    };

    class State {

    };

    class Action {
        std::string toString() const {
            // TODO
        }
    };
    class State {
        bool operator==(const State& state) const {
            // TODO
        }

        std::size_t hash() const {
            // TODO
        }

        std::string toString() const {
            // TODO
        }
    };

    Domain(const Configuration& configuration, std::istream& input) {
        // TODO
    }

    const State transition(const State& state, const Action& action) const {
        // TODO
    }

    bool isGoal(const State& location) const {
        // TODO
    }

    Cost distance(const State& state) const {
        // TODO
    }

    Cost heuristic(const State& state) const {
        // TODO
    }

    std::vector<SuccessorBundle<Domain>> successors(State state) const {
        // TODO
    }

    const State getStartState() const {
        // TODO
    }


};

}


#endif //METRONOME_SLIDINGTILEPUZZLE_HPP
