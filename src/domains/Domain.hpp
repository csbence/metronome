#ifndef METRONOME_DOMAIN_HPP
#define METRONOME_DOMAIN_HPP

#include <experiment/Configuration.hpp>
#include <limits>
#include <string>
#include <vector>
#include <boost/optional.hpp>
#include "SuccessorBundle.hpp"

namespace metronome {

class Domain {
public:
    typedef long long int Cost;
    static constexpr Cost COST_MAX = std::numeric_limits<Cost>::max();

    class Action {
    public:
        static Action getIdentity() {
            // TODO
        }

        bool operator==(const Action& action) const {
            // TODO
        }
        
        bool operator!=(const Action& rhs) const { return !(rhs == *this); }

        std::string toString() const {
            // TODO
        }
        
        friend std::ostream& operator<<(std::ostream& stream, const Action& action) {
            return stream; // TODO
        }
    };

    class State {
    public:
        bool operator==(const State& state) const {
            // TODO
        }
        
        bool operator!=(const State& rhs) const { return !(rhs == *this); }

        std::size_t hash() const {
            // TODO
        }

        std::string toString() const {
            // TODO
        }

        friend std::ostream& operator<<(std::ostream& stream, const State& state) {
            return stream; // TODO
        }
    };

    Domain(const Configuration& configuration, std::istream& input) {
        // TODO
    }

    boost::optional<State> transition(const State& state, const Action& action) const {
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

    std::vector<SuccessorBundle<Domain>> successors(const State& state) const {
        // TODO
    }

    const State getStartState() const {
        // TODO
    }

    Cost getActionDuration() const {
        //TODO
    }

    Action getIdentityAction() const {
        // TODO
    }

    bool safetyPredicate(const State& state) const {
        // TODO
    }
};
}
#endif // METRONOME_DOMAIN_HPP
