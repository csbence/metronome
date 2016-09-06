#ifndef METRONOME_EXPANSIONTERMINATIONCHECKER_HPP
#define METRONOME_EXPANSIONTERMINATIONCHECKER_HPP

namespace metronome {

class ExpansionTerminationChecker {
public:
    void resetTo(unsigned int expansionLimit) {
        this->expansionLimit = expansionLimit;
        expansionCount = 0;
    }

    bool reachedTermination() const {
        return expansionCount >= expansionLimit;
    }

    /**
     * Should be called by the planner at every expansion.
     */
    void notifyExpansion() {
        ++expansionCount;
    }

    unsigned int expansionsPerAction(unsigned long long actionDuration) {
        return static_cast<unsigned int>(actionDuration);
    }

private:
    unsigned int expansionCount{0};
    unsigned int expansionLimit{0};
};
}

#endif //METRONOME_EXPANSIONTERMINATIONCHECKER_HPP
