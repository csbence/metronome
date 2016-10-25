#ifndef METRONOME_EXPANSIONTERMINATIONCHECKER_HPP
#define METRONOME_EXPANSIONTERMINATIONCHECKER_HPP

namespace metronome {

class ExpansionTerminationChecker {
public:
    ExpansionTerminationChecker() {}

    ExpansionTerminationChecker(const ExpansionTerminationChecker&) = delete;

    ExpansionTerminationChecker(ExpansionTerminationChecker&&) = delete;

    void resetTo(unsigned int expansionLimit) {
        this->expansionLimit = expansionLimit;
        expansionCount = 0;
    }

    void setRatio(double ratio) {
        assert(ratio > 0 && ratio <= 1);
        this->ratio = ratio;
    }

    bool reachedTermination() const {
        return expansionCount >= expansionLimit * ratio;
    }

    /**
     * Should be called by the planner at every expansion.
     */
    void notifyExpansion() {
        ++expansionCount;
    }

    unsigned int expansionsPerAction(unsigned long long actionDuration) const {
        return static_cast<unsigned int>(actionDuration);
    }

private:
    unsigned int expansionCount{0};
    unsigned int expansionLimit{0};
    double ratio{1};
};
}

#endif //METRONOME_EXPANSIONTERMINATIONCHECKER_HPP
