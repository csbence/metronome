#ifndef METRONOME_PLANNER_HPP
#define METRONOME_PLANNER_HPP

namespace metronome {
class Planner {
public:
    unsigned long long getGeneratedNodeCount() {
        return generatedNodeCount;
    }

    unsigned long long getExpandedNodeCount() {
        return expandedNodeCount;
    }

protected:
    unsigned long long generatedNodeCount{0};
    unsigned long long expandedNodeCount{0};
};
}

#endif // METRONOME_PLANNER_HPP
