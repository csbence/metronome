#ifndef METRONOME_PLANNER_HPP
#define METRONOME_PLANNER_HPP

namespace metronome {
class Planner {
public:
    unsigned int getGeneratedNodeCound() {
        return generatedNodeCount;
    }

    unsigned int getExpandedNodeCount() {
        return expandedNodeCount;
    }

protected:
    unsigned int generatedNodeCount{0};
    unsigned int expandedNodeCount{0};
};

}

#endif //METRONOME_PLANNER_HPP
