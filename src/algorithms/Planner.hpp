#ifndef METRONOME_PLANNER_HPP
#define METRONOME_PLANNER_HPP

namespace metronome {
class Planner {
public:
    virtual unsigned long long getGeneratedNodeCount() const final {
        return generatedNodeCount;
    }

    virtual unsigned long long getExpandedNodeCount() const final {
        return expandedNodeCount;
    }

    virtual ~Planner() = default;

protected:
    virtual inline void incrementGeneratedNodeCount() final {
        ++generatedNodeCount;
    }

    virtual inline void incrementExpandedNodeCount() final {
        ++expandedNodeCount;
    }

private:
    unsigned long long generatedNodeCount{0};
    unsigned long long expandedNodeCount{0};
};
}

#endif // METRONOME_PLANNER_HPP
