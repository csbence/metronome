#ifndef METRONOME_ASTAR_HPP
#define METRONOME_ASTAR_HPP
#define BOOST_POOL_NO_MT

#include "util/Hasher.hpp"
#include <boost/pool/object_pool.hpp>
#include <unordered_map>
#include <util/PriorityQueue.hpp>
#include <vector>

namespace metronome {

template <typename Domain>
class AStar {
    typedef typename Domain::State State;
    typedef typename Domain::Action Action;
    typedef typename Domain::Cost Cost;

public:
    AStar(Domain domain)
            : domain(domain), openList(10000000, fValueComparator) {
    }

    std::vector<Action> plan(State startState) {

        std::vector<Action> constructedPlan;

        const Node localStartNode = Node(nullptr, std::move(startState),
                Action(), 0, domain.heuristic(startState), true);

        auto startNode = nodePool.construct(localStartNode);

        metronome::Hasher<State> hasher;

        hasher(localStartNode.state);

        nodes[localStartNode.state] = startNode;

        return constructedPlan;
    }

private:
    class Node {
    public:
        Node(Node* parent, State state, Action action, Cost g, Cost f,
                bool open)
                : parent(parent),
                  state(state),
                  action(std::move(action)),
                  g(g),
                  f(f),
                  open(open) {
        }

        unsigned long hash() {
            return state->hash();
        }

        bool operator==(const Node& node) const {
            return state == node.state;
        }

        Node* parent;
        State state;
        Action action;
        Cost g;
        Cost f;
        /** True if the node is in the open list. */
        bool open;
    };

    static int fValueComparator(const Node& lhs, const Node& rhs) {
        if (lhs.f < rhs.f)
            return -1;
        if (lhs.f > rhs.f)
            return 1;
        if (lhs.f > rhs.f)
            return -1;
        if (lhs.f < rhs.f)
            return 1;
        return 0;
    }

    Domain domain;
    PriorityQueue<Node> openList;
    std::unordered_map<State, Node*, typename metronome::Hasher<State>> nodes;
    boost::object_pool<Node> nodePool{100000000, 100000000};
};
}

#endif // METRONOME_ASTAR_HPP
