#ifndef METRONOME_LSSLRTASTAR_HPP
#define METRONOME_LSSLRTASTAR_HPP
#include <boost/pool/object_pool.hpp>
#include <unordered_map>
#include <util/Hasher.hpp>
#include <util/PriorityQueue.hpp>
#include <vector>
#define BOOST_POOL_NO_MT

namespace metronome {

template <typename Domain>
class LssLrtaStar {
    typedef typename Domain::State State;
    typedef typename Domain::Action Action;
    typedef typename Domain::Cost Cost;

    LssLrtaStar(Domain domain) : domain{domain} {
    }

    std::vector<Action> selectAction(const State& startState) { // TODO add termination checker

        if (domain.isGoal(startState)) {
            // Goal is already reached
            return std::vector<Action>();
        }

        // Learning phase
        if (openList.isNotEmpty()) {
            learn(); // TODO add termination checker
        }

        explore(startState); // TODO add termination checker

        // TODO return best node and extract plan
    }

private:
    class Edge {
    public:
        Edge(Node* predecessor, Action action, Cost actionCost)
                : predecessor{predecessor}, action{action}, actionCost{actionCost} {
        }

        const Node* predecessor;
        const Action action;
        const Cost actionCost;
    };

    class Node {
    public:
        Node(Node* parent, const State& state, Action action, Cost g, Cost h, bool open)
                : parent{parent}, state{state}, action{std::move(action)}, g{g}, h{h}, open{open} {
        }

        unsigned long hash() {
            return state->hash();
        }

        bool operator==(const Node& node) const {
            return state == node.state;
        }

        /** Index used by the priority queue */
        mutable unsigned int index;
        /** Parent node */
        Node* parent;
        /** Internal state */
        const State state;
        /** Action that led to the current node from the parent node */
        Action action;
        /** Cost from the root node */
        Cost g;
        /** Heuristic cost of the node */
        Cost h;
        /** True if the node is in the open list */
        bool open;
        /** Last iteration when the node was updated */
        unsigned int iteration{0};
        /** List of all the predecessors that were discovered in the current exploration phase. */
        std::vector<Edge> predecessors;
    };

    void learn() {
        //TODO
    }

    void explore(const State& startState) {
        ++iterationCounter;
        clearOpenList();
        openList.reorder(fValueComparator);

        const Node localStartNode =
            Node(nullptr, std::move(startState), Action(-1), 0, domain.heuristic(startState), true);

        auto startNode = nodePool.construct(localStartNode);

        nodes[startNode->state] = startNode;

        // TODO finish implementation
    }

    void clearOpenList() {
        // TODO clear open and set the open value of the cleared nodes to false
    }

    Node* popOpenList() {
        // TODO check if open is empty end throw goal not reachable if yes
        Node* node = openList.pop();
        node->open = false;
        return node;
    }

    void addToOpenList(Node& node) {
        node.open = true;
        openList.push(node);
    }

    static int fValueComparator(const Node& lhs, const Node& rhs) {
        Cost lhsF = lhs.g + lhs.h;
        Cost rhsF = rhs.g + rhs.h;

        if (lhsF < rhsF)
            return -1;
        if (lhsF > rhsF)
            return 1;
        if (lhs.g > rhs.g)
            return -1;
        if (lhs.g < rhs.g)
            return 1;
        return 0;
    }

    static int hValueComparator(const Node& lhs, const Node& rhs) {
        if (lhs.h < rhs.h)
            return -1;
        if (lhs.h > rhs.h)
            return 1;
        return 0;
    }

    Domain domain;
    PriorityQueue<Node> openList{10000000, fValueComparator};
    std::unordered_map<State, Node*, typename metronome::Hasher<State>> nodes;
    boost::object_pool<Node> nodePool{100000000, 100000000};
    unsigned int iterationCounter{0};
};
}

#endif // METRONOME_LSSLRTASTAR_HPP
