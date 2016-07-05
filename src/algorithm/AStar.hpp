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
    AStar(Domain domain) : domain(domain), openList(10000000, fValueComparator) {
    }

    std::vector<Action> plan(State startState) {
        std::vector<Action> constructedPlan;

        const Node localStartNode =
                Node(nullptr, std::move(startState), Action(-1), 0, domain.heuristic(startState), true);
        auto startNode = nodePool.construct(localStartNode);

        nodes[localStartNode.state] = startNode;

        openList.push(localStartNode);

        while (!openList.isEmpty()) {
            // TODO increment expanded counter
            Node* currentNode = openList.pop();

            if (!currentNode->open) {
                continue; // This node was disabled
            }

            if (domain.isGoal(currentNode->state)) {
                // Goal is reached
                while (!domain.isStart(currentNode->state)) {
                    constructedPlan.push_back(currentNode->action);
                    currentNode = currentNode->parent;
                }
                return constructedPlan;
            }

            for (auto successor : domain.successors(currentNode->state)) {
                if (successor.state == currentNode->state) {
                    continue; // Skip parent TODO this might be unnecessary
                }

                // TODO increment generated node count

                auto& successorNode = nodes[successor.state];
                auto newCost = successor.actionCost + currentNode->g;

                if (successorNode == nullptr) {
                    // New state discovered
                    const Node tempSuccessorNode(currentNode, successor.state, successor.action, newCost,
                            newCost + domain.heuristic(successor.state), true);

                    successorNode = nodePool.construct(std::move(tempSuccessorNode));
                    openList.push(*successorNode);
                } else if (successorNode->open && successorNode->g > newCost) {
                    // Better path found to an existing state
                    successorNode->g = newCost;
                    openList.update(*successorNode);
                } else {
                    // The new path is not better than the existing
                }
            }
        }

        return std::vector<Action>();
    }

private:
    class Node {
    public:
        Node(Node* parent, State state, Action action, Cost g, Cost f, bool open)
                : parent{parent}, state{state}, action{std::move(action)}, g{g}, f{f}, open{open} {
        }

        unsigned long hash() {
            return state->hash();
        }

        bool operator==(const Node& node) const {
            return state == node.state;
        }

        mutable unsigned int index;
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
        if (lhs.g > rhs.g)
            return -1;
        if (lhs.g < rhs.g)
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
