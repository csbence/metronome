#ifndef METRONOME_LSSLRTASTAR_HPP
#define METRONOME_LSSLRTASTAR_HPP
#include "Planner.hpp"
#include <boost/pool/object_pool.hpp>
#include <experiment/termination/TimeTerminationChecker.hpp>
#include <fcntl.h>
#include <unordered_map>
#include <utils/Hasher.hpp>
#include <utils/PriorityQueue.hpp>
#include <vector>
#define BOOST_POOL_NO_MT

namespace metronome {

template <typename Domain>
class LssLrtaStar : public Planner {
public:
    typedef typename Domain::State State;
    typedef typename Domain::Action Action;
    typedef typename Domain::Cost Cost;

    LssLrtaStar(const Domain& domain, const Configuration&) : domain{domain} {
    }
    //    LssLrtaStar(const LssLrtaStar&) = default;
    LssLrtaStar(LssLrtaStar&&) = default;

    std::vector<Action> selectActions(const State& startState, TimeTerminationChecker terminationChecker) {
        if (domain.isGoal(startState)) {
            // Goal is already reached
            return std::vector<Action>();
        }

        // Learning phase
        if (openList.isNotEmpty()) {
            learn(terminationChecker);
        }

        const Node localStartNode =
                Node(nullptr, std::move(startState), Action(-1), 0, domain.heuristic(startState), true);

        ++generatedNodeCount;
        auto startNode = nodePool.construct(localStartNode);

        auto bestNode = explore(startNode, terminationChecker);

        return extractPath(bestNode, startNode);
        // TODO return best node and extract plan
    }

private:
    class Edge;

    class Node {
    public:
        Node(Node* parent, const State& state, Action action, Cost g, Cost h, bool open, unsigned int iteration = 0)
                : parent{parent},
                  state{state},
                  action{std::move(action)},
                  g{g},
                  h{h},
                  open{open},
                  iteration{iteration} {
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
        unsigned int iteration;
        /** List of all the predecessors that were discovered in the current exploration phase. */
        std::vector<Edge> predecessors;
    };

    class Edge {
    public:
        Edge(Node* predecessor, Action action, Cost actionCost)
                : predecessor{predecessor}, action{action}, actionCost{actionCost} {
        }

        Node* predecessor;
        const Action action;
        const Cost actionCost;
    };

    void learn(TimeTerminationChecker terminationChecker) {
        ++iterationCounter;

        // Reorder the open list based on the heuristic values
        openList.reorder(hValueComparator);

        while (!terminationChecker.reachedTermination() && openList.isNotEmpty()) {
            auto currentNode = popOpenList();
            currentNode->iteration = iterationCounter;

            Cost currentHeuristicValue = currentNode->h;

            // update heuristic actionDuration of each predecessor
            for (auto predecessor : currentNode->predecessors) {
                Node* predecessorNode = predecessor.predecessor;

                if (predecessorNode->iteration == iterationCounter && !predecessorNode->open) {
                    // This node was already learned and closed in the current iteration
                    continue;
                    // TODO Review this. This could be incorrect if the action costs are not uniform
                }

                if (!predecessorNode->open) {
                    // This node is not open yet, because it was not visited in the current planning iteration

                    predecessorNode->h = currentHeuristicValue + predecessor.actionCost;
                    assert(predecessorNode->iteration == iterationCounter - 1);
                    predecessorNode->iteration = iterationCounter;

                    addToOpenList(*predecessorNode);
                } else if (predecessorNode->h > currentHeuristicValue + predecessor.actionCost) {
                    // This node was visited in this learning phase, but the current path is better then the previous
                    predecessorNode->h = currentHeuristicValue + predecessor.actionCost;
                    openList.update(*predecessorNode);
                }
            }
        }
    }

    Node* explore(Node* startNode, TimeTerminationChecker terminationChecker) {
        ++iterationCounter;
        clearOpenList();
        openList.reorder(fValueComparator);

        Node* currentNode = startNode;

        nodes[startNode->state] = startNode;
        addToOpenList(*startNode);

        while (!terminationChecker.reachedTermination() && !domain.isGoal(currentNode->state)) {
            currentNode = popOpenList();
            expandNode(currentNode);
        }

        return currentNode;
    }

    void expandNode(Node* sourceNode) {
        ++expandedNodeCount;

        auto currentGValue = sourceNode->g;
        for (auto successor : domain.successors(sourceNode->state)) {
            auto successorState = successor.state;

            Node*& successorNode = nodes[successorState];

            if (successorNode == nullptr) {
                ++generatedNodeCount;

                const Node tempSuccessorNode(
                        sourceNode, successor.state, successor.action, successor.actionCost, domain.COST_MAX, true);

                successorNode = nodePool.construct(std::move(tempSuccessorNode));
            }

            // If the node is outdated it should be updated.
            if (successorNode->iteration != iterationCounter) {
                successorNode->iteration = iterationCounter;
                successorNode->predecessors.clear();
                successorNode->g = domain.COST_MAX;
                successorNode->open = false; // It is not on the open list yet, but it will be
                // parent, action, and actionCost is outdated too, but not relevant.
            }

            // Add the current state as the predecessor of the child state
            successorNode->predecessors.emplace_back(sourceNode, successor.action, successor.actionCost);

            // Skip if we got back to the parent
            if (successorState == sourceNode->parent->state) {
                continue;
            }

            // only generate those state that are not visited yet or whose cost actionDuration are lower than this path
            Cost successorGValueFromCurrent{currentGValue + successor.actionCost};
            if (successorNode->g > successorGValueFromCurrent) {
                successorNode->g = successorGValueFromCurrent;
                successorNode->parent = sourceNode;
                successorNode->action = successor.action;

                if (!successorNode->open) {
                    addToOpenList(*successorNode);
                } else {
                    openList.update(*successorNode);
                }
            }
        }
    }

    void clearOpenList() {
        openList.forEach([](Node* node) { node->open = false; });
        openList.clear();
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

    std::vector<Action> extractPath(const Node* targetNode, const Node* sourceNode) const {
        if (targetNode == sourceNode) {
            return std::vector<Action>();
        }

        std::vector<Action> actions{1000};
        auto currentNode = targetNode;

        while (currentNode != sourceNode) {
            actions.push_back(currentNode->action);
            currentNode = currentNode->parent;
        }

        std::reverse(actions.begin(), actions.end());
        return actions;
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

    const Domain& domain;
    PriorityQueue<Node> openList{10000000, fValueComparator};
    std::unordered_map<State, Node*, typename metronome::Hasher<State>> nodes{};
    boost::object_pool<Node> nodePool{100000000, 100000000};
    unsigned int iterationCounter{0};
};
}

#endif // METRONOME_LSSLRTASTAR_HPP
