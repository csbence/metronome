#ifndef METRONOME_FHAT_HPP
#define METRONOME_FHAT_HPP
#include <fcntl.h>
#include <boost/pool/object_pool.hpp>
#include <domains/SuccessorBundle.hpp>
#include <unordered_map>
#include <vector>
#include "MetronomeException.hpp"
#include "OnlinePlanner.hpp"
#include "experiment/Configuration.hpp"
#include "utils/Hasher.hpp"
#include "utils/PriorityQueue.hpp"
#define BOOST_POOL_NO_MT

namespace metronome {

template <typename Domain, typename TerminationChecker>
class FHat final : public OnlinePlanner<Domain, TerminationChecker> {
public:
    typedef typename Domain::State State;
    typedef typename Domain::Action Action;
    typedef typename Domain::Cost Cost;
    typedef typename OnlinePlanner<Domain, TerminationChecker>::ActionBundle ActionBundle;

    FHat(const Domain& domain, const Configuration&) : domain{domain} {
        // Force the object pool to allocate memory
        State state;
        Node node = Node(nullptr, std::move(state), Action(), 0, 0, true, 0, 0, 0);
        nodePool.destroy(nodePool.construct(node));

        // Initialize hash table
        nodes.max_load_factor(1);
        nodes.reserve(100000000);
    }

    std::vector<ActionBundle> selectActions(const State& startState, TerminationChecker& terminationChecker) override {
        if (domain.isGoal(startState)) {
            // Goal is already reached
            return std::vector<ActionBundle>();
        }

        // Learning phase
        if (openList.isNotEmpty()) {
            learn(terminationChecker);
        }

        auto bestNode = explore(startState, terminationChecker);

        return extractPath(bestNode, nodes[startState]);
    }

private:
    class Edge;

    class Node {
    public:
        Node(Node* parent,
                const State& state,
                Action action,
                Cost g,
                Cost h,
                bool open,
                Cost fHat,
                Cost distance,
                Cost distanceError,
                unsigned int iteration = 0)
                : parent{parent},
                  state{state},
                  action{std::move(action)},
                  g{g},
                  h{h},
                  open{open},
                  fHat{fHat},
                  distance{distance},
                  distanceError{distanceError},
                  iteration{iteration} {}

        Cost f() const { return g + h; }

        Cost gHat() const { return g + fHat; }

        unsigned long hash() const { return state->hash(); }

        bool operator==(const Node& node) const { return state == node.state; }

        std::string toString() const {
            std::ostringstream stream;
            stream << "s: " << state << " g: " << g << " h: " << h << " fhat: " << fHat << " a: " << action << " "
                                                                                                               "p: ";
            if (parent == nullptr) {
                stream << "None";
            } else {
                stream << parent->state;
            }
            stream << (open ? " Open" : " Not Open");
            return stream.str();
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

        // FHat

        /** Corrected heuristic cost of the node */
        Cost fHat;
        Cost distance;
        Cost distanceError;

        /** Last iteration when the node was updated */
        unsigned int iteration;
        /** List of all the predecessors that were discovered in the current exploration phase. */
        std::vector<Edge> predecessors;
    };

    class Edge {
    public:
        Edge(Node* predecessor, Action action, Cost actionCost)
                : predecessor{predecessor}, action{action}, actionCost{actionCost} {}

        Node* predecessor;
        const Action action;
        const Cost actionCost;
    };

    void learn(TerminationChecker& terminationChecker) {
        ++iterationCounter;

        // Reorder the open list based on the heuristic values
        openList.reorder(hComparator);

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

                    predecessorNode->distanceError = currentNode->distanceError;
                    predecessorNode->distance = currentNode->distance + 1;

                    addToOpenList(*predecessorNode);
                } else if (predecessorNode->h > currentHeuristicValue + predecessor.actionCost) {
                    // This node was visited in this learning phase, but the current path is better then the previous
                    predecessorNode->h = currentHeuristicValue + predecessor.actionCost;
                    openList.update(*predecessorNode);
                }
            }
        }
    }

    Node* explore(const State& startState, TerminationChecker& terminationChecker) {
        ++iterationCounter;
        clearOpenList();
        openList.reorder(fHatComparator);

        Planner::incrementGeneratedNodeCount();
        Node*& startNode = nodes[startState];

        if (startNode == nullptr) {
            startNode = nodePool.construct(
                    Node{nullptr, startState, Action(), 0, domain.heuristic(startState), true, 0, 0, 0});
        } else {
            startNode->g = 0;
            startNode->action = Action();
            startNode->predecessors.clear();
            startNode->parent = nullptr;
        }

        startNode->iteration = iterationCounter;
        addToOpenList(*startNode);

        Node* currentNode = startNode;

        nextHeuristicError = 0;
        nextDistanceError = 0;

        while (!terminationChecker.reachedTermination() && !domain.isGoal(currentNode->state)) {
            terminationChecker.notifyExpansion();
            currentNode = popOpenList();
            expandNode(currentNode);
        }

        distanceError = nextDistanceError;
        heuristicError = nextHeuristicError;

        return currentNode; // todo this might be one step behind the best
    }

    void expandNode(Node* sourceNode) {
        Planner::incrementExpandedNodeCount();

        Node* bestChildNode{nullptr};

        for (auto successor : domain.successors(sourceNode->state)) {
            auto successorState = successor.state;

            Node*& successorNode = nodes[successorState];

            if (successorNode == nullptr) {
                successorNode = createNode(sourceNode, successor);
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
            if (sourceNode->parent != nullptr && successorState == sourceNode->parent->state) {
                continue;
            }

            // only generate those state that are not visited yet or whose cost value are lower than this path
            Cost successorGValueFromCurrent{sourceNode->g + successor.actionCost};
            if (successorNode->g > successorGValueFromCurrent) {
                successorNode->g = successorGValueFromCurrent;
                successorNode->parent = sourceNode;
                successorNode->action = successor.action;

                double currentDistanceEstimate =
                        successorNode->distanceError / (1.0 - distanceError); // Dionne 2011 (3.8)
                successorNode->fHat = successorNode->g + successorNode->h + heuristicError * currentDistanceEstimate;

                if (!successorNode->open) {
                    addToOpenList(*successorNode);
                } else {
                    openList.update(*successorNode);
                }
            }

            if (bestChildNode == nullptr || bestChildNode->f() > successorNode->f()) {
                bestChildNode = successorNode;
            }
        }

        if (bestChildNode != nullptr) {
            // Local error values (min 0.0)
            double localHeuristicError = bestChildNode->f() - sourceNode->f();
            double localDistanceError = bestChildNode->distance - sourceNode->distance + 1;

            localHeuristicError = (localHeuristicError < 0.0) ? 0.0 : localHeuristicError;
            localDistanceError = (localDistanceError < 0.0) ? 0.0 : localDistanceError;

            // The next error values are the weighted average of the local error and the previous error
            nextHeuristicError += (localHeuristicError - nextHeuristicError) / Planner::getExpandedNodeCount();
            nextDistanceError += (localDistanceError - nextDistanceError) / Planner::getExpandedNodeCount();
        }
    }

    Node* createNode(Node* sourceNode, SuccessorBundle<Domain> successor) {
        Planner::incrementGeneratedNodeCount();

        auto successorState = successor.state;
        auto distance = domain.distance(successorState);
        auto heuristic = domain.heuristic(successorState);

        return nodePool.construct(Node{sourceNode,
                successorState,
                successor.action,
                domain.COST_MAX,
                heuristic,
                true,
                static_cast<Cost>(distance * heuristicError + heuristic),
                distance,
                distance});
    }

    void clearOpenList() {
        openList.forEach([](Node* node) { node->open = false; });
        openList.clear();
    }

    Node* popOpenList() {
        // TODO check if open is empty end throw goal not reachable if yes
        if (openList.isEmpty()) {
            throw MetronomeException("Open list was empty.");
        }

        Node* node = openList.pop();
        node->open = false;
        return node;
    }

    void addToOpenList(Node& node) {
        node.open = true;
        openList.push(node);
    }

    std::vector<ActionBundle> extractPath(const Node* targetNode, const Node* sourceNode) const {
        if (targetNode == sourceNode) {
            //            LOG(INFO) << "We didn't move:" << sourceNode->toString();
            return std::vector<ActionBundle>();
        }

        std::vector<ActionBundle> actionBundles;
        auto currentNode = targetNode;

        while (currentNode != sourceNode) {
            // The g difference of the child and the parent gives the action cost from the parent
            actionBundles.emplace_back(currentNode->action, currentNode->g - currentNode->parent->g);
            currentNode = currentNode->parent;
        }

        std::reverse(actionBundles.begin(), actionBundles.end());
        return actionBundles;
    }

    static int fHatComparator(const Node& lhs, const Node& rhs) {
        if (lhs.fHat < rhs.fHat)
            return -1;
        if (lhs.fHat > rhs.fHat)
            return 1;
        if (lhs.g > rhs.g)
            return -1;
        if (lhs.g < rhs.g)
            return 1;
        return 0;
    }

    static int hComparator(const Node& lhs, const Node& rhs) {
        if (lhs.h < rhs.h)
            return -1;
        if (lhs.h > rhs.h)
            return 1;
        return 0;
    }

    const Domain& domain;
    PriorityQueue<Node> openList{100000000, fHatComparator};
    std::unordered_map<State, Node*, typename metronome::Hasher<State>> nodes{};
    boost::object_pool<Node> nodePool{100000000, 100000000};
    unsigned int iterationCounter{0};

    double heuristicError{0};
    double distanceError{0};

    double nextHeuristicError{0};
    double nextDistanceError{0};
};
}

#endif // METRONOME_FHAT_HPP
