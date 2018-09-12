#ifndef METRONOME_ONLINEPLANNER_HPP
#define METRONOME_ONLINEPLANNER_HPP

#include "experiment/termination/TimeTerminationChecker.hpp"

#include <fcntl.h>
#include <MemoryConfiguration.hpp>
#include <domains/SuccessorBundle.hpp>
#include <unordered_map>
#include <vector>
#include "MetronomeException.hpp"
#include "OnlinePlanner.hpp"
#include "Planner.hpp"
#include "dynamic_priority_queue.hpp"
#include "easylogging++.h"
#include "experiment/Configuration.hpp"
#include "utils/Hash.hpp"
#include "utils/ObjectPool.hpp"
#include "utils/PriorityQueue.hpp"
#include "utils/TimeMeasurement.hpp"

namespace metronome {

template <typename Domain, typename TerminationChecker>
class ClusterRts final : public OnlinePlanner<Domain, TerminationChecker> {
public:
    typedef typename Domain::State State;
    typedef typename Domain::Action Action;
    typedef typename Domain::Cost Cost;
    typedef typename OnlinePlanner<Domain, TerminationChecker>::ActionBundle ActionBundle;

    static constexpr std::size_t CLUSTER_NODE_LIMIT = 1000;

    ClusterRts(const Domain& domain, const Configuration&) : domain{domain} {
        // Initialize hash table
        nodes.max_load_factor(1);
        nodes.reserve(Memory::NODE_LIMIT);
    }

    std::vector<ActionBundle> selectActions(const State& startState, TerminationChecker& terminationChecker) override {
        if (domain.isGoal(startState)) {
            // Goal is already reached
            return std::vector<ActionBundle>();
        }

        // ---    Expansion    ---
        // Select region to expand
        // Expand region

        // ---      Learn      ---
        // ???

        // --- Path extraction ---
        // Find abstract path to goal
        // 1. Find path to containing region center
        // 2. Find abstract path to goal via region centers
        // 3. Find last segment from region center to target node
        // If the path partially overlaps with the current path stitch

        return extractPath(bestNode, nodes[startState]);
    }

private:
    class Edge;
    class Cluster;

    class Node {
    public:
        Node(Node* parent, const State& state, Action action, Cost g, Cost h, bool open, unsigned int iteration = 0)
                : parent{parent},
                  state{state},
                  action{std::move(action)},
                  g{g},
                  h{h},
                  open{open},
                  iteration{iteration} {}

        Cost f() const { return g + h; }

        unsigned long hash() const { return state.hash(); }

        bool operator==(const Node& node) const { return state == node.state; }

        std::string toString() const {
            std::ostringstream stream;
            stream << "s: " << state << " g: " << g << " h: " << h << " f: " << f() << " a: " << action << " p: ";
            if (parent == nullptr) {
                stream << "None";
            } else {
                stream << parent->state;
            }
            stream << (open ? " Open" : " Not Open");
            return stream.str();
        }

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

        Cluster* containingCluster;
    };

    class NodeComparatorF {
        int operator()(const Node* lhs, const Node* rhs) const {
            if (lhs->f() < rhs->f())
                return -1;
            if (lhs->f() > rhs->f())
                return 1;
            if (lhs->g > rhs->g)
                return -1;
            if (lhs->g < rhs->g)
                return 1;
            return 0;
        }
    };

    class NodeEquals {
        bool operator()(const Node* lhs, const Node* rhs) const { lhs == rhs; }
    };

    class Edge {
    public:
        Edge(Node* predecessor, Action action, Cost actionCost)
                : predecessor{predecessor}, action{action}, actionCost{actionCost} {}

        Node* predecessor;
        const Action action;
        const Cost actionCost;
    };

    class ClusterEdge {
    public:
        Cluster* cluster;
        Node* bestFrontierNode;

        /**
         * Actions leading toward a cluster. Only populated on demand.
         * Should be reset when the best frontier node toward the cluster is changed.
         */
        std::vector<Action> actionsTowardsCluster;
    };

    class Cluster {
    public:
        Node& coreNode;
        std::size_t nodeCount;
        bool completed = false;

        std::vector<ClusterEdge> reachableClusters;
        cserna::DynamicPriorityQueue<Node*,
                cserna::NonIntrusiveIndexFunction<Node*, Hash<Node*>, NodeEquals>,
                NodeComparatorF,
                CLUSTER_NODE_LIMIT,
                CLUSTER_NODE_LIMIT> openList;
    };

    //    void learn(TerminationChecker& terminationChecker) {
    //        ++iterationCounter;
    //
    //        // Reorder the open list based on the heuristic values
    //        openList.reorder(hComparator);
    //
    //        while (!terminationChecker.reachedTermination() && openList.isNotEmpty()) {
    //            auto currentNode = popOpenList();
    //            currentNode->iteration = iterationCounter;
    //
    //            Cost currentHeuristicValue = currentNode->h;
    //
    //            // update heuristic actionDuration of each predecessor
    //            for (auto predecessor : currentNode->predecessors) {
    //                Node* predecessorNode = predecessor.predecessor;
    //
    //                if (predecessorNode->iteration == iterationCounter && !predecessorNode->open) {
    //                    // This node was already learned and closed in the current iteration
    //                    continue;
    //                    // TODO Review this. This could be incorrect if the action costs are not uniform
    //                }
    //
    //                if (!predecessorNode->open) {
    //                    // This node is not open yet, because it was not visited in the current planning iteration
    //
    //                    predecessorNode->h = currentHeuristicValue + predecessor.actionCost;
    //                    assert(predecessorNode->iteration == iterationCounter - 1);
    //                    predecessorNode->iteration = iterationCounter;
    //
    //                    addToOpenList(*predecessorNode);
    //                } else if (predecessorNode->h > currentHeuristicValue + predecessor.actionCost) {
    //                    // This node was visited in this learning phase, but the current path is better then the
    //                    previous predecessorNode->h = currentHeuristicValue + predecessor.actionCost;
    //                    openList.update(*predecessorNode);
    //                }
    //            }
    //        }
    //    }

    void insertStartState() {}

    const Node* explore(const State& startState, TerminationChecker& terminationChecker) {
        ++iterationCounter;
        //        clearOpenList();

        Planner::incrementGeneratedNodeCount();
        Node*& startNode = nodes[startState];

        if (startNode == nullptr) {
            startNode = nodePool.construct(Node{nullptr, startState, Action(), 0, domain.heuristic(startState), true});
        } else {
            startNode->g = 0;
            startNode->action = Action();
            startNode->predecessors.clear();
            startNode->parent = nullptr;
        }

        startNode->iteration = iterationCounter;
        addToOpenList(*startNode);

        while (!terminationChecker.reachedTermination() && openList.isNotEmpty()) {
            Node* const currentNode = popOpenList();

            if (domain.isGoal(currentNode->state)) {
                return currentNode;
            }

            terminationChecker.notifyExpansion();
            expandNode(currentNode);
        }

        return openList.top();
    }

    void expandCluster(Cluster* cluster) {
        if (cluster->completed) {
            return;
        }

        // Check cluster node limit
        if (cluster->nodeCount >= CLUSTER_NODE_LIMIT) {
            cluster->completed = true;
            return;
        }

        if (cluster->openList.isEmpty()) {
            cluster->completed = true;
            return;
        }

        auto currentNode = cluster->openList.pop();

        if (currentNode->containingCluster != nullptr) {
            return;
        }

        currentNode->containingCluster = cluster;
        expandNode(currentNode);
    }

    void expandNode(Node* sourceNode) {
        Planner::incrementExpandedNodeCount();
        auto containingCluster = sourceNode->containingCluster;

        for (auto successor : domain.successors(sourceNode->state)) {
            auto successorState = successor.state;

            Node*& successorNode = nodes[successorState];

            if (successorNode == nullptr) {
                successorNode = createNode(sourceNode, successor);
            }

            // This node is already taken by another cluster
            if (successorNode->containingCluster != nullptr) {
                continue;
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

                sourceNode->containingCluster->openList.insertOrUpdate(successorNode);
            }
        }
    }

    Node* createNode(Node* sourceNode, SuccessorBundle<Domain> successor) {
        Planner::incrementGeneratedNodeCount();
        return nodePool.construct(Node{sourceNode,
                successor.state,
                successor.action,
                domain.COST_MAX,
                domain.heuristic(successor.state),
                true});
    }

    Node* popOpenList() {
        if (openList.isEmpty()) {
            throw MetronomeException("Open list was empty, goal not reachable.");
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
            //                        LOG(INFO) << "We didn't move:" << sourceNode->toString();
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

    static int fComparator(const Node& lhs, const Node& rhs) {
        if (lhs.f() < rhs.f())
            return -1;
        if (lhs.f() > rhs.f())
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
    PriorityQueue<Cluster> openClusters{Memory::OPEN_LIST_SIZE, fComparator};
    std::unordered_map<State, Node*, typename metronome::Hash<State>> nodes{};
    ObjectPool<Node, Memory::NODE_LIMIT> nodePool;
    ObjectPool<Cluster, Memory::NODE_LIMIT> clusterPool;

    unsigned int iterationCounter{0};
};
} // namespace metronome

#endif // METRONOME_ONLINEPLANNER_HPP