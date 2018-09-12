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
    using State = typename Domain::State;
    using Action = typename Domain::Action;
    using Cost = typename Domain::Cost;
    using ActionBundle = typename OnlinePlanner<Domain, TerminationChecker>::ActionBundle;

    static constexpr std::size_t CLUSTER_NODE_LIMIT = 1000;
    static constexpr std::size_t CLUSTER_G_RADIUS = 10;
    static constexpr std::size_t MAX_CLUSTER_COUNT = 1000;

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
        Node(Node* parent, const State& state, Action action, Cost g, Cost h, unsigned int iteration = 0)
                : parent{parent}, state{state}, action{std::move(action)}, g{g}, h{h}, iteration{iteration} {}

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
        Node* coreNode;
        std::size_t nodeCount = 0;
        bool completed = false;

        Node* bestHNode;

        std::vector<ClusterEdge> reachableClusters;
        cserna::DynamicPriorityQueue<Node*,
                cserna::NonIntrusiveIndexFunction<Node*, Hash<Node*>, NodeEquals>,
                NodeComparatorF,
                CLUSTER_NODE_LIMIT,
                CLUSTER_NODE_LIMIT>
                openList;

        std::size_t openListIndex = std::numeric_limits<std::size_t>::max();
    };

    struct ClusterIndex {
        std::size_t& operator()(const Cluster* cluster) const { return cluster->openListIndex; }
    };

    struct ClusterHash {
        std::size_t operator()(const Cluster* cluster) const { return cluster->coreNode->hash(); }
    };

    struct ClusterEquals {
        bool operator()(const Cluster* lhs, const Cluster* rhs) const { return lhs == rhs; }
    };

    class ClusterComparatorH {
        int operator()(const Cluster* lhs, const Cluster* rhs) const {
            if (lhs->bestHNode->h < rhs->bestHNode->h)
                return -1;
            if (lhs->bestHNode->h > rhs->bestHNode->h)
                return 1;
            if (lhs->bestHNode->g < rhs->bestHNode->g)
                return -1;
            if (lhs->bestHNode->g > rhs->bestHNode->g)
                return 1;
            return 0;
        }
    };

    class ClusterComparatorCoreH {
        int operator()(const Cluster* lhs, const Cluster* rhs) const {
            if (lhs->coreNode->h < rhs->coreNode->h)
                return -1;
            if (lhs->coreNode->h > rhs->coreNode->h)
                return 1;
            return 0;
        }
    };

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

    void expandCluster(Cluster* sourceCluster) {
        if (sourceCluster->completed) {
            return;
        }

        if (sourceCluster->openList.isEmpty()) {
            sourceCluster->completed = true;
            return;
        }

        // A new cluster core should be spawned in the following cases:
        // * Cluster node limit is reached
        // * Node is beyond a threshold cost (g) from the cluster core
        // * Node is beyond a threshold distance (d) from the cluster core -> todo implement d

        auto currentNode = sourceCluster->openList.pop();

        if (currentNode->containingCluster != nullptr) {
            return;
        }

        bool spawnNewCore = sourceCluster->nodeCount >= CLUSTER_NODE_LIMIT || currentNode->g >= CLUSTER_G_RADIUS;

        if (spawnNewCore) {
            auto spawnedCluster = clusterPool.construct();
            spawnedCluster->coreNode = currentNode;
            openClusters.push(spawnedCluster);

            currentNode->containingCluster = spawnedCluster;
        } else {
            currentNode->containingCluster = sourceCluster;
        }

        ++(currentNode->containingCluster->nodeCount);
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
    cserna::DynamicPriorityQueue<Cluster*, ClusterIndex, ClusterComparatorCoreH, MAX_CLUSTER_COUNT, MAX_CLUSTER_COUNT>
            openClusters;

    std::unordered_map<State, Node*, typename metronome::Hash<State>> nodes{};
    ObjectPool<Node, Memory::NODE_LIMIT> nodePool;
    ObjectPool<Cluster, Memory::NODE_LIMIT> clusterPool;

    unsigned int iterationCounter{0};
};
} // namespace metronome

#endif // METRONOME_ONLINEPLANNER_HPP
