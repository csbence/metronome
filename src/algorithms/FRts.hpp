#ifndef METRONOME_FRTS_H
#define METRONOME_FRTS_H

#include <fcntl.h>
#include <MemoryConfiguration.hpp>
#include <algorithm>
#include <boost/pool/object_pool.hpp>
#include <unordered_map>
#include <utils/LinearMemoryPool.hpp>
#include <vector>
#include "MetronomeException.hpp"
#include "OnlinePlanner.hpp"
#include "domains/SuccessorBundle.hpp"
#include "easylogging++.h"
#include "experiment/Configuration.hpp"
#include "utils/Hasher.hpp"
#include "utils/PriorityQueue.hpp"
#include "utils/Statistic.hpp"
#include "utils/TimeMeasurement.hpp"

namespace metronome {

template <typename Domain, typename TerminationChecker>
class FRts final : public OnlinePlanner<Domain, TerminationChecker> {
public:
    typedef typename Domain::State State;
    typedef typename Domain::Action Action;
    typedef typename Domain::Cost Cost;
    typedef typename OnlinePlanner<Domain, TerminationChecker>::ActionBundle ActionBundle;

    FRts(const Domain& domain, const Configuration&) : domain{domain} {
        // Initialize hash table
        nodes.max_load_factor(1);
        nodes.reserve(Memory::NODE_LIMIT);
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

        terminationChecker.setRatio(0.5);

        const Node* bestNode{explore(startState, terminationChecker)};
        std::vector<ActionBundle> bestPath{extractPath(bestNode, nodes[startState])};

        // Find potential nodes
        const auto potentialHeadNodes = findHeadNodes();
        double availableTimePerNode = 0.5 / potentialHeadNodes.size();
        double currentRatio = 0.5;

        LOG(INFO) << "Labels";
        // Decide what to do
        for (auto node : potentialHeadNodes) {
            LOG(INFO) << node->actionLabel;

            currentRatio += availableTimePerNode;
            terminationChecker.setRatio(currentRatio);

            // Addition lookahead search on the head nodes
            const Node* focusedHead = explore(node, terminationChecker, focusedOpenList);

            if (focusedHead != nullptr && bestNode->fHat > focusedHead->fHat) {
                bestNode = focusedHead;
                bestPath = extractPath(bestNode, nodes[startState]);
            }
        }

        return bestPath;
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

        Node(const Node&) = default;

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
        mutable unsigned int index{0};
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

        // MoRTS

        /** Corrected heuristic cost of the node */
        Cost fHat;
        Cost distance;
        Cost distanceError;

        /** Expansion counter when the node was generated. */
        unsigned int generation{0};
        /** Expansion counter when the node was expanded. */
        unsigned int expansion{0};
        /** Last iteration when the node was updated */
        unsigned int iteration;
        /** Depth in the search tree relative to the current source */
        unsigned int depth{0};
        /** Top level action label */
        Action actionLabel{};

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
            auto currentNode = popOpenList(openList);
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
//                    assert(predecessorNode->iteration == iterationCounter - 1);
//                    predecessorNode->iteration = iterationCounter;

                    predecessorNode->distanceError = currentNode->distanceError;
                    predecessorNode->distance = currentNode->distance + 1;

                    addToOpenList(openList, *predecessorNode);
                } else if (predecessorNode->h > currentHeuristicValue + predecessor.actionCost) {
                    // This node was visited in this learning phase, but the current path is better then the previous
                    predecessorNode->h = currentHeuristicValue + predecessor.actionCost;
                    openList.update(*predecessorNode);
                }
            }
        }
    }

    const Node* explore(Node* startNode, TerminationChecker& terminationChecker, PriorityQueue<Node>& openList) {
        clearOpenList(openList); // TODO fix for learning

        // Do not insert the first node to the open list
       if (!terminationChecker.reachedTermination()) {
           if (domain.isGoal(startNode->state)) {
               return startNode;
           }

           terminationChecker.notifyExpansion();
           expandNode(startNode, openList);
       }


        while (!terminationChecker.reachedTermination() && openList.isNotEmpty()) {
            Node* const currentNode = popOpenList(openList);

            if (domain.isGoal(currentNode->state)) {
                return currentNode;
            }

            terminationChecker.notifyExpansion();
            expandNode(currentNode, openList);
        }

        return openList.top();
    }

    const Node* explore(const State& startState, TerminationChecker& terminationChecker) {
        // Skip initialization if an identity action was applied
        if (!identityIndicator) {
            ++iterationCounter;
            clearOpenList(openList);
            openList.reorder(fHatComparator);

            Planner::incrementGeneratedNodeCount();
            Node*& startNode = nodes[startState];

            if (startNode == nullptr) {
                startNode = nodePool->construct(
                        nullptr, startState, Action(), 0, domain.heuristic(startState), true, 0, 0, 0);
            } else {
                startNode->g = 0;
                startNode->action = Action();
                startNode->predecessors.clear();
                startNode->parent = nullptr;
            }

            startNode->depth = 0; // Depth relative to the LSS search tree

            startNode->iteration = iterationCounter;
            addToOpenList(openList, *startNode);
        }

        bool expanded = false;
        while (!terminationChecker.reachedTermination() && openList.isNotEmpty()) {
            expanded = true;
            Node* const currentNode = popOpenList(openList);

            if (domain.isGoal(currentNode->state)) {
                return currentNode;
            }

            terminationChecker.notifyExpansion();
            expandNode(currentNode, openList);
        }

        // Normal action - discovery failed - top is current
        // Identity - discovery should succeed.

        assert(!identityIndicator || expanded);

        if (!expanded) {
            LOG(INFO) << "Take emergency step.";
            return emergencyStep(popOpenList(openList));
        }

        return openList.top();
    }

    /**
     * Find the best successor node. Learning is ommitted.
     *
     * @param sourceState
     * @return best successor node.
     */
    Node* emergencyStep(Node* sourceNode) {
        Node* bestChildNode{nullptr};

        for (auto successor : domain.successors(sourceNode->state)) {
            auto successorState = successor.state;

            Node*& successorNode = nodes[successorState];

            if (successorNode == nullptr) {
                successorNode = createNode(sourceNode, successor);
            }

            //            successorNode->g = sourceNode->g + successor.actionCost;
            //            successorNode->parent = sourceNode;
            //            successorNode->action = successor.action;
            //            successorNode->depth = sourceNode->depth + 1;
            //
            //            double currentDistanceEstimate =
            //                successorNode->distanceError / (1.0 - distanceError); // Dionne 2011 (3.8)
            //            successorNode->fHat = successorNode->g + successorNode->h + heuristicError *
            //            currentDistanceEstimate;

            if (bestChildNode == nullptr || bestChildNode->f() > successorNode->f()) {
                bestChildNode = successorNode;
            }
        }

        return bestChildNode;
    }

    void expandNode(Node* sourceNode, PriorityQueue<Node>& openList) {
        Planner::incrementExpandedNodeCount();

        sourceNode->expansion = ++expansionCounter;

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
                successorNode->g = Domain::COST_MAX;
                successorNode->open = false; // It is not on the open list yet, but it will be
                successorNode->generation = expansionCounter;
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
                successorNode->depth = sourceNode->depth + 1;

                double currentDistanceEstimate =
                        successorNode->distanceError / (1.0 - distanceError); // Dionne 2011 (3.8)
                successorNode->fHat = successorNode->g + successorNode->h + heuristicError * currentDistanceEstimate;

                // Labeling
                if (sourceNode->depth == 0) {
                    // This is a top level action
                    successorNode->actionLabel = successor.action;
                } else {
                    // Inherit the top level action label
                    successorNode->actionLabel = sourceNode->actionLabel;
                }

                // Open list management
                if (!successorNode->open) {
                    addToOpenList(openList, *successorNode);
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

    std::vector<Node*> findHeadNodes() {
        std::vector<Node*> nodes;

        Node* const topNode = openList.top();
        nodes.push_back(topNode);

        const Cost topCost = topNode->g;
        const Cost topHHat = topNode->fHat;

        openList.forEach([&](Node* node) {
            Node* matchingNode{nullptr};

            // Find the matching node with the same top level action label
            for (Node* bestNode : nodes) {
                if (bestNode->actionLabel == node->actionLabel) {
                    matchingNode = bestNode;
                    break;
                }
            }

            if (matchingNode == nullptr) {
                // Add new head node
                nodes.push_back(node);
            } else if (matchingNode->fHat > node->fHat) {
                // Replace existing head with a better node
                // TODO this could be optimized
                std::replace(nodes.begin(), nodes.end(), matchingNode, node);
            }

        });

        return nodes;
    }

    std::vector<Node*> findTopTwoHeadNodes() {
        // Find best two actions
        const auto alphaTargetNode = openList.top();
        const Action alphaAction = alphaTargetNode->actionLabel;

        Node* betaTargetNode{nullptr};
        openList.forEach([&](Node* node) {
            if (node->actionLabel != alphaAction && (betaTargetNode == nullptr || betaTargetNode->fHat > node->fHat)) {
                betaTargetNode = node;
            }
        });

        std::vector<Node*> nodes({alphaTargetNode});
        if (betaTargetNode != nullptr) {
            nodes.push_back(betaTargetNode);
        }

        return nodes;
    }

    bool isBenefitialToSearch(const Node* sourceNode, const TerminationChecker& terminationChecker) {
        if (openList.getSize() <= 1) {
            return false; // No alternative actions
        }

        // Find best two actions
        const auto alphaTargetNode = openList.top();
        const Action alphaAction = alphaTargetNode->actionLabel;

        Node* betaTargetNode{nullptr};
        openList.forEach([&](Node* node) {
            if (node->actionLabel != alphaAction && (betaTargetNode == nullptr || betaTargetNode->fHat > node->fHat)) {
                betaTargetNode = node;
            }
        });

        if (betaTargetNode == nullptr) {
            // There are no alternative actions to take
            LOG(INFO) << "No alternative actions are available";
            return false;
        }

        // Calculate the expansion advance
        unsigned int expectedExpansions = terminationChecker.expansionsPerAction(domain.getActionDuration());

        // Calculate the benefit
        double benefit = computeBenefit(sourceNode, alphaTargetNode, betaTargetNode, expectedExpansions);

        Cost actionCost = domain.getActionDuration(); // Reconsider for identity actions

        //        if (benefit > actionCost) {
        //        LOG(INFO) << "Benefit:  " << benefit << " Cost " << actionCost;
        //        }

        return benefit > actionCost;
    }

    double expansionDelay(const Node* source, const Node* target) const {
        long delay{1 + expansionCounter - target->generation};
        int count = 1;

        for (Node* current = target->parent; current != nullptr && current->generation > 0 && target != source;
                current = current->parent) {
            ++count;
            delay += current->expansion - current->generation;
        }

        return delay / static_cast<double>(count); // Normalize (calculate average)
    }

    // Returns the utility of searching at a given state
    double computeBenefit(const Node* source,
            const Node* alpha,
            const Node* beta,
            unsigned int expansionAdvance) const {
        // Calculate delay for alpha and beta path
        double alphaDelay{expansionDelay(source, alpha)};
        double betaDelay{expansionDelay(source, beta)};

        assert(alpha->fHat <= beta->fHat);

        // Variance belief (eq. 2)
        // Estimated error to the goal
        // f difference between current and frontier node is the total error on the path divided by the length of the
        // path gives the per step error
        const double bvarianceAlpha =
                pow((source->f() - alpha->f()) / (alpha->depth - source->depth) * alpha->distance, 2);
        const double bvarianceBeta = pow((source->f() - beta->f()) / (beta->depth - source->depth) * beta->distance, 2);

        // Mean
        const double mu_alpha = alpha->f() + sqrt(bvarianceAlpha);
        const double mu_beta = beta->f() + sqrt(bvarianceBeta);

        // Number of expansion predicted on the alpha/beta path towards the goal
        // (can't be more than the predicted goal distance)
        double expansionAdvanceOnAlpha = std::min((double)alpha->distance, expansionAdvance / alphaDelay);
        double expansionAdvanceOnBeta = std::min((double)beta->distance, expansionAdvance / betaDelay);

        // Variance' (eq. 3)
        double varianceAlpha = bvarianceAlpha * (expansionAdvanceOnAlpha / alpha->distance);
        double varianceBeta = bvarianceBeta * (expansionAdvanceOnBeta / beta->distance);

        const double alphaStandardDeviation = sqrt(varianceAlpha);
        const double betaStandardDeviation = sqrt(varianceBeta);

        double startAlpha = mu_alpha - 3.0 * alphaStandardDeviation;
        double startBeta = mu_beta - 3.0 * betaStandardDeviation;

        double endAlpha = mu_alpha + 3.0 * alphaStandardDeviation;
        double endBeta = mu_beta + 3.0 * betaStandardDeviation;

        if (endAlpha < startBeta || (alphaStandardDeviation < 0.00001 && betaStandardDeviation < 0.00001)) {
            return 0; // Not overlapping ranges or zero variance
        }

        // Integration step
        double benefit = 0.0;
        double alphaStep = (endAlpha - startAlpha) / 100.0;
        double betaStep = (endBeta - startBeta) / 100.0;

        if (alphaStep < 0.00001 && betaStep < 0.00001) {
            LOG(INFO) << "Double delta spikes";

            // Zero variance
            return 0;
        }

        measureNanoTime([&]() {

            int alphaIndex{0};
            for (double a = startAlpha; a < endAlpha && alphaIndex < 100; a += alphaStep) {
                double sum = 0.0;

                int betaIndex{0};
                for (double b = startBeta; b < endBeta && betaIndex < 100; b += betaStep) {
                    if (a < b) {
                        break;
                    }

                    // PDF of normal distribution
                    sum += (a - b) * Statistic::standardNormalTable100(betaIndex);

                    ++betaIndex;
                }

                benefit += sum * Statistic::standardNormalTable100(alphaIndex);
                ++alphaIndex;
            }

        });

        return benefit;
    }

    Node* createNode(Node* sourceNode, SuccessorBundle<Domain> successor) {
        Planner::incrementGeneratedNodeCount();

        auto successorState = successor.state;
        auto distance = domain.distance(successorState);
        auto heuristic = domain.heuristic(successorState);

        auto costMax = Domain::COST_MAX;

        return nodePool->construct(sourceNode,
                successorState,
                successor.action,
                costMax,
                heuristic,
                true,
                static_cast<Cost>(distance * heuristicError + heuristic),
                distance,
                distance);
    }

    void clearOpenList(PriorityQueue<Node>& openList) {
        openList.forEach([](Node* node) { node->open = false; });
        openList.clear();
    }

    Node* popOpenList(PriorityQueue<Node>& openList) {
        if (openList.isEmpty()) {
            throw MetronomeException("Open list was empty. Goal is not reachable.");
        }

        Node* node = openList.pop();
        node->open = false;
        return node;
    }

    void addToOpenList(PriorityQueue<Node>& openList, Node& node) {
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
    PriorityQueue<Node> openList{Memory::OPEN_LIST_SIZE, fHatComparator};
    std::unordered_map<State, Node*, typename metronome::Hasher<State>> nodes{};
    std::unique_ptr<StaticVector<Node, Memory::NODE_LIMIT>> nodePool{
            std::make_unique<StaticVector<Node, Memory::NODE_LIMIT>>()};

    PriorityQueue<Node> focusedOpenList{Memory::OPEN_LIST_SIZE, fHatComparator};

    unsigned int iterationCounter{0};
    unsigned int expansionCounter{0};

    double heuristicError{0};
    double distanceError{0};

    double nextHeuristicError{0};
    double nextDistanceError{0};

    /** True an identity action was taken and the search should continue where it was left of.
     *  Learning step should be omitted when set.
     */
    bool identityIndicator{false};
    //    Action alphaAction;
};
}

#endif // METRONOME_FRTS_H