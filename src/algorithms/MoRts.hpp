#ifndef METRONOME_MO_RTS_HPP
#define METRONOME_MO_RTS_HPP
#include <fcntl.h>
#include <stdlib.h>
#include <algorithm>
#include <boost/pool/object_pool.hpp>
#include <domains/SuccessorBundle.hpp>
#include <unordered_map>
#include <vector>
#include <utils/statistic.hpp>
#include "MetronomeException.hpp"
#include "OnlinePlanner.hpp"
#include "experiment/Configuration.hpp"
#include "utils/Hasher.hpp"
#include "utils/PriorityQueue.hpp"
#define BOOST_POOL_NO_MT

//#define PI 3.141592653589793

namespace metronome {

template <typename Domain, typename TerminationChecker>
class MoRts final : public OnlinePlanner<Domain, TerminationChecker> {
public:
    typedef typename Domain::State State;
    typedef typename Domain::Action Action;
    typedef typename Domain::Cost Cost;
    typedef typename OnlinePlanner<Domain, TerminationChecker>::ActionBundle ActionBundle;

    MoRts(const Domain& domain, const Configuration&) : domain{domain} {
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
        if (!identityIndicator && openList.isNotEmpty()) {
            learn(terminationChecker);
        }

        // Reset error counters if the exploration step is not to be continued
        if (!identityIndicator) {
            nextHeuristicError = 0;
            nextDistanceError = 0;
        }

        const auto bestNode = explore(startState, terminationChecker);

        // Apply meta-reasoning

        auto nano = measureNanoTime(
                [&]() { identityIndicator = isBenefitialToSearch(nodes[startState], terminationChecker); });
        static double avg{-1};
        if (avg < 0) {
            avg = nano;
        } else {
            avg = avg * 0.9 + nano * 0.1;
        }

        LOG_EVERY_N(1, INFO) << "Nano time: " << nano << " exp avg: " << avg;

        // Update error counters if the exploration step is done
        if (!identityIndicator) {
            distanceError = nextDistanceError;
            heuristicError = nextHeuristicError;
        }
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

    void learn(TerminationChecker terminationChecker) {
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

    const Node* explore(const State& startState, TerminationChecker terminationChecker) {
        // Skip initialization if an identity action was applied
        if (!identityIndicator) {
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

            startNode->depth = 0; // Depth relative to the LSS search tree

            startNode->iteration = iterationCounter;
            addToOpenList(*startNode);
        }

        bool expanded = false;
        while (!terminationChecker.reachedTermination() && openList.isNotEmpty()) {
            expanded = true;
            Node* const currentNode = popOpenList();

            if (domain.isGoal(currentNode->state)) {
                return currentNode;
            }

            terminationChecker.notifyExpansion();
            expandNode(currentNode);
        }

        // Normal action - discovery failed - top is current
        // Identity - discovery should succeed.

        assert(!identityIndicator || expanded);

        if (!expanded) {
            LOG(INFO) << "Take emergency step.";
            return emergencyStep(popOpenList());
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

    void expandNode(Node* sourceNode) {
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
                successorNode->g = domain.COST_MAX;
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

    bool isBenefitialToSearch(const Node* sourceNode, const TerminationChecker& terminationChecker) {
        if (openList.getSize() <= 1) {
            return false;
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
            return false;
        }

        // Calculate the expansion advance
        unsigned int expectedExpansions = terminationChecker.expansionsPerAction(domain.getActionDuration());

        // Calculate the benefit
        double benefit = computeBenefit(sourceNode, alphaTargetNode, betaTargetNode, expectedExpansions);
        //        LOG(INFO) << "Benefit: " << benefit;

        return false;
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

        // Variance belief (eq. 2)
        // f difference between current and frontier node is the total error on the path divided by the length of the
        // path gives the per step error
        double bvarianceAlpha = pow((source->f() - alpha->f()) / (alpha->depth - source->depth) * alpha->distance, 2);
        double bvarianceBeta = pow((source->f() - beta->f()) / (beta->depth - source->depth) * beta->distance, 2);

        // Mean
        double mu_alpha = alpha->f() + sqrt(bvarianceAlpha);
        double mu_beta = beta->f() + sqrt(bvarianceBeta);

        // Number of expansion predicted on the alpha/beta path towards the goal
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

        if (endAlpha < startBeta) {
            // Not overlapping intervals
            //            LOG(INFO) << "Not overlapping intervals";
            return 0;
        }

        // Integration step
        double benefit = 0.0;
        double alphaStep = (endAlpha - startAlpha) / 100.0;
        double betaStep = (endBeta - startBeta) / 100.0;

        if (std::abs(alphaStep) < 0.00001 && std::abs(betaStep) < 0.00001) {
            // Zero variance
            return 0;
        }

        LOG(INFO) << measureNanoTime([&]() {

            int alphaIndex{0};
            for (double a = startAlpha; a < endAlpha; a += alphaStep) {
                double sum = 0.0;

                int betaIndex{0};
                for (double b = startBeta; b < endBeta; b += betaStep) {
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



    double normalPDF(double mean, double variance, double variable) const {
        return std::exp(-pow(variable - mean, 2) / (2 * variance)) / (sqrt(2 * 3.1415 * variance));
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
        if (openList.isEmpty()) {
            throw MetronomeException("Open list was empty. Goal is not reachable.");
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
    unsigned int expansionCounter{0};

    double heuristicError{0};
    double distanceError{0};

    double nextHeuristicError{0};
    double nextDistanceError{0};

    /** True an identity action was taken and the search should continue where it was left of.
     *  Learning step should be omitted when set.
     */
    bool identityIndicator{false};
};
}

#endif // METRONOME_MO_RTS_HPP
