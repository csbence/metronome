#ifndef METRONOME_SLOW_RTS_HPP
#define METRONOME_SLOW_RTS_HPP
#include <fcntl.h>
#include <stdlib.h>
#include <MemoryConfiguration.hpp>
#include <algorithm>
#include <domains/SuccessorBundle.hpp>
#include <unordered_map>
#include <utils/StaticVector.hpp>
#include <utils/Statistic.hpp>
#include <vector>
#include "MetronomeException.hpp"
#include "OnlinePlanner.hpp"
#include "easylogging++.h"
#include "experiment/Configuration.hpp"
#include "utils/Hasher.hpp"
#include "utils/PriorityQueue.hpp"
#include "utils/TimeMeasurement.hpp"

namespace metronome {

template <typename Domain, typename TerminationChecker>
class SlowRts final : public OnlinePlanner<Domain, TerminationChecker> {
 public:
  using State = typename Domain::State;
  using Action = typename Domain::Action;
  using Cost = typename Domain::Cost;
  using Planner = Planner<Domain>;
  using ActionBundle = typename Planner::ActionBundle;

  SlowRts(const Domain& domain, const Configuration&) : domain{domain} {
    // Initialize hash table
    nodes.max_load_factor(1);
    nodes.reserve(Memory::NODE_LIMIT);
  }

  std::vector<ActionBundle> selectActions(
      const State& startState,
      TerminationChecker& terminationChecker) override {
    if (domain.isGoal(startState)) {
      // Goal is already reached
      return std::vector<ActionBundle>();
    }

    // Learning phase
    if (openList.isNotEmpty()) {
      learn(terminationChecker);
    }

    nextHeuristicError = 0;
    nextDistanceError = 0;

    auto bestNode = explore(startState, terminationChecker);

    // Apply meta-reasoning
    auto slowDown = findRobustPath(nodes[startState], terminationChecker);

    distanceError = nextDistanceError;
    heuristicError = nextHeuristicError;

    std::vector<ActionBundle> path = extractPath(bestNode, nodes[startState]);
    if (slowDown) {
      // Search a little more
      //            terminationChecker.resetTo(path[0].actionDuration / 1000 *
      //            2); bestNode = explore(startState, terminationChecker,
      //            true);
      //
      //            this->incrementIdentityActionCount();
      //
      // Update the path
      //            path = extractPath(bestNode, nodes[startState]);
      //            path[0].actionDuration = 0;
      path[0].actionDuration = path[0].actionDuration * 2;
      this->incrementIdentityActionCount();
    }

    return path;
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
      stream << "s: " << state << " g: " << g << " h: " << h
             << " fhat: " << fHat << " a: " << action
             << " "
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
    Action topLevelAction{};
    /** Second level action label */
    Action secondLevelAction{};

    /** List of all the predecessors that were discovered in the current
     * exploration phase. */
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

        if (predecessorNode->iteration == iterationCounter &&
            !predecessorNode->open) {
          // This node was already learned and closed in the current iteration
          continue;
          // TODO Review this. This could be incorrect if the action costs are
          // not uniform
        }

        if (!predecessorNode->open) {
          // This node is not open yet, because it was not visited in the
          // current planning iteration

          predecessorNode->h = currentHeuristicValue + predecessor.actionCost;
          assert(predecessorNode->iteration == iterationCounter - 1);
          predecessorNode->iteration = iterationCounter;

          predecessorNode->distanceError = currentNode->distanceError;
          predecessorNode->distance = currentNode->distance + 1;

          addToOpenList(*predecessorNode);
        } else if (predecessorNode->h >
                   currentHeuristicValue + predecessor.actionCost) {
          // This node was visited in this learning phase, but the current path
          // is better then the previous
          predecessorNode->h = currentHeuristicValue + predecessor.actionCost;
          openList.update(*predecessorNode);
        }
      }
    }
  }

  const Node* explore(const State& startState,
                      TerminationChecker& terminationChecker,
                      const bool continued = false) {
    if (!continued) {
      ++iterationCounter;
      clearOpenList();
      openList.reorder(fHatComparator);

      Planner::incrementGeneratedNodeCount();
      Node*& startNode = nodes[startState];

      if (startNode == nullptr) {
        startNode = nodePool->construct(nullptr,
                                        startState,
                                        Action(),
                                        0,
                                        domain.heuristic(startState),
                                        true,
                                        0,
                                        0,
                                        0);
      } else {
        startNode->g = 0;
        startNode->action = Action();
        startNode->predecessors.clear();
        startNode->parent = nullptr;
      }

      startNode->depth = 0;  // Depth relative to the LSS search tree

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

    if (!expanded) {
      LOG(INFO) << "Take emergency step.";
      //            return emergencyStep(popOpenList());
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
      //                successorNode->distanceError / (1.0 - distanceError); //
      //                Dionne 2011 (3.8)
      //            successorNode->fHat = successorNode->g + successorNode->h +
      //            heuristicError * currentDistanceEstimate;

      if (bestChildNode == nullptr || bestChildNode->f() > successorNode->f()) {
        bestChildNode = successorNode;
      }
    }

    return bestChildNode;
  }

  void expandNode(Node* sourceNode) {
    Planner::incrementExpandedNodeCount();

    sourceNode->expansion = ++expansionCounter;

    //        LOG(INFO);
    //        LOG(INFO) << "Expand node: " << sourceNode->toString();
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
        successorNode->open =
            false;  // It is not on the open list yet, but it will be
        successorNode->generation = expansionCounter;
        // parent, action, and actionCost is outdated too, but not relevant.
      }

      // Add the current state as the predecessor of the child state
      successorNode->predecessors.emplace_back(
          sourceNode, successor.action, successor.actionCost);

      // Skip if we got back to the parent
      if (sourceNode->parent != nullptr &&
          successorState == sourceNode->parent->state) {
        continue;
      }

      // only generate those state that are not visited yet or whose cost value
      // are lower than this path
      Cost successorGValueFromCurrent{sourceNode->g + successor.actionCost};
      if (successorNode->g > successorGValueFromCurrent) {
        successorNode->g = successorGValueFromCurrent;
        successorNode->parent = sourceNode;
        successorNode->action = successor.action;
        successorNode->depth = sourceNode->depth + 1;

        double currentDistanceEstimate =
            successorNode->distanceError /
            (1.0 - distanceError);  // Dionne 2011 (3.8)
        successorNode->fHat =
            successorNode->g +
            successorNode->h;  // TODO enable // + heuristicError *
        // currentDistanceEstimate;

        // Labeling
        if (sourceNode->depth == 0) {
          // This is a top level action
          successorNode->topLevelAction = successor.action;
        } else {
          // Inherit the top level action label
          successorNode->topLevelAction = sourceNode->topLevelAction;
        }

        if (sourceNode->depth == 1) {
          // Second layer in the search graph
          successorNode->secondLevelAction = successor.action;
          //
        }

        // Open list management
        if (!successorNode->open) {
          addToOpenList(*successorNode);
        } else {
          openList.update(*successorNode);
        }
        //                LOG(INFO) << "suc: " << successorNode->toString();
      }

      if (bestChildNode == nullptr || bestChildNode->f() > successorNode->f()) {
        bestChildNode = successorNode;
      }
    }

    if (bestChildNode != nullptr) {
      // Local error values (min 0.0)
      double localHeuristicError = bestChildNode->f() - sourceNode->f();
      double localDistanceError =
          bestChildNode->distance - sourceNode->distance + 1;

      localHeuristicError =
          (localHeuristicError < 0.0) ? 0.0 : localHeuristicError;
      localDistanceError =
          (localDistanceError < 0.0) ? 0.0 : localDistanceError;

      // The next error values are the weighted average of the local error and
      // the previous error
      nextHeuristicError += (localHeuristicError - nextHeuristicError) /
                            Planner::getExpandedNodeCount();
      nextDistanceError += (localDistanceError - nextDistanceError) /
                           Planner::getExpandedNodeCount();
    }
  }

  bool findRobustPath(const Node* sourceNode,
                      const TerminationChecker& terminationChecker) {
    //        LOG(INFO) << "Open list size after expansion: " <<
    //        openList.getSize();
    if (openList.getSize() <= 1) {
      return false;
    }

    // Find best two actions
    const auto alphaTargetNode = openList.top();
    const Action alphaAction = alphaTargetNode->topLevelAction;

    Node* betaTargetNode{nullptr};
    openList.forEach([&](Node* node) {
      if (node->topLevelAction != alphaAction &&
          (betaTargetNode == nullptr || betaTargetNode->fHat > node->fHat)) {
        betaTargetNode = node;
      }
    });

    const Node* alphaSourceNode =
        findFirstDescendant(sourceNode, alphaTargetNode);
    const std::pair<Node*, Node*>& alphaTargets =
        findSecondLevelTargets(alphaAction);
    Node* const alphaAlphaTarget = alphaTargets.first;
    Node* const alphaBetaTarget = alphaTargets.second;

    if (alphaAlphaTarget == nullptr || alphaBetaTarget == nullptr) {
      return false;
    }

    Cost alphaExpansionAdvance = alphaSourceNode->g - sourceNode->g;
    double alphaFastExpectedValue = computeBenefit(alphaSourceNode,
                                                   alphaAlphaTarget,
                                                   alphaBetaTarget,
                                                   alphaExpansionAdvance);
    double alphaSlowExpectedValue = computeBenefit(alphaSourceNode,
                                                   alphaAlphaTarget,
                                                   alphaBetaTarget,
                                                   alphaExpansionAdvance * 2,
                                                   alphaExpansionAdvance);

    const auto slowDown = alphaSlowExpectedValue < alphaFastExpectedValue;
    if (slowDown) {
      //            LOG(INFO) << "E[fast]: " << alphaFastExpectedValue;
      //            LOG(INFO) << "E[slow]: " << alphaSlowExpectedValue;
      //            LOG(INFO) << "E[fast - slow]: " << alphaFastExpectedValue -
      //            alphaSlowExpectedValue;
    }
    return slowDown;

    //        // Beta can be null if we can take alpha with two speeds
    //        if (betaTargetNode == nullptr) {
    //            // There are no alternative actions to take
    //            LOG(INFO) << "No alternative actions are available";
    //            return nullptr;
    //        }
    //
    //        // Find the two top level nodes
    //        const Node* alphaSourceNode = findFirstDescendant(sourceNode,
    //        alphaTargetNode); const Node* betaSourceNode =
    //        findFirstDescendant(sourceNode, betaTargetNode);
    //
    //        assert(alphaAction == alphaSourceNode->action);
    //        assert(alphaAction == alphaTargetNode->topLevelAction);
    //
    //        // Find two best actions under alpha and beta
    //        const std::pair<Node*, Node*>& alphaTargets =
    //        findSecondLevelTargets(alphaAction); const std::pair<Node*,
    //        Node*>& betaTargets =
    //        findSecondLevelTargets(betaSourceNode->action);
    //
    //        Node* const alphaAlphaTarget = alphaTargets.first;
    //        Node* const alphaBetaTarget = alphaTargets.second;
    //        Node* const betaAlphaTarget = betaTargets.first;
    //        Node* const betaBetaTarget = betaTargets.second;
    //
    //        if (alphaAlphaTarget == nullptr || alphaBetaTarget == nullptr ||
    //        betaAlphaTarget == nullptr ||
    //                betaBetaTarget == nullptr) {
    //            LOG(INFO) << "No 4 heads found";
    //            return nullptr;
    //        }
    //
    //        // Calculate the expansion advance
    //        //        unsigned int expectedExpansions =
    //        terminationChecker.expansionsPerAction(domain.getActionDuration());
    //
    //        Cost alphaExpansionAdvance = alphaSourceNode->g - sourceNode->g;
    //        Cost betaExpansionAdvance = betaSourceNode->g - sourceNode->g;
    //
    //        // Calculate the benefit
    //        double alphaBenefit = computeBenefit(sourceNode, alphaAlphaTarget,
    //        alphaBetaTarget, alphaExpansionAdvance); double betaBenefit =
    //        computeBenefit(sourceNode, betaAlphaTarget, betaBetaTarget,
    //        betaExpansionAdvance);
    //
    //        LOG(INFO) << "Alpha benefit: " << alphaBenefit;
    //        LOG(INFO) << "Beta benefit: " << betaBenefit;
    //
    //        if (alphaBenefit > betaBenefit) {
    //            return alphaTargetNode;
    //        }
    //
    //        return betaTargetNode;
  }

  static const Node* findFirstDescendant(const Node* sourceNode,
                                         const Node* targetNode) {
    if (sourceNode == targetNode) {
      return nullptr;
    }

    auto currentNode = targetNode;
    while (sourceNode != currentNode->parent) {
      currentNode = currentNode->parent;
    }

    return currentNode;
  }

  /**
   * Find the best two target nodes under a given top level action.
   * @param action top level action
   * @return pair of Node* first is alpha (lower/better), second is beta (second
   * best)
   */
  std::pair<Node*, Node*> findSecondLevelTargets(Action action) {
    Node* alphaTargetNode{nullptr};
    Node* betaTargetNode{nullptr};

    openList.forEach([&](Node* node) {
      if (node->topLevelAction == action) {
        if (alphaTargetNode == nullptr || alphaTargetNode->fHat > node->fHat) {
          betaTargetNode = alphaTargetNode;
          alphaTargetNode = node;
        } else if (betaTargetNode == nullptr ||
                   betaTargetNode->fHat > node->fHat) {
          betaTargetNode = node;
        }
      }
    });

    return std::make_pair(alphaTargetNode, betaTargetNode);
  }

  double expansionDelay(const Node* source, const Node* target) const {
    long delay{1 + expansionCounter - target->generation};
    int count = 1;

    for (Node* current = target->parent;
         current != nullptr && current->generation > 0 && target != source;
         current = current->parent) {
      ++count;
      delay += current->expansion - current->generation;
    }

    return delay / static_cast<double>(count);  // Normalize (calculate average)
  }

  // Returns the utility of searching at a given state
  double computeBenefit(const Node* source,
                        const Node* alpha,
                        const Node* beta,
                        unsigned int expansionAdvance,
                        const Cost delay = 0) const {
    // Calculate delay for alpha and beta path
    double alphaDelay{expansionDelay(source, alpha)};
    double betaDelay{expansionDelay(source, beta)};

    assert(alpha->fHat <= beta->fHat);

    // Variance belief (eq. 2)
    // Estimated error to the goal
    // f difference between current and frontier node is the total error on the
    // path divided by the length of the path gives the per step error
    const double bvarianceAlpha =
        pow((source->f() - alpha->f() + delay) /
                (alpha->depth - source->depth) * alpha->distance,
            2);
    const double bvarianceBeta =
        pow((source->f() - beta->f() + delay) / (beta->depth - source->depth) *
                beta->distance,
            2);

    // Mean
    const double mu_alpha = alpha->f() + delay + sqrt(bvarianceAlpha);
    const double mu_beta = beta->f() + delay + sqrt(bvarianceBeta);

    // Number of expansion predicted on the alpha/beta path towards the goal
    // (can't be more than the predicted goal distance)
    double expansionAdvanceOnAlpha =
        std::min((double)alpha->distance, expansionAdvance / 1000 / alphaDelay);
    double expansionAdvanceOnBeta =
        std::min((double)beta->distance, expansionAdvance / 1000 / betaDelay);

    // Variance' (eq. 3)
    double varianceAlpha =
        bvarianceAlpha * (expansionAdvanceOnAlpha / alpha->distance);
    double varianceBeta =
        bvarianceBeta * (expansionAdvanceOnBeta / beta->distance);

    const double alphaStandardDeviation = sqrt(varianceAlpha);
    const double betaStandardDeviation = sqrt(varianceBeta);

    double startAlpha = mu_alpha - Statistic::SIGMA * alphaStandardDeviation;
    double startBeta = mu_beta - Statistic::SIGMA * betaStandardDeviation;

    double endAlpha = mu_alpha + Statistic::SIGMA * alphaStandardDeviation;
    double endBeta = mu_beta + Statistic::SIGMA * betaStandardDeviation;

    //        startAlpha = startBeta = std::min(startAlpha, startBeta);
    //        endAlpha = endBeta = std::max(startAlpha, startBeta);

    //        if (endAlpha < startBeta || (alphaStandardDeviation < 0.00001 &&
    //        betaStandardDeviation < 0.00001)) {
    //            return 0; // Not overlapping ranges or zero variance
    //        }

    // Integration step
    double benefit = 0.0;
    double alphaStep = (endAlpha - startAlpha) / 100.0;
    double betaStep = (endBeta - startBeta) / 100.0;

    double sum2{0};

    int alphaIndex{0};
    for (double a = startAlpha; alphaIndex < 100; a += alphaStep) {
      double sum = 0.0;

      int betaIndex{0};
      for (double b = startBeta; betaIndex < 100; b += betaStep) {
        // PDF of normal distribution
        sum += std::min(a, b) * Statistic::standardNormalTable100(betaIndex);

        ++betaIndex;
      }

      benefit += sum * Statistic::standardNormalTable100(alphaIndex);
      sum2 += Statistic::standardNormalTable100(alphaIndex);
      ++alphaIndex;
    }

    //        LOG(INFO) << "Benefit" << benefit;

    return benefit;
  }

  Node* createNode(Node* sourceNode, SuccessorBundle<Domain> successor) {
    Planner::incrementGeneratedNodeCount();

    auto successorState = successor.state;
    auto distance = domain.distance(successorState);
    auto heuristic = domain.heuristic(successorState);

    auto costMax = Domain::COST_MAX;

    return nodePool->construct(
        sourceNode,
        successorState,
        successor.action,
        costMax,
        heuristic,
        true,
        static_cast<Cost>(distance * heuristicError + heuristic),
        distance,
        distance);
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

  std::vector<ActionBundle> extractPath(const Node* targetNode,
                                        const Node* sourceNode) const {
    if (targetNode == sourceNode || targetNode == nullptr) {
      return std::vector<ActionBundle>();
    }

    std::vector<ActionBundle> actionBundles;
    auto currentNode = targetNode;

    while (currentNode != sourceNode) {
      // The g difference of the child and the parent gives the action cost from
      // the parent
      actionBundles.emplace_back(currentNode->action,
                                 currentNode->g - currentNode->parent->g);
      currentNode = currentNode->parent;
    }

    std::reverse(actionBundles.begin(), actionBundles.end());
    return actionBundles;
  }

  static int fHatComparator(const Node& lhs, const Node& rhs) {
    if (lhs.fHat < rhs.fHat) return -1;
    if (lhs.fHat > rhs.fHat) return 1;
    if (lhs.g > rhs.g) return -1;
    if (lhs.g < rhs.g) return 1;
    return 0;
  }

  static int hComparator(const Node& lhs, const Node& rhs) {
    if (lhs.h < rhs.h) return -1;
    if (lhs.h > rhs.h) return 1;
    return 0;
  }

  const Domain& domain;
  PriorityQueue<Node> openList{Memory::OPEN_LIST_SIZE, fHatComparator};
  std::unordered_map<State, Node*, typename metronome::Hasher<State>> nodes{};
  std::unique_ptr<StaticVector<Node, Memory::NODE_LIMIT>> nodePool{
      std::make_unique<StaticVector<Node, Memory::NODE_LIMIT>>()};

  unsigned int iterationCounter{0};
  unsigned int expansionCounter{0};

  double heuristicError{0};
  double distanceError{0};

  double nextHeuristicError{0};
  double nextDistanceError{0};
};
}  // namespace metronome

#endif  // METRONOME_SLOW_RTS_HPP
