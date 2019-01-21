#pragma once

#include <fcntl.h>
#include <ostream>
#include <unordered_map>
#include <vector>
#include "MemoryConfiguration.hpp"
#include "MetronomeException.hpp"
#include "OnlinePlanner.hpp"
#include "Planner.hpp"
#include "domains/SuccessorBundle.hpp"
#include "dynamic_priority_queue.hpp"
#include "easylogging++.h"
#include "experiment/Configuration.hpp"
#include "experiment/termination/TimeTerminationChecker.hpp"
#include "utils/Hash.hpp"
#include "utils/ObjectPool.hpp"
#include "utils/PriorityQueue.hpp"
#include "utils/TimeMeasurement.hpp"
#include "visualization/Visualizer.hpp"

namespace metronome {

template <typename Domain, typename TerminationChecker>
class TimeBoundedAStar final
    : public OnlinePlanner<Domain, TerminationChecker> {
 public:
  using State = typename Domain::State;
  using Action = typename Domain::Action;
  using Cost = typename Domain::Cost;
  using Planner = Planner<Domain>;
  using OnlinePlanner = OnlinePlanner<Domain, TerminationChecker>;
  using ActionBundle = typename Planner::ActionBundle;

  // Only used for visualizations
  static constexpr std::size_t NODE_ID_OFFSET = 100000;

  TimeBoundedAStar(const Domain &domain, const Configuration &configuration)
      : domain(domain),
        weight(configuration.getDouble(WEIGHT, 1.0)),
        projection(configuration.getBool(PROJECTION, false)),
        shortcut(configuration.getBool(SHORTCUT, false)) {
    // Initialize hash table
    nodes.max_load_factor(1);
    nodes.reserve(Memory::NODE_LIMIT);
  }

  virtual std::vector<std::pair<std::string, std::int64_t>> getAttributes()
      const final override {
    auto attributes = OnlinePlanner::getAttributes();
    attributes.emplace_back("backtrackCount", backtrackCount);

    return attributes;
  }

  std::vector<ActionBundle> selectActions(
      const State &agentState,
      TerminationChecker &terminationChecker) override {
    OnlinePlanner::beginIteration();
    ++iteration;
    if (domain.isGoal(agentState)) {
      // Goal is already reached
      return {};
    }

    // ---    Initialize    ---
    if (nodePool.empty()) {
      createInitialNode(agentState);
      explore(agentState, terminationChecker);
    }

    // info for shortcutting
    auto currentNode = nodes.at(agentState);
    if (shortcut) {
      std::size_t mostRecentIteration =
          std::max(currentNode->pathIterationA, currentNode->pathIterationB);

      // only switch tracked path if we've reached the next target
      if (mostRecentIteration == projectionIteration) {
        agentPathIteration = mostRecentIteration;
        agentPathIsA = agentPathIteration == currentNode->pathIterationA;
      }
    }


    std::vector<ActionBundle> rootToTargetPath;
    std::vector<ActionBundle> selectedActions;

    const auto extractionStartTime = currentNanoTime();
    auto targetNode = goalNode != nullptr ? goalNode : openList.top();

    bool projectionValid = checkProjectedPath(targetNode);

    if (!projectionValid) {
      if (shortcutFound) {
        reversedProjectedPath = extractPath(currentNode, goalNode, true);

        selectedActions = {reversedProjectedPath.back()};
        reversedProjectedPath.pop_back();
      } else {
//        PRINT_NANO_TIME("Path Traceback") {
          rootToTargetPath = extractPath(targetNode, rootNode);
//        }
//        PRINT_NANO_TIME("Extract Action") {
          selectedActions = extractAction(agentState, rootToTargetPath);
//        }
      }
    } else {
      selectedActions = {reversedProjectedPath.back()};
      reversedProjectedPath.pop_back();
    }

    const auto extractionEndTime = currentNanoTime();

    const auto explorationStartTime = currentNanoTime();
//    PRINT_NANO_TIME("Explore") {
      explore(agentState, terminationChecker);
//    }
    const auto explorationEndTime = currentNanoTime();

#ifdef STREAM_GRAPH
    visualizeProgress(agentState, rootToTargetPath);
#endif

    OnlinePlanner::recordAttribute("explorationTime",
                                   explorationEndTime - explorationStartTime);
    OnlinePlanner::recordAttribute("extractionTime",
                                   extractionEndTime - extractionStartTime);

    return selectedActions;
  }
  std::vector<ActionBundle> extractAction(
      const State &agentState,
      const std::vector<ActionBundle> &rootToTargetPath) {
    for (size_t i = 0; i < rootToTargetPath.size(); ++i) {
      if (rootToTargetPath[i].expectedTargetState == agentState) {
        if (rootToTargetPath.size() == i + 1) {
          // The agent reached the frontier
          ActionBundle actionBundle{domain.getIdentityAction(),
                                    domain.getActionDuration()};
          actionBundle.label = "wait at the frontier";
          actionBundle.expectedTargetState = agentState;
          return {actionBundle};
        }

        if (projection && rootToTargetPath.size() > i + 2) {
          // Update the projected path
          reversedProjectedPath = {rootToTargetPath.begin() + i + 2,
                                   rootToTargetPath.end()};
          std::reverse(reversedProjectedPath.begin(),
                       reversedProjectedPath.end());
        }

        auto actionBundle = rootToTargetPath.at(i + 1);
        actionBundle.label = "forward step";

        return {actionBundle};
      }
    }

    if (agentState == rootNode->state) {
      if (rootToTargetPath.empty()) {
        throw MetronomeException("No path found from the root.");
      }

      auto firstActionBundle = rootToTargetPath[0];
      firstActionBundle.label = "firstStep";
      return {firstActionBundle};
    }

    // Backtrack
    ActionBundle actionBundle = backtrackFromNode(agentState);
    ++backtrackCount;
    return {actionBundle};
  }

  ActionBundle backtrackFromNode(const State &sourceState) const {
    const auto nodeIterator = nodes.find(sourceState);
    if (nodeIterator == nodes.cend()) {
      throw MetronomeException(
          "Agent node was not created yet. Cannot "
          "backtrack.");
    }

    const Node *agentNode = nodeIterator->second;
    const Action action = agentNode->action.inverse();
    const Cost cost = agentNode->g - agentNode->parent->g;
    const State expectedState = agentNode->parent->state;

    ActionBundle actionBundle(action, cost);
    actionBundle.expectedTargetState = expectedState;
    actionBundle.label = "backtrack";
    return actionBundle;
  }

  void visualizeProgress(const State &agentState,
                         const std::vector<ActionBundle> &path) {
    visualizer.addNode(0, agentState.getX(), agentState.getY(), 0, 4, "source");

    std::size_t id = 0;
    for (const auto &actionBundle : path) {
      const auto &state = actionBundle.expectedTargetState;
      visualizer.addNode(++id, state.getX(), state.getY(), 0, 2, "path");
    }

    visualizer.post();

    visualizer.removeNode(0);
    while (id > 0) {
      visualizer.removeNode(id);
      --id;
    }
  }

 private:
  class Edge;

  class Node {
   public:
    Node(Node *parent, const State &state, Action action, Cost g, Cost h)
        : parent{parent}, state{state}, action{std::move(action)}, g{g}, h{h} {}

    Cost f() const { return g + h; }

    unsigned long hash() const { return state.hash(); }

    bool operator==(const Node &node) const { return state == node.state; }

    std::string toString() const {
      std::ostringstream stream;
      stream << "s: " << state << " g: " << g << " h: " << h << " f: " << f()
             << " a: " << action << " p: ";
      if (parent == nullptr) {
        stream << "None";
      } else {
        stream << parent->state;
      }
      return stream.str();
    }

    friend std::ostream &operator<<(std::ostream &os, const Node &node) {
      os << "state: " << node.state << " action: " << node.action
         << " g: " << node.g << " h: " << node.h;
      return os;
    }

    friend std::ostream &operator<<(std::ostream &os, const Node *node) {
      return operator<<(os, *node);
    }

    /** Parent node */
    Node *parent;
    /** Internal state */
    const State state;
    /** Action that led to the current node from the parent node */
    Action action;
    /** Cost from the root node */
    Cost g;
    /** Heuristic cost of the node */
    Cost h;
    bool closed = false;
    std::size_t projectionIteration = 0;

    /**
     * Keeping track of paths for shortcutting.
     * we need to efficiently find the intersection with the agent's current path,
     * so track agent path iteration in one variable and more recent path info
     * in another. We will switch between the variables.
     */
    std::size_t pathIterationA = 0;
    std::size_t pathIterationB = 0;
    std::size_t shortcutSearchIteration = 0;
  };

  struct NodeComparatorWeightedF {
    NodeComparatorWeightedF(double weight) : weight(weight) {}

    int operator()(const Node *lhs, const Node *rhs) const {
      if (lhs->g + weight * lhs->h < rhs->g + weight * rhs->h) return -1;
      if (lhs->g + weight * lhs->h > rhs->g + weight * rhs->h) return 1;
      if (lhs->g > rhs->g) return -1;
      if (lhs->g < rhs->g) return 1;
      return 0;
    }

    const double weight;
  };

  struct NodeEquals {
    bool operator()(const Node *lhs, const Node *rhs) const {
      return lhs == rhs;
    }
  };

  class Edge {
   public:
    Edge(Node *predecessor, Action action, Cost actionCost)
        : predecessor{predecessor}, action{action}, actionCost{actionCost} {}

    Node *predecessor;
    const Action action;
    const Cost actionCost;
  };

  void createInitialNode(const State &initialState) {
    Planner::incrementGeneratedNodeCount();

    Node *&initialNode = nodes[initialState];
    initialNode = nodePool.construct(Node{
        nullptr, initialState, Action(), 0, domain.heuristic(initialState)});

    rootNode = initialNode;
    openList.push(initialNode);
  }

  void explore(const State &agentState, TerminationChecker &terminationChecker) {
    // If we've reached the goal and shortcuts are on, re-seed the open list
    // with all nodes on the path from the goal to the intersection with
    // the agent's current path. Resetting G and H to be relative to the
    // goal and agent respectively
    if (shortcut && goalNode != nullptr && !shortcutFound) {
      openList.clear();
      shortcutIterationCount++;

      std::size_t count = 0;
      auto currentNode = goalNode;
      while ((agentPathIsA && currentNode->pathIterationA != agentPathIteration) ||
          (!agentPathIsA && currentNode->pathIterationB != agentPathIteration)) {

        currentNode->closed = false;
        currentNode->g = count;
        currentNode->h = domain.heuristic(currentNode->state, agentState);
        currentNode->shortcutSearchIteration = shortcutIterationCount;

        openList.insertOrUpdate(currentNode);
        currentNode = currentNode->parent;

        // check for termination, but don't check often
        if (count++ % 1000 == 999 && terminationChecker.reachedTermination()) break;
      }
    }

    while (!terminationChecker.reachedTermination() && !openList.empty()) {
      if (goalNode != nullptr && (!shortcut || shortcutFound)) return;

      auto currentNode = openList.pop();
      expandNode(currentNode, agentState);
      terminationChecker.notifyExpansion();

      // Visualization
      std::size_t id = nodePool.index(currentNode) + NODE_ID_OFFSET;
      visualizer.addNode(
          id, currentNode->state.getX(), currentNode->state.getY(), 0, 1);

      if (currentNode->parent != nullptr) {
        visualizer.addEdge(
            id, nodePool.index(currentNode->parent) + NODE_ID_OFFSET, id);
      }
    }
  }

  void expandNode(Node *sourceNode, const State &agentState) {
    Planner::incrementExpandedNodeCount();

    const bool shortcutMode = shortcut && goalNode != nullptr;

    if (sourceNode->closed) return;
    sourceNode->closed = true;

    if (domain.isGoal(sourceNode->state) && !shortcutMode) {
      LOG(INFO) << "Goal was expanded!";
      this->goalFound();
      goalNode = sourceNode;
    } else if (shortcutMode && sourceNode->state == agentState) {
      LOG(INFO) << "Found shortcut to agent from goal";
      shortcutFound = true;
      reversedProjectedPath.clear(); //invalidate previous projection
    }

    for (auto successor : domain.successors(sourceNode->state)) {
      auto successorState = successor.state;
      Node *&successorNode = nodes[successorState];

      if (successorNode == nullptr) {
        Planner::incrementGeneratedNodeCount();
        
        Cost h;
        if (shortcutMode) h = domain.heuristic(successor.state, agentState);
        else h = domain.heuristic(successor.state);

        successorNode = nodePool.construct(sourceNode,
                                           successor.state,
                                           successor.action,
                                           std::numeric_limits<Cost>::max(),
                                           h);
        successorNode->projectionIteration = sourceNode->projectionIteration;
        successorNode->shortcutSearchIteration = shortcutIterationCount;
      }

      // Similar to an LSS algorithm, invalidate certain info if we are in the shortcut phase
      // This will only happen if we are in shortcut mode
      if (successorNode->shortcutSearchIteration != shortcutIterationCount) {
        successorNode->shortcutSearchIteration = shortcutIterationCount;
        successorNode->closed = false;
        successorNode->g = std::numeric_limits<Cost>::max();
        successorNode->h = domain.heuristic(successorState, agentState);
      }

      // Skip if we got back to the parent
      if (sourceNode->parent != nullptr &&
          successorState == sourceNode->parent->state) {
        continue;
      }

      Cost successorGValueFromCurrent{sourceNode->g + successor.actionCost};
      if (successorNode->g > successorGValueFromCurrent) {
        successorNode->g = successorGValueFromCurrent;
        successorNode->parent = sourceNode;
        successorNode->action = successor.action;
        successorNode->projectionIteration = sourceNode->projectionIteration;

        openList.insertOrUpdate(successorNode);
      }
    }
  }

  bool checkProjectedPath(Node *targetNode) {
    if (projection && targetNode->projectionIteration == projectionIteration &&
        !reversedProjectedPath.empty()) {
      // The projected path is still a prefix of the final path
      return true;
    } else {
      // The projected path is no longer a prefix
      // The last node of the new projection is going to be the targetNode
      ++projectionIteration;
      reversedProjectedPath.clear();
      targetNode->projectionIteration = projectionIteration;
    }

    return false;
  }

  std::vector<ActionBundle> extractPath(Node *targetNode,
                                        const Node *sourceNode,
                                        bool reverseActions = false) const {
    if (targetNode == sourceNode) return {};

    std::vector<ActionBundle> actionBundles;
    auto currentNode = targetNode;

    while (currentNode != sourceNode) {
      if (shortcut) {
        // don't overwrite iteration number of agent's current path
        if (agentPathIsA) {
          currentNode->pathIterationB = projectionIteration;
        } else {
          currentNode->pathIterationA = projectionIteration;
        }
      }

      Action currentAction = currentNode->action;
      State expectedTargetState = currentNode->state;
      // in shortcuts, we search from goal to agent meaning the "parent" node
      // has the state we actually want to transition to
      // Depends entirely on all actions being reversable
      if (reverseActions) {
        currentAction = currentAction.inverse();
        expectedTargetState = currentNode->parent->state;
      }

      ActionBundle actionBundle(currentAction,
                                currentNode->g - currentNode->parent->g);
      actionBundle.expectedTargetState = expectedTargetState;
      actionBundles.push_back(std::move(actionBundle));

      currentNode = currentNode->parent;
    }

    std::reverse(actionBundles.begin(), actionBundles.end());
    return actionBundles;
  }

  const Domain &domain;
  const double weight;
  const bool projection;
  const bool shortcut;
  const Node *rootNode;

  cserna::DynamicPriorityQueue<
      Node *,
      cserna::NonIntrusiveIndexFunction<Node *, Hash<Node>, NodeEquals>,
      NodeComparatorWeightedF,
      Memory::OPEN_LIST_SIZE,
      Memory::OPEN_LIST_SIZE>
      openList{{weight}};
  std::unordered_map<State, Node *, typename metronome::Hash<State>> nodes;
  ObjectPool<Node, Memory::NODE_LIMIT> nodePool;

  std::size_t iteration = 0;
  std::size_t backtrackCount = 0;  // Measurement only
  std::size_t projectionIteration = 0;
  std::vector<ActionBundle> reversedProjectedPath;

  Node *goalNode = nullptr;
  bool agentPathIsA = false;
  std::size_t agentPathIteration = 0;
  std::size_t shortcutIterationCount = 0;
  bool shortcutFound = false;

  Visualizer visualizer;
};

}  // namespace metronome
