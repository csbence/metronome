#pragma once

/* Standard */
#include <string>
#include <unordered_map>
#include <vector>

/* Third Party */
#include "easylogging++.h"

#include <MemoryConfiguration.hpp>
#include <algorithms/planner_tools/Comparators.hpp>
#include <algorithms/planner_tools/Nodes.hpp>
#include <domains/SuccessorBundle.hpp>
#include "MetronomeException.hpp"
#include "OnlinePlanner.hpp"
#include "experiment/Configuration.hpp"
#include "utils/Hash.hpp"
#include "utils/ObjectPool.hpp"
#include "utils/PriorityQueue.hpp"
#include "utils/TimeMeasurement.hpp"

namespace metronome {

// configuration options and values
static const std::string TBA_STRATEGY{"tbaStrategy"};
static const std::string TBA_STRATEGY_A_STAR{"A_STAR"};
static const std::string TBA_STRATEGY_GBFS{"GBFS"};

/**
 * TBA* implementation.
 * Algorithm performs a single A* search interrupted
 * by the time bound. When interrupted, returns path
 * to the best-so-far node on the search graph
 */
template <typename Domain, typename TerminationChecker>
class TBAStar final : public OnlinePlanner<Domain, TerminationChecker> {
 public:
  typedef typename Domain::State State;
  typedef typename Domain::Action Action;
  typedef typename Domain::Cost Cost;
  typedef typename OnlinePlanner<Domain, TerminationChecker>::ActionBundle
      ActionBundle;
  using Node = SearchNode<Domain>;
  using Edge = metronome::Edge<Domain>;
  using OnlinePlanner = metronome::OnlinePlanner<Domain, TerminationChecker>;
  using Planner = metronome::Planner<Domain>;

  static constexpr double traceCost = 10;
  static constexpr double tracebackRatio = 0.1;

  TBAStar(const Domain& domain, const Configuration& config)
      : domain{domain}, config{config} {
    // Initialize hash table
    nodes.max_load_factor(1);
    nodes.reserve(Memory::NODE_LIMIT);

    // weight configuration
    if (config.hasMember(WEIGHT)) {
      weight = config.getDouble(WEIGHT);
    } else {
      weight = 1.0;
    }

    // strategy configuration
    bool aStar = false;
    bool gbfs = false;

    if (config.hasMember(TBA_STRATEGY)) {
      std::string strategy = config.getString(TBA_STRATEGY);
      if (strategy == TBA_STRATEGY_A_STAR) {
        aStar = true;
      } else if (strategy == TBA_STRATEGY_GBFS) {
        gbfs = true;
      }
    } else {
      aStar = true;
    }

    if (gbfs) {
      openList.reorder(hComparator);
    } else if (!aStar) {
      throw metronome::MetronomeException(
          "Unknown strategy specified for TBA*" +
          config.getString(TBA_STRATEGY));
    }

    // Optimization configuration
    if (config.hasMember(SHORTCUT)) {
      this->shortcut = config.getBool(SHORTCUT);
    }

    if (config.hasMember(THRESHOLD)) {
      this->threshold = config.getBool(THRESHOLD);
    }
  }

  std::vector<ActionBundle> selectActions(
      const State& startState,
      TerminationChecker& terminationChecker) override {
    OnlinePlanner::beginIteration();

    OnlinePlanner::recordAttribute("backtrack", 0);

    if (domain.isGoal(startState)) {
      // Goal is already reached
      return std::vector<ActionBundle>();
    }


    Node* thisNode = getNode(startState);
    if (rootNode == nullptr) {
      rootNode = thisNode;
      rootNode->g = 0;
      openList.insertOrUpdate(*rootNode);

      // only allow 30% of time on first iteration
      // We will give the next Explore phase the rest
      // after the first path is traced
      terminationChecker.setRatio(0.3);
      explore(terminationChecker, true); // first iteration
      terminationChecker.setRatio(1.0);
    }
    else {
      terminationChecker.setRatio(tracebackRatio);
    }

    PathTrace* nextPath = getCurrentPath(startState, terminationChecker);

    // If threshold optimization, only switch paths
    // when the g-value of the new path is large enough
    if (threshold) {
      if (targetPath != nullptr && nextPath != targetPath) {
        bool gIsGreater = nextPath->costIsAsBig(targetPath);
        bool agentAtEnd =
            targetPath->reachedEnd() && targetPath->isHead(startState);
        if (gIsGreater || agentAtEnd) {
          // destruct old path trace
          delete targetPath;
          targetPath = nullptr;
        } else {
          delete nextPath;
          nextPath = targetPath;
        }
      }
    }

    if (nextPath != targetPath) {
      if (targetPath != nullptr) delete targetPath;

      targetPath = nextPath;
    }

    std::vector<ActionBundle> plan = extractPlan(thisNode);

    // Use remaining time to explore
    terminationChecker.setRatio(1.0);
    explore(terminationChecker);
    
    return plan;
  }

  std::unordered_map<std::string, std::string> getAttributeAggregationStrategies() const override {
    return {{"backtrack", "sum"}};
  }

 private:
  static constexpr std::size_t EDGE_LIMIT = Memory::NODE_LIMIT / 10;

  /**
   *  Simple linked list of edges
   */
  struct PathEdge {
   public:
    Node* node;
    PathEdge* next;
    Action action;
    Cost actionCost;
    PathEdge(Node* node,
             PathEdge* next,
             Action action,
             Cost actionCost)
        : node{node}, next{next}, action{action}, actionCost{actionCost} {}
  };
  static inline ObjectPool<PathEdge, EDGE_LIMIT> edgePool{};

  /**
   *  Class to abstract the trace storage and logic. Uses PathEdge
   *  class as a linked list. Maintains hashmap of states
   *  and pointers to the head and end.
   */
  class PathTrace {
   private:
    std::unordered_map<State, PathEdge*, metronome::Hash<State>> edges{};
    PathEdge* pathEnd;
    PathEdge* pathHead;

   public:
    PathTrace(Node* goalNode) {
      pathEnd = edgePool.construct(
          goalNode, nullptr, goalNode->action, goalNode->actionCost);
      pathHead = pathEnd;

      edges.insert({goalNode->state, pathEnd});
    }

    // Deallocate all the edges we created
    ~PathTrace() {
      PathEdge* current = pathHead;
      while (current != nullptr) {
        PathEdge* temp = current->next;
        edgePool.destruct(current);

        current = temp;
      }
    }

    /**
     *  Add node to the front of the trace.
     */
    void push_front(Node* headNode) {
      if (edges.count(headNode->state) && pathHead != nullptr) {
        throw MetronomeException(
            "Path trace asked to add a state that is already in the trace");
      }

      pathHead = edgePool.construct(
          headNode, pathHead, headNode->action, headNode->actionCost);
      if (pathHead->next == nullptr) {
        pathEnd = pathHead;
      }

      edges.insert({headNode->state, pathHead});
    }

    Node* top() { return pathHead->node; }

    bool reachedEnd() const { return pathHead == pathEnd || pathHead == nullptr; }
    bool isHead(const State& state) const { return state == pathHead->node->state; }
    bool isEnd(const State& state) const { return state == pathEnd->node->state; }
    bool isOnPath(const State& state) const { return edges.count(state) > 0; }
    bool costIsAsBig(const PathTrace* rhs) {
      return pathEnd->node->g >= rhs->pathEnd->node->g;
    }

    /**
     *  Slice the path from the given state. All edges before and including the state
     *  will be deleted, and an edge usable by the user is returned
     *  If the state is not in the path, this effectively deletes the entire
     *  path.
     */
    Edge slicePath(const State& state) {
      Node* predecessor;
      while (pathHead != nullptr) {
        predecessor = pathHead->node;

        PathEdge* temp = pathHead;
        pathHead = pathHead->next;
        edgePool.destruct(temp);

        //we've reached the end of the slice
        if (predecessor->state == state) break;
      }

      return pathHead == nullptr ?
          Edge{ nullptr, nullptr, Action(), std::numeric_limits<Cost>::max() }
        : Edge{ predecessor, pathHead->node, pathHead->action, pathHead->actionCost};
    }

    /**
     *  Pops the front of the path which should correspond to the
     *  agent's current state. Return an edge usable by user code
     *  representing the next action to take. Deletes the path head
     *
     *  For multiple commit, consider allowing a variable number
     *  of edges to be popped
     */
    Edge pop_front() {
      if (reachedEnd()) {
        throw MetronomeException(
            "Attempted to advance past the end of the path");
      }

      Node* predecessor = pathHead->node;

      PathEdge* temp = pathHead;
      pathHead = pathHead->next;
      edgePool.destruct(temp);

      return {
          predecessor, pathHead->node, pathHead->action, pathHead->actionCost};
    }
  };

  PathTrace* getCurrentPath(const State& startState,
                            TerminationChecker& terminationChecker) {
    // check if we've already found and traced the goal
    bool goalTraced = goalNode != nullptr && targetPath->isEnd(goalNode->state);

    Node* bestNode = openList.top();

    if (bestNode == nullptr) {
      throw MetronomeException("Goal not reachable - open list empty");
    }
    if (goalTraced) return targetPath;

    if (domain.isGoal(bestNode->state)) {
      goalNode = bestNode;
    }

    Node* nextNode;
    if (traceInProgress != nullptr) {
      nextNode = traceInProgress->top()->parent;
    } else {
      traceInProgress = new PathTrace(bestNode);
      nextNode = bestNode->parent;
    }

    unsigned long int traces = 0L;

    PathTrace* currentTrace = traceInProgress;
    while (!terminationChecker.reachedTermination()
        || targetPath == nullptr) {
      currentTrace->push_front(nextNode);

      if (nextNode->state == rootNode->state || nextNode->state == startState) {
        traceInProgress = nullptr;
        break;
      }

      nextNode = nextNode->parent;
      traces++;
    }

    OnlinePlanner::recordAttribute("tracebacks", traces);

    return traceInProgress == nullptr ? currentTrace : targetPath;
  }

  std::vector<ActionBundle> extractPlan(Node* currentNode) {
    std::vector<ActionBundle> plan {};

    if (targetPath->isHead(currentNode->state)) { // agent is at the head of target path
      if (targetPath->reachedEnd()) {
        // wait at the edge of the path until a new target path is traced
        plan.push_back(getIdentityBundle());

      } else {
        //move forward one
        const Edge edge = targetPath->pop_front();
        plan.push_back({ edge.action, edge.actionCost });
      }
    } else {
      if (targetPath->isOnPath(currentNode->state)) {
        const Edge edge = targetPath->slicePath(currentNode->state);

        if (edge.successor == nullptr) { //indicates that we've gone off the end of the path
          targetPath->push_front(currentNode); // re-seed with current node so we can park here
          plan.push_back(getIdentityBundle());

        } else {
          //move forward one
          plan.push_back({ edge.action, edge.actionCost });
        }

      } else if (currentNode == rootNode) {
        //Identity Action. Edge case!
        plan.push_back(getIdentityBundle());
      } else {
        //we're not on the path. Backtrack
        plan.push_back(backtrack(currentNode));
      }
    }

    return plan;
  }

  Node* explore(TerminationChecker& terminationChecker, bool first = false) {
    while ((!terminationChecker.reachedTermination() && openList.isNotEmpty())
        || (first && Planner::getExpandedNodeCount() == 0)) {
      Node* currentNode = openList.top();
      if (domain.isGoal(currentNode->state)) {
        if (OnlinePlanner::getGoalFirstFoundIteration() == 0) {
          OnlinePlanner::goalFound();
          LOG(INFO) << "Goal found!";
        }
        return currentNode;
      }

      openList.pop();
      currentNode->open = false;
      currentNode->closed = true;

      expandNode(currentNode);
      terminationChecker.notifyExpansion();
    }

    Node* openListTop = openList.top();
    if (openListTop == nullptr) {
      throw MetronomeException("Goal not reachable");
    }
    return openListTop;
  }

  void expandNode(Node* sourceNode) {
    Planner::incrementExpandedNodeCount();

    for (auto successor : domain.successors(sourceNode->state)) {
      Node* successorNode = getNode(successor.state);

      if (successorNode->closed) continue;

      Cost successorG = successor.actionCost + sourceNode->g;
      if (successorG < successorNode->g) {
        successorNode->g = successorG;
        successorNode->actionCost = successor.actionCost;
        successorNode->action = successor.action;
        successorNode->parent = sourceNode;

        openList.insertOrUpdate(*successorNode);
      }
    }
  }

  /*  Note: This depends on the domain being able to
   *  invert actions
   */
  ActionBundle backtrack(const Node* node) {
    Action backupAction = node->action.inverse();

    OnlinePlanner::recordAttribute("backtrack", 1);
    return { backupAction, domain.getActionDuration(backupAction) };
  }
  /* Note: This depends on an available Identity action
  */
  ActionBundle getIdentityBundle() const {
    Action identity = domain.getIdentityAction();
    return {identity, domain.getActionDuration(identity)};
  }

  Node* getNode(const State& state) {
    Node* node = nodes[state];

    if (node == nullptr) {
      Planner::incrementGeneratedNodeCount();
      Cost h = static_cast<Cost>(domain.heuristic(state) * weight);
      node = nodePool.construct(nullptr,
                                state,
                                Action(),
                                0,
                                std::numeric_limits<Cost>::max(),
                                h,
                                true);
      nodes[state] = node;
    }

    return node;
  }

  typedef int (*Comparator)(const Node&, const Node&);
  static constexpr Comparator fComparator = &metronome::fComparator<Domain>;
  static constexpr Comparator hComparator = &metronome::hComparator<Domain>;

  // Config
  bool threshold;
  bool shortcut;
  double weight;

  const Domain& domain;
  const Configuration& config;
  PriorityQueue<Node> openList{Memory::OPEN_LIST_SIZE, fComparator};
  std::unordered_map<State, Node*, typename metronome::Hash<State>> nodes{};
  ObjectPool<Node, Memory::NODE_LIMIT> nodePool;
  PathTrace* targetPath = nullptr;
  PathTrace* traceInProgress = nullptr;
  Node* goalNode = nullptr;
  Node* rootNode = nullptr;
  Node* lastAgentNode = nullptr;
};
}  // namespace metronome