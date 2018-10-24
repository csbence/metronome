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
static const std::string TBA_OPTIMIZATION{"tbaOptimization"};
static const std::string TBA_STRATEGY{"tbaStrategy"};

static const std::string TBA_OPTIMIZATION_NONE{"NONE"};
static const std::string TBA_OPTIMIZATION_THRESHOLD{"THRESHOLD"};
static const std::string TBA_OPTIMIZATION_SHORTCUT{"SHORTCUT"};
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
  typedef typename SearchNode<Domain> Node;
  typedef typename Edge<Domain> Edge;

  static constexpr double traceCost = 10;
  static constexpr double tracebackRatio = 1.0;

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
    if (config.hasMember(TBA_OPTIMIZATION)) {
      std::string optStr = config.getString(TBA_OPTIMIZATION);

      if (optStr == TBA_OPTIMIZATION_NONE)
        optimization = Optimization::NONE;
      else if (optStr == TBA_OPTIMIZATION_THRESHOLD)
        optimization = Optimization::THRESHOLD;
      else if (optStr == TBA_OPTIMIZATION_SHORTCUT)
        optimization = Optimization::SHORTCUT;
    } else {
      optimization = Optimization::NONE;
    }
  }

  std::vector<ActionBundle> selectActions(
      const State& startState,
      TerminationChecker& terminationChecker) override {
    if (domain.isGoal(startState)) {
      // Goal is already reached
      return std::vector<ActionBundle>();
    }
    expansions = 0;  // reset

    Node* thisNode = getNode(startState);
    if (rootNode == nullptr) {
      rootNode = thisNode;
      openList.insertOrUpdate(*rootNode);
    }

    PathTrace* nextPath = getCurrentPath(startState, terminationChecker);

    // If threshold optimization, only switch paths
    // when the g-value of the new path is large enough
    if (optimization == Optimization::THRESHOLD) {
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

    std::vector<ActionBundle> plan {};
    if (targetPath->isHead(startState)) {
      if (targetPath->reachedEnd()) {
        delete targetPath;
        targetPath = nullptr;

        //backtrack one except if root, in which case cycle with last node
        if (thisNode == rootNode) {
          plan.push_back({lastAgentNode->action, lastAgentNode->actionCost});
          targetPath = new PathTrace(lastAgentNode);
        } else {
          targetPath = new PathTrace(thisNode->parent);
          plan.push_back(backtrack(thisNode));
        }
      
      } else {
        //move forward one
        const Edge edge = targetPath->pop_front();
        plan.push_back({edge.action, edge.actionCost});
      }
    } else {
      if (targetPath->isOnPath(startState)) {
        const Edge edge = targetPath->slicePath(startState);

        if (edge.successor == nullptr) { //indicates that we've gone off the end of the path
          delete targetPath;
          targetPath = new PathTrace(thisNode->parent);
          plan.push_back(backtrack(thisNode));

        } else {
          //move forward one
          plan.push_back({edge.action, edge.actionCost});
        }

      } else if (thisNode == rootNode) {
        //flip between root node and successor. Edge case!
        plan.push_back({lastAgentNode->action, lastAgentNode->actionCost});
      } else {
        //we're not on the path. Backtrack
        plan.push_back(backtrack(thisNode));
      }
    }

    lastAgentNode = thisNode; //in case of edge case w/ root node

    return plan;
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
      if (edges.count(headNode->state)) {
        throw MetronomeException(
            "Path trace asked to add a state that is already in the trace");
      }

      pathHead = edgePool.construct(
          headNode, pathHead, headNode->action, headNode->actionCost);

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

  enum class Optimization { NONE, THRESHOLD, SHORTCUT };

  PathTrace* getCurrentPath(const State& startState,
                            TerminationChecker& terminationChecker) {
    // check if we've already found and traced the goal
    bool goalTraced = goalNode != nullptr && targetPath->isEnd(goalNode->state);

    Node* bestNode;

    Node* top = openList.top();
    if (top == nullptr) {
      throw MetronomeException("Goal not reachable - open list empty");
    }
    if (goalTraced) return targetPath;

    if (domain.isGoal(top->state)) {
      goalNode = top;
      bestNode = top;
    } else {
      bestNode = aStar(terminationChecker);
    }

    Node* nextNode;
    if (traceInProgress != nullptr) {
      nextNode = traceInProgress->top()->parent;
    } else {
      traceInProgress = new PathTrace(bestNode);
      nextNode = bestNode->parent;
    }

    unsigned long long int tracebacks {0};
    if (config.getString(TERMINATION_CHECKER_TYPE) == TERMINATION_CHECKER_EXPANSION) {
      tracebacks =
          static_cast<unsigned long long int>(config.getLong(ACTION_DURATION) * tracebackRatio);
    }

    PathTrace* currentTrace = traceInProgress;
    while (tracebacks > 0) {
      currentTrace->push_front(nextNode);

      if (nextNode->state == rootNode->state || nextNode->state == startState) {
        traceInProgress = nullptr;
        break;
      }

      nextNode = nextNode->parent;
      tracebacks--;
    }

    return traceInProgress == nullptr ? currentTrace : targetPath;
  }

  Node* aStar(TerminationChecker& terminationChecker) {
    while (!terminationChecker.reachedTermination() && openList.isNotEmpty()) {
      Node* currentNode = openList.top();
      if (domain.isGoal(currentNode->state)) {
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
    expansions++;

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

  /*  Note: This depends on the relatively dangerous
   *  domain function getActionDuration(). This will
   *  not work in domains with non-constant action
   *  costs. Also depends on the domain being able to
   *  invert actions
   */
  ActionBundle backtrack(const Node* node) {
    Action backupAction = node->action.inverse();

    return { backupAction, domain.getActionDuration() };
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
  static constexpr Comparator fComparator = &fComparator<Node::Domain>;
  static constexpr Comparator hComparator = &hComparator<Node::Domain>;

  // Config
  Optimization optimization;
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
  unsigned long long int expansions = 0;
};
}  // namespace metronome