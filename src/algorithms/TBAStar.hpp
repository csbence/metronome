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

    return std::vector<ActionBundle>{};
  }

 private:
  /**
   *  Simple linked list of edges
   */
  struct PathEdge {
   public:
    Node* node;
    PathEdge* next;
    Action action;
    Cost actionCost;
  };

  /**
   *  Class to abstract the trace storage and logic. Uses PathEdge
   *  class as a linked list. Maintains hashmap of states
   *  and pointers to the head and end.
   */
  class PathTrace {
   private:
    std::unordered_map<State, PathEdge*, metronome::Hash<State>> edges{};
    PathEdge* pathEnd;
    PathEdge* cursor;
    PathEdge* pathHead;

   public:
    PathTrace(const Node* goalNode) {
      pathEnd = new PathEdge{
          goalNode, nullptr, goalNode->action, goalNode->actionCost};
      pathHead = pathEnd;

      edges.insert({goalNode->state, pathEnd});
    }

    // Deallocate all the edges we created
    ~PathTrace() {
      PathEdge* current = pathHead;
      while (current != nullptr) {
        PathEdge* temp = current->next;
        delete current;

        current = temp;
      }
    }

    /**
     *  Add node to the front of the trace.
     */
    void push_front(const Node* headNode) {
      if (edges.count(headNode->state)) {
        throw MetronomeException(
            "Path trace asked to add a state that is already in the trace");
      }

      pathHead = new PathEdge{
          headNode, pathHead, headNode->action, headNode->actionCost};

      edges.insert({headNode->state, pathHead});
    }

    bool reachedEnd() { return pathHead == pathEnd; }

    /**
     *  Pops the front of the path which should correspond to the
     *  agent's current state. Return an edge usable by user code
     *  representing the next action to take. Deletes the path head
     *  
     *  For multiple commit, consider allowing a variable number
     *  of edges to be popped
     */
    const Edge pop_front() {
      if (reachedEnd()) {
        throw MetronomeException("Attempted to advance past the end of the path");
      }

      Node* predecessor = pathHead->node;

      PathEdge* temp = pathHead;
      pathHead = pathHead->next;
      delete temp;

      return {
          predecessor, pathHead->node, pathHead->action, pathHead->actionCost};
    }
  };

  enum class Optimization { NONE, THRESHOLD, SHORTCUT };

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
  PathTrace* targetPath;
  PathTrace* traceInProgress;
};
}  // namespace metronome