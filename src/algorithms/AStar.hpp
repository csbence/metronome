#ifndef METRONOME_ASTAR_HPP
#define METRONOME_ASTAR_HPP
#define BOOST_POOL_NO_MT

#include <boost/pool/object_pool.hpp>
#include <unordered_map>
#include <utils/PriorityQueue.hpp>
#include <vector>
#include "OfflinePlanner.hpp"
#include "Planner.hpp"
#include "experiment/Configuration.hpp"
#include "utils/Hasher.hpp"

namespace metronome {

template <typename Domain>
class AStar final : public OfflinePlanner<Domain> {
    typedef typename Domain::State State;
    typedef typename Domain::Action Action;
    typedef typename Domain::Cost Cost;

public:
    AStar(const Domain& domain, const Configuration&) : domain(domain), openList(10000000, fValueComparator) {
        // Force the object pool to allocate memory
        State state;
        Node node = Node(nullptr, std::move(state), Action(), 0, 0);
        nodePool.destroy(nodePool.construct(node));
    }

    std::vector<Action> plan(const State& startState) override {
        Cost heuristic = domain.heuristic(startState);
        Node localStartNode = Node(nullptr, startState, Action(), 0, heuristic);

        auto startNode = nodePool.construct(localStartNode);

        nodes[localStartNode.state] = startNode;

        openList.push(localStartNode);

        while (!openList.isEmpty()) {
            Planner::incrementExpandedNodeCount();
            Node* currentNode = openList.pop();

            if (domain.isGoal(currentNode->state)) {
                std::vector<Action> actions;
                // Goal is reached

                while (!domain.isStart(currentNode->state)) {
                    actions.push_back(currentNode->action);
                    currentNode = currentNode->parent;
                }

                std::reverse(actions.begin(), actions.end());
                return actions;
            }

            auto successors = domain.successors(currentNode->state);
            for (auto successor : successors) {
                //                if (successor.state == currentNode->state) {
                //                    continue; // Skip parent TODO this might be unnecessary
                //                }

                Planner::incrementGeneratedNodeCount();

                auto& successorNode = nodes[successor.state];
                auto newCost = successor.actionCost + currentNode->g;

                if (successorNode == nullptr) {
                    // New state discovered
                    const Node tempSuccessorNode(currentNode,
                            successor.state,
                            successor.action,
                            newCost,
                            newCost + domain.heuristic(successor.state));

                    successorNode = nodePool.construct(std::move(tempSuccessorNode));
                    //                    LOG(INFO) << "addToOpen(NEW): " + successorNode->toString();
                    openList.push(*successorNode);
                } else if (successorNode->g > newCost) {
                    // Better path found to an existing state
                    successorNode->parent = currentNode;
                    successorNode->action = successor.action;
                    successorNode->g = newCost;
                    successorNode->f = newCost + domain.heuristic(successor.state);

                    openList.update(*successorNode);
                } else {
                    // The new path is not better than the existing
                }
            }
        }

        return std::vector<Action>();
    }

private:
    class Node {
    public:
        Node(Node* parent, const State state, Action action, Cost g, Cost f)
                : parent{parent}, state{state}, action{std::move(action)}, g{g}, f{f} {
        }

        unsigned long hash() const {
            return state->hash();
        }

        std::string toString() const {
            std::ostringstream stream;
            stream << "s: " << state << " g: " << g <<  " h: " << f - g << " f: " << f << " a: " << action << " p: ";
            if (parent == nullptr) {
                stream << "None";
            } else {
                stream << parent->state;
            }
            return stream.str();
        }

        bool operator==(const Node& node) const {
            return state == node.state;
        }

        mutable unsigned int index;
        Node* parent;
        const State state;
        Action action;
        Cost g;
        Cost f;
    };

    static int fValueComparator(const Node& lhs, const Node& rhs) {
        if (lhs.f < rhs.f)
            return -1;
        if (lhs.f > rhs.f)
            return 1;
        if (lhs.g > rhs.g)
            return -1;
        if (lhs.g < rhs.g)
            return 1;
        return 0;
    }

    const Domain& domain;
    PriorityQueue<Node> openList;
    std::unordered_map<State, Node*, typename metronome::Hasher<State>> nodes;
    boost::object_pool<Node> nodePool{1, 100000000};
};
}

#endif // METRONOME_ASTAR_HPP
