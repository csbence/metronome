#pragma once

#include <cstdlib>
#include <ostream>

#include "../../dependencies/Eigen/Dense"
#include "GridWorld.hpp"

namespace metronome {

class DynamicGridWorld {
 public:
  using Action = typename GridWorld::Action;
  using CollisionVector = typename Eigen::VectorXf;
  using DomainVector = typename Eigen::MatrixXf;
  using DomainMatrix = typename Eigen::MatrixXf;
  using Cost = double;

  class State {
   public:
    State(const GridWorld::State internalState,
          const CollisionVector collisionVector,
          double collisionProbability,
          size_t time)
        : internalState(internalState),
          collisionVector(std::move(collisionVector)),
          collisionProbability(collisionProbability),
          time(time) {}

    bool operator==(const State& rhs) const {
      return time == rhs.time && internalState == rhs.internalState &&
             collisionVector == rhs.collisionVector &&
             collisionProbability == rhs.collisionProbability;
    }
    bool operator!=(const State& rhs) const { return !(rhs == *this); }

    std::size_t hash() const {
      // This could be improved
      return internalState.hash();
    }

    friend std::ostream& operator<<(std::ostream& os, const State& state) {
      os << "internalState: " << state.internalState
         << " collisionVector: " << state.collisionVector
         << " collisionProbability: " << state.collisionProbability
         << " time: " << state.time;
      return os;
    }

    GridWorld::State internalState;

    // Vector that describes the cumulative collision probability with each
    // obstacle in order.
    CollisionVector collisionVector;
    double collisionProbability = 0;
    std::size_t time = 0;
  };

  DynamicGridWorld(const Configuration& configuration, std::istream& input)
      : gridWorld(configuration, input),
        obstacleCount(configuration.getLong("obstacleCount")),
        domainSize(gridWorld.getHeight() * gridWorld.getWidth()) {
    // Initialize initial distribution
    std::vector<DomainVector> initialObstacleDistributions;
    initialObstacleDistributions.reserve(obstacleCount);

    for (std::size_t i = 0; i < obstacleCount; ++i) {
      DomainMatrix transitionMatrix =
          DomainMatrix::Random(domainSize, domainSize).cwiseAbs();

      for (int rowIndex = 0; rowIndex < transitionMatrix.rows(); ++rowIndex) {
        transitionMatrix.row(rowIndex) /= transitionMatrix.row(rowIndex).sum();
      }

      obstacleTransitionMatrices.push_back(std::move(transitionMatrix));

      DomainVector initialDistribution =
          DomainVector::Random(1, domainSize).cwiseAbs();
      initialDistribution /= initialDistribution.sum();

      initialObstacleDistributions.push_back(std::move(initialDistribution));
    }

    timestampedObstacleDistribution.push_back(
        std::move(initialObstacleDistributions));
  }

  State getStartState() const {
    return State(gridWorld.getStartState(),
                 CollisionVector::Zero(obstacleCount),
                 0.0,
                 0);
  }

  bool isGoal(const State& state) const {
    return gridWorld.isGoal(state.internalState);
  }

  Cost heuristic(const State& state) const {
    // TODO calculate heuristic
    return 0;
  }

  std::vector<SuccessorBundle<DynamicGridWorld>> successors(
      const State& sourceState) const {
    auto internalSuccessors = gridWorld.successors(sourceState.internalState);

    std::vector<SuccessorBundle<DynamicGridWorld>> successors;
    successors.reserve(internalSuccessors.size());

    const auto successorTime = sourceState.time + 1;

    // Propagete the obstacles for one more step
    if (timestampedObstacleDistribution.size() <= successorTime) {
      expandObstacleDistributionHorizon();
    }

    if (timestampedObstacleDistribution.size() <= successorTime) {
      throw MetronomeException(
          "Logical error if obstacle distribution array is not large enough "
          "(size is " +
          std::to_string(timestampedObstacleDistribution.size()) +
          ", at T=" + std::to_string(successorTime) + ")");
    }

    const auto& obstacleDistributions =
        timestampedObstacleDistribution[successorTime];

    for (auto& internalSuccessor : internalSuccessors) {
      // Map the state to the vector representation
      const auto x = internalSuccessor.state.getX();
      const auto y = internalSuccessor.state.getY();
      const std::size_t locationIndex = y * gridWorld.getWidth() + x;

      CollisionVector independentCollisionVector(obstacleCount);

      for (std::size_t obstacleIndex = 0; obstacleIndex < obstacleCount;
           ++obstacleIndex) {
        // This could be vectorized but it would be super sparse
        // TODO Let's try it after the baseline
        float independentCollisionProbability =
            obstacleDistributions[obstacleIndex](locationIndex);

        independentCollisionVector[obstacleIndex] =
            independentCollisionProbability;
      }

      // prob_collision = (1 - collided) * collied_now) + collided
      // = p2 - p1p2 + p1
      const auto oneVector = CollisionVector::Ones(obstacleCount);
      auto collisionVector = sourceState.collisionVector.array() +
                             independentCollisionVector.array() -
                             sourceState.collisionVector.array() *
                                 independentCollisionVector.array();

      auto const collisionProbability =
          1 - (oneVector.array() - collisionVector.array()).prod();

      State successorState(internalSuccessor.state,
                           std::move(collisionVector),
                           collisionProbability,
                           successorTime);

      // The cost of the action is the collision probability increase
      double cost = collisionProbability - sourceState.collisionProbability;
      if (cost < 0) {
        throw MetronomeException("DGW: The probability of failure " +
                                 std::to_string(cost) + " must not decrease.");
      }

      // The successor bundle we create contains the state, the action, and the
      // cost
      successors.emplace_back(
          std::move(successorState), internalSuccessor.action, cost);
    }

    return successors;
  }

  void expandObstacleDistributionHorizon() const {
    std::vector<DomainVector> nextObstacleDistribution;
    nextObstacleDistribution.reserve(obstacleCount);

    const auto& currentObstacleDistribution =
        timestampedObstacleDistribution.back();

    for (std::size_t i = 0; i < obstacleCount; ++i) {
      nextObstacleDistribution.push_back(currentObstacleDistribution[i] *
                                         obstacleTransitionMatrices[i]);
    }

    timestampedObstacleDistribution.emplace_back(nextObstacleDistribution);
  }

  std::optional<State> transition(const State& sourceState,
                                  const Action& action) const {
    const auto internalTransitionState =
        gridWorld.transition(sourceState.internalState, action);

    if (not internalTransitionState) {
      return {};
    }

    // TODO
    return {sourceState};
  }

 private:
  // We store the probability distribution of each obstacle for each time
  // Q: Should the probability distributions of each obstacle merged to one
  // giant matrix?
  // Example: timestampedObstDist[time][obstacleId][state]
  mutable std::vector<std::vector<DomainVector>>
      timestampedObstacleDistribution;
  std::vector<DomainMatrix> obstacleTransitionMatrices;

  // GridWorld instance used as an agent model;
  const GridWorld gridWorld;
  const std::size_t obstacleCount;
  const std::size_t domainSize;
};
}  // namespace metronome
