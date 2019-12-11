#pragma once

#import <cstdlib>
#include <ostream>

#import "../../dependencies/Eigen/Dense"
#import "GridWorld.hpp"

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

    std::size_t hash() {
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
          DomainMatrix::Random(domainSize, domainSize);

      for (int rowIndex = 0; rowIndex < transitionMatrix.rows(); ++rowIndex) {
        transitionMatrix.row(i).normalize();
      }

      obstacleTransitionMatrices.push_back(std::move(transitionMatrix));

      DomainVector initialDistribution = DomainVector::Random(1, domainSize);
      initialDistribution.normalize();

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
    if (timestampedObstacleDistribution.size() < successorTime) {
      expandObstacleDistributionHorizon();
    }

    const auto& obstacleDistributions =
        timestampedObstacleDistribution[successorTime];

    for (auto& internalSuccessor : internalSuccessors) {
      // Map the state to the vector representation
      auto x = internalSuccessor.state.getX();
      auto y = internalSuccessor.state.getY();
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

      // Calculate the cumulative collision probability: 1 - (1 - P1)(1 - P2)
      const auto oneVector = CollisionVector::Ones(obstacleCount);
      auto collisionVector =
          oneVector - (oneVector - sourceState.collisionVector) *
                          (oneVector - independentCollisionVector);

      // TODO Use obstacle probabilities instead of sum.
      const double collisionProbability = collisionVector.sum();

      State successorState(internalSuccessor.state,
                           std::move(collisionVector),
                           collisionProbability,
                           successorTime);

      // The cost of the action is the collision probability increase
      double cost = collisionProbability - sourceState.collisionProbability;
      if (cost < 0) {
        throw MetronomeException(
            "DGW: The probability of failure can't decrease.");
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
