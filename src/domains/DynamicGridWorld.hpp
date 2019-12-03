#pragma once

#import "GridWorld.hpp"

#import "../../dependencies/Eigen/Dense"

#import <cstdlib>

namespace metronome {

template <std::size_t OBSTACLE_COUNT, std::size_t DOMAIN_SIZE>
class DynamicGridWorld {
 public:
  using Action = typename GridWorld::Action;
  using CollisionVector = typename Eigen::MatrixXf;
  using DomainVector = typename Eigen::MatrixXf;
  using DomainMatrix = typename Eigen::MatrixXf;
  using Cost = double;

  class State {
   private:
    GridWorld::State internalState;

    // Vector that describes the cumulative collision probability with each
    // obstacle in order.
    CollisionVector collisionVector;
    double collisionProbability = 0;
    std::size_t time = 0;
  };

  DynamicGridWorld(const Configuration& configuration, std::istream& input)
      : gridWorld(configuration, input) {
    // Initialize initial distribution
    std::vector<DomainVector> initialObstacleDistributions;
    initialObstacleDistributions.reserve(OBSTACLE_COUNT);

    for (std::size_t i = 0; i < OBSTACLE_COUNT; ++i) {
      DomainMatrix transitionMatrix = DomainMatrix::Random(DOMAIN_SIZE, DOMAIN_SIZE);

      for (int rowIndex = 0; rowIndex < transitionMatrix.rows(); ++rowIndex) {
        transitionMatrix.row(i).normalize();
      }

      obstacleTransitionMatrices.push_back(std::move(transitionMatrix));

      DomainVector initialDistribution = DomainVector::Random(1, DOMAIN_SIZE);
      initialDistribution.normalize();

      initialObstacleDistributions.push_back(std::move(initialDistribution));
    }

    timestampedObstacleDistribution.push_back(
        std::move(initialObstacleDistributions));

  }

  bool isGoal(const State& state) const {
    return gridWorld.isGoal(state.internalState);
  }

  Cost heuristic(const State& state) const {
    // TODO calculate heuristic
    return 0;
  }

  std::vector<SuccessorBundle<DynamicGridWorld>> successors(
      const State& state) const {
    auto internalSuccessors = gridWorld.successors(state.internalState);

    std::vector<SuccessorBundle<DynamicGridWorld>> successors;
    successors.reserve(internalSuccessors.size());

    for (auto& internalSuccessor : internalSuccessors) {
      // TODO create dynamic grid successor
    }

    return successors;
  }

  void expandObstacleDistributionHorizon() const {
    std::vector<DomainVector> nextObstacleDistribution;
    nextObstacleDistribution.reserve(OBSTACLE_COUNT);

    const auto& currentObstacleDistribution =
        timestampedObstacleDistribution.back();

    for (std::size_t i = 0; i < OBSTACLE_COUNT; ++i) {
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
};
}  // namespace metronome
