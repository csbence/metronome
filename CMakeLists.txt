cmake_minimum_required(VERSION 3.2)
project(Metronome)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

set(SOURCE_FILES
        src/main.cpp
        src/algorithms/AStar.hpp
        src/algorithms/LssLrtaStar.hpp
        src/algorithms/FHat.hpp
        src/algorithms/OfflinePlanner.hpp
        src/algorithms/OnlinePlanner.hpp
        src/algorithms/Planner.hpp
        src/domains/GridWorld.hpp
        src/domains/Domain.hpp
        src/domains/SlidingTilePuzzle.hpp
        src/domains/TestDomain.hpp
        src/domains/Traffic.hpp
        src/domains/SuccessorBundle.hpp
        src/utils/PriorityQueue.hpp
        src/utils/Hasher.hpp
        src/utils/Visualizer.hpp
        src/utils/Location2D.hpp
        src/experiment/termination/TimeTerminationChecker.hpp
        src/experiment/PlanManager.hpp
        src/experiment/OfflinePlanManager.hpp
        src/experiment/RealTimePlanManager.hpp
        src/experiment/Configuration.hpp
        src/experiment/Result.hpp
        src/experiment/ConfigurationExecutor.hpp
        src/MetronomeException.hpp
        )

set(TEST_FILES
        test/test.cpp
        test/utils/PriorityQueueTest.hpp
        test/domains/GridWorldTest.hpp
        test/algorithms/AStarTest.hpp
        test/algorithms/LssLrtaStarTest.hpp
        src/utils/TimeMeasurement.hpp
        test/experiment/TimeTerminationCheckerTest.hpp test/domains/TrafficTest.hpp)

add_executable(Metronome ${SOURCE_FILES})
add_executable(MetronomeTest ${TEST_FILES})

find_package(Boost 1.55.0 COMPONENTS thread system REQUIRED)

include_directories(
        ${Boost_INCLUDE_DIRS}
        ${Boost_LIBRARY_DIRS}
        dependencies
        src
)

target_link_libraries(Metronome ${Boost_LIBRARIES})
target_link_libraries(MetronomeTest ${Boost_LIBRARIES})
