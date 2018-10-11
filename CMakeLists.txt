cmake_minimum_required(VERSION 3.2)
project(Metronome)

# Ensure that we find all of our support CMake scripts. These are things like
# locating required libraries for addons, etc. Store them in modules/
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/modules/")

if (NOT APPLE)
    set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++")
endif(NOT APPLE)

set(CXX clang++)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -pedantic -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}  ${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_CXX_FLAGS_DEBUG  "${CMAKE_CXX_FLAGS_DEBUG}  ${CMAKE_CXX_FLAGS} -g -O0")

set(SOURCE_FILES
        src/Metronome.cpp
        src/algorithms/AStar.hpp
        src/algorithms/ClusterRts.hpp
        src/algorithms/LssLrtaStar.hpp
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
        src/utils/Hash.hpp
        src/visualization/Visualizer.hpp
        src/utils/Location2D.hpp
        src/experiment/termination/TimeTerminationChecker.hpp
        src/experiment/Experiment.hpp
        src/experiment/OfflinePlanManager.hpp
        src/experiment/RealTimeExperiment.hpp
        src/experiment/Configuration.hpp
        src/experiment/Result.hpp
        src/experiment/ConfigurationExecutor.hpp
        src/MetronomeException.hpp
        src/utils/File.hpp
        src/experiment/termination/ExpansionTerminationChecker.hpp
        src/utils/Statistic.hpp
        src/utils/ObjectPool.hpp
        src/MemoryConfiguration.hpp
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

include_directories(SYSTEM
        ${Boost_INCLUDE_DIRS}
        ${Boost_LIBRARY_DIRS}
        dependencies
)

include_directories(src)

# The following options can enable algorithms and domains.
# These options have a large impact on the compilation speed.

# Comment the following line out to disable Gephi (Stream Graph HTTP) visualization
add_definitions(-DSTREAM_GRAPH)

# Select the possible algorithms.
#add_definitions(-DENABLE_A_STAR)
#add_definitions(-DENABLE_LSS_LRTA_STAR)
add_definitions(-DENABLE_CLUSTER_RTS)

# Select the possible domains
add_definitions(-DENABLE_GRID_WORLD)
#add_definitions(-DENABLE_SLIDING_TILE_PUZZLE)
#add_definitions(-DENABLE_TRAFFIC_WORLD)

target_link_libraries(Metronome ${Boost_LIBRARIES})
target_link_libraries(MetronomeTest ${Boost_LIBRARIES})
