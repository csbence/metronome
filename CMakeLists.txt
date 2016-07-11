cmake_minimum_required(VERSION 3.2)
project(Metronome)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

set(SOURCE_FILES
        src/main.cpp
        src/algorithm/AStar.hpp
        src/algorithm/LssLrtaStar.hpp
        src/domains/VacuumWorld.hpp
        src/domains/VacuumWorldIO.hpp
        src/util/PriorityQueue.hpp
        src/util/Hasher.hpp
        src/domains/TestDomain.hpp
        src/domains/SuccessorBundle.hpp src/experiment/termination/TimeTerminationChecker.hpp src/domains/Vehicle.hpp src/algorithm/Planner.hpp)


set(TEST_FILES
        test/test.cpp
        test/util/PriorityQueueTest.hpp
        test/domains/VacuumWorldTest.hpp
        test/algorithm/AStarTest.hpp
        test/algorithm/LssLrtaStarTest.hpp)

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
