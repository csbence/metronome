cmake_minimum_required(VERSION 3.2)
project(Metronome)

# Ensure that we find all of our support CMake scripts. These are things like
# locating required libraries for addons, etc. Store them in modules/
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/modules/")

if (NOT APPLE AND NOT WIN32)
    set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++")
elseif(WIN32)
	set(CMAKE_EXE_LINKER_FLAGS "/STACK:0x2000000")
endif()

set(CXX clang++)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
if (WIN32 AND NOT MINGW)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall /bigobj")
else()
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -pedantic -Wextra")
endif(WIN32 AND NOT MINGW)

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
        src/experiment/PlanManager.hpp
        src/experiment/OfflinePlanManager.hpp
        src/experiment/RealTimePlanManager.hpp
        src/experiment/Configuration.hpp
        src/experiment/Result.hpp
        src/experiment/ConfigurationExecutor.hpp
        src/MetronomeException.hpp
        src/utils/File.hpp
        src/experiment/termination/ExpansionTerminationChecker.hpp
        src/utils/Statistic.hpp src/MemoryConfiguration.hpp
        src/utils/ObjectPool.hpp
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
find_package(X11)

include_directories(
        ${Boost_INCLUDE_DIRS}
        ${Boost_LIBRARY_DIRS}
        dependencies
        src
)

# Comment the following line out to disable Gephi (Stream Graph HTTP) visualization
add_definitions(-DSTREAM_GRAPH)

if (MINGW OR WIN32)
    target_link_libraries(Metronome ws2_32)
endif()
target_link_libraries(Metronome ${Boost_LIBRARIES} ${X11_LIBRARIES})
target_link_libraries(MetronomeTest ${Boost_LIBRARIES} ${X11_LIBRARIES})
