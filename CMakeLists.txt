cmake_minimum_required(VERSION 3.5)
project(Metronome)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

set(SOURCE_FILES
        src/main.cpp
        src/util/PriorityQueue.hpp
        )
set(TEST_FILES
        test/test.cpp
        test/util/PriorityQueueTest.hpp)

add_executable(Metronome ${SOURCE_FILES})
add_executable(MetronomeTest ${TEST_FILES})

find_package(Boost 1.59.0 COMPONENTS thread system REQUIRED)

include_directories(
        ${Boost_INCLUDE_DIRS}
        ${Boost_LIBRARY_DIRS}
        dependencies/
)

target_link_libraries(Metronome ${Boost_LIBRARIES})
target_link_libraries(MetronomeTest ${Boost_LIBRARIES})