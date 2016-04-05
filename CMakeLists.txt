cmake_minimum_required(VERSION 3.4)
project(R2D2_pathfinding)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

set(SOURCE_FILES main.cpp Astar.cpp Astar.hpp Dummy.cpp Dummy.hpp PathFinder.cpp PathFinder.hpp)
add_executable(R2D2_pathfinding ${SOURCE_FILES})
target_link_libraries(R2D2_pathfinding gtest)