cmake_minimum_required(VERSION 2.8.9)
project(R2D2_pathfinding)
find_package (Threads)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -Wall -Werror")

file(GLOB SOURCES_GTEST
		../adt/source/src/Length.cpp
		../adt/source/src/Coordinate.cpp
		../adt/source/src/Translation.cpp
		../adt/source/src/Box.cpp
		../adt/source/src/Box.hpp
		../map/source/src/MapInterface.cpp
		source/src/Dummy.cpp
		source/src/AStarPathFinder.cpp
		test/PathFinder_Test.cpp
		../sharedobjects/source/include/SharedObject.hpp
		../sharedobjects/source/include/LockingSharedObject.hpp
		../sharedobjects/source/include/NotCopyable.hpp)

include_directories(
		../deps/gtest-1.7.0/include
		../deps/gtest-1.7.0)

link_directories(../deps/gtest-1.7.0/src)
ADD_LIBRARY(gtest ../deps/gtest-1.7.0/src/gtest-all.cc)
ADD_LIBRARY(gtest_main ../deps/gtest-1.7.0/src/gtest_main.cc)

add_executable(R2D2_pathfinding_gtest ${GTEST} ${SOURCES_GTEST})
target_link_libraries(R2D2_pathfinding_gtest gtest gtest_main ${CMAKE_THREAD_LIBS_INIT})