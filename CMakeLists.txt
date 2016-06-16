cmake_minimum_required(VERSION 2.8.9)
project(R2D2_pathfinding)
find_package (Threads)

file(GLOB SOURCES
		source/include/*.hpp
		source/src/*.cpp
		example/*.cpp
		../map/source/src/MapInterface.cpp
		../adt/source/include/*.hpp
		../adt/source/src/*.cpp
		../sharedobjects/source/include/*.hpp
		../sharedobjects/source/src/*.cpp)

file(GLOB SOURCES_GTEST
		../adt/source/src/Length.cpp
		../adt/source/src/Coordinate.cpp
		../adt/source/src/Translation.cpp
		../adt/source/src/Box.cpp
		../map/source/src/MapInterface.cpp
		source/src/Dummy.cpp
		source/src/AStarPathFinder.cpp
		test/PathFinder_Test.cpp
		../sharedobjects/source/include/SharedObject.hpp
		../sharedobjects/source/include/LockingSharedObject.hpp
		../sharedobjects/source/include/NotCopyable.hpp)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -Wall")

include_directories(
		../map/source/include
		../adt/source/include
		../sharedobjects/source/include
		../deps/gtest-1.7.0/include
		../deps/gtest-1.7.0)


link_directories(../deps/gtest-1.7.0/src)
ADD_LIBRARY(gtest ../deps/gtest-1.7.0/src/gtest-all.cc)
ADD_LIBRARY(gtest_main ../deps/gtest-1.7.0/src/gtest_main.cc)

add_executable(R2D2_pathfinding_example ${SOURCES})
add_executable(R2D2_pathfinding_gtest ${GTEST} ${SOURCES_GTEST})
target_link_libraries(R2D2_pathfinding_gtest gtest gtest_main ${CMAKE_THREAD_LIBS_INIT})
