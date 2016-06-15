cmake_minimum_required(VERSION 2.8.9)
project(R2D2_pathfinding)
find_package (Threads)

file(GLOB SOURCES "source/include/*.hpp" "source/src/*.cpp" "example/*.cpp" "../adt/source/include/*.hpp" "../adt/source/src/*.cpp" "../sharedobjects/source/include/*.hpp" "../sharedobjects/source/src/*.cpp" )
file(GLOB SOURCES_GTEST "source/src/*.cpp" "source/include/*.hpp" 
		"test/*.cpp" "test/*.hpp" 
		"../adt/source/src/Coordinate.cpp" "../adt/source/include/Coordinate.hpp"
		"../adt/source/src/Length.cpp" "../adt/source/include/Length.hpp"
		"../adt/source/src/Translation.cpp" "../adt/source/include/Translation.hpp"
		"../adt/source/src/Box.cpp" "../adt/source/include/Box.hpp"
		"../sharedobjects/source/include/SharedObject.hpp"
		"../sharedobjects/source/include/LockingSharedObject.hpp"
		"../sharedobjects/source/include/NotCopyable.hpp"
		)
list(REMOVE_ITEM SOURCES_GTEST ${CMAKE_CURRENT_SOURCE_DIR}../adt/source/src/main.cpp)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -Wall")

include_directories(
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