cmake_minimum_required(VERSION 2.8.9)
project(R2D2_pathfinding)
find_package (Threads)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -Wall -Werror")

file(GLOB SOURCES "source/src/*.cpp" "source/include/*.hpp"
		"../adt/source/src/*.cpp" "../adt/source/include/*.hpp")
file(GLOB SOURCES_GTEST "source/src/*.cpp" "source/include/*.hpp" "test/*.cpp" "test/*.hpp"
		"../adt/source/src/*.cpp" "../adt/source/include/*.hpp")
list(REMOVE_ITEM SOURCES_GTEST ${CMAKE_CURRENT_SOURCE_DIR}/source/src/main.cpp)
list(REMOVE_ITEM SOURCES_GTEST ${CMAKE_CURRENT_SOURCE_DIR}../adt/source/src/main.cpp)

include_directories(
  ../deps/gtest-1.7.0/include
  ../deps/gtest-1.7.0)

link_directories(../deps/gtest-1.7.0/src)
ADD_LIBRARY(gtest ../deps/gtest-1.7.0/src/gtest-all.cc)
ADD_LIBRARY(gtest_main ../deps/gtest-1.7.0/src/gtest_main.cc)

add_executable(R2D2_pathfinding ${SOURCES})
add_executable(R2D2_pathfinding_gtest ${GTEST} ${SOURCES_GTEST})
target_link_libraries(R2D2_pathfinding_gtest gtest gtest_main ${CMAKE_THREAD_LIBS_INIT})