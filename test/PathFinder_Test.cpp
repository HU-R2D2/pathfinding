//! \addtogroup 0007 Pathfinding
//! \brief A pathfinding module
//!
//! A pathfinding module that can be used in the R2D2 project.
//! The module is currently based on the A star algorithm.
//!
//! \file   PathFinder_Test.hpp
//! \author Ole Achterberg 1651981
//! \author Jasper Schoenmaker 1661818
//! \author Chiel Douwes 1666311
//! \date   Created: 01-04-2016
//! \brief  gtest unit tests
//!
//! all the gtest units test are defined in this file
//!
//! \copyright Copyright © 2016, HU University of Applied Sciences Utrecht.
//! All rights reserved.
//!
//! License: newBSD
//!
//! Redistribution and use in source and binary forms,
//! with or without modification, are permitted provided that
//! the following conditions are met:
//! - Redistributions of source code must retain the above copyright notice,
//!   this list of conditions and the following disclaimer.
//! - Redistributions in binary form must reproduce the above copyright notice,
//!   this list of conditions and the following disclaimer in the documentation
//!   and/or other materials provided with the distribution.
//! - Neither the name of the HU University of Applied Sciences Utrecht
//!   nor the names of its contributors may be used to endorse or promote
//!   products derived from this software without specific prior written
//!   permission.
//!
//! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//! "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
//! BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
//! AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
//! IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
//! BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//! CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
//! PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
//! OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
//! WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
//! OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
//! EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// ~< HEADER_VERSION 2016 04 12 >~

#include <iostream>
#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include <fstream>
#include "../source/include/Dummy.hpp"
#include "../source/include/AStarPathFinder.hpp"
#include "../../sharedobjects/source/include/LockingSharedObject.hpp"

bool equal(const std::vector<r2d2::Coordinate> &lhs,
           const std::vector<r2d2::Coordinate> &rhs) {
    EXPECT_EQ(lhs.size(), rhs.size());
    if (lhs.size() != rhs.size()) return false;
    int size = int(lhs.size());
    for (int i = 0; i < size; i++) {
        EXPECT_EQ(lhs[i].get_x() / r2d2::Length::METER,
                  rhs[i].get_x() / r2d2::Length::METER);
        if (lhs[i].get_x() / r2d2::Length::METER !=
            rhs[i].get_x() / r2d2::Length::METER)
            return false;
        EXPECT_EQ(lhs[i].get_y() / r2d2::Length::METER,
                  rhs[i].get_y() / r2d2::Length::METER);
        if (lhs[i].get_y() / r2d2::Length::METER !=
            rhs[i].get_y() / r2d2::Length::METER)
            return false;
    }
    return true;
}

std::vector<std::vector<int>> make_map(int pathSize, int x, int y) {
    std::vector<std::vector<int>> map;
    for (int i = 0; i < x; i++) {
        std::vector<int> current;
        for (int j = 0; j < y; j++) {
            if (i - pathSize <= j && i + pathSize > j) {
                current.push_back(0);
            }
            else {
                current.push_back(1);
            }
        }
        map.push_back(current);
    }
    return map;
}

#define MAX_TRIES 10000 // can be scaled down if it takes too much processing

std::tuple<bool, r2d2::Dummy> test_until_true(int mapX, int mapY,
                                            r2d2::Translation robotBox,
                                            r2d2::Coordinate start,
                                            r2d2::Coordinate goal,
                                            std::vector<r2d2::Coordinate> &path) {
    for (int i = 0; i < MAX_TRIES; i++) {
        r2d2::Dummy map(mapX, mapY);
        LockingSharedObject<r2d2::ReadOnlyMap> sharedMap{map};
        r2d2::AStarPathFinder pf{sharedMap, {{}, robotBox}};

        if (pf.get_path_to_coordinate(start, goal, path)) {
            return std::tuple<bool, r2d2::Dummy>{true, map};
        }
    }
    return std::tuple<bool, r2d2::Dummy>{false, r2d2::Dummy{}};
}

TEST(PathFinder, constructor) {
    r2d2::Dummy map(50, 50, 0);
    r2d2::Translation robotBox{.5 * r2d2::Length::METER, .5 * r2d2::Length::METER,
                         0 * r2d2::Length::METER};
    LockingSharedObject<r2d2::ReadOnlyMap> sharedMap{map};
    r2d2::AStarPathFinder pf(sharedMap, {{}, robotBox});

    r2d2::Coordinate start{.5 * r2d2::Length::METER, .5 * r2d2::Length::METER, 0 * r2d2::Length::METER};
    r2d2::Coordinate goal{49.5 * r2d2::Length::METER, 49.5 * r2d2::Length::METER,
                    0 * r2d2::Length::METER};
    std::vector<r2d2::Coordinate> path;
    ASSERT_TRUE(pf.get_path_to_coordinate(start, goal, path))
                                << start << " " << goal;
    ASSERT_FALSE(path.empty());

}

TEST(PathFinder, not_existing_begin) {
    r2d2::Dummy map(50, 50, 0);
    r2d2::Translation robotBox{.5 * r2d2::Length::METER,
                               .5 * r2d2::Length::METER,
                               0 * r2d2::Length::METER};
    LockingSharedObject<r2d2::ReadOnlyMap> sharedMap{map};
    r2d2::AStarPathFinder pf(sharedMap, {{}, robotBox});

    r2d2::Coordinate start{-1 * r2d2::Length::METER, -1 * r2d2::Length::METER, 0 * r2d2::Length::METER};
    r2d2::Coordinate goal{49.5 * r2d2::Length::METER, 49.5 * r2d2::Length::METER,
                    0 * r2d2::Length::METER};
    std::vector<r2d2::Coordinate> path;
    ASSERT_FALSE(pf.get_path_to_coordinate(start, goal, path))
                                << start << " " << goal;
    ASSERT_TRUE(path.empty());
}

TEST(PathFinder, not_existing_end) {
    r2d2::Dummy map(50, 50, 0);
    r2d2::Translation robotBox{.5 * r2d2::Length::METER,
                               .5 * r2d2::Length::METER,
                               0 * r2d2::Length::METER};
    LockingSharedObject<r2d2::ReadOnlyMap> sharedMap{map};
    r2d2::AStarPathFinder pf(sharedMap, {{}, robotBox});

    r2d2::Coordinate start{.5 * r2d2::Length::METER,
                           .5 * r2d2::Length::METER,
                           0 * r2d2::Length::METER};
    r2d2::Coordinate goal{50 * r2d2::Length::METER,
                          50 * r2d2::Length::METER,
                          0 * r2d2::Length::METER};
    std::vector<r2d2::Coordinate> path;
    ASSERT_FALSE(pf.get_path_to_coordinate(start, goal, path))
                                << start << " " << goal;
    ASSERT_TRUE(path.empty());
}

TEST(PathFinder, without_obstacles) {
    r2d2::Dummy map(50, 50, 0);
    r2d2::Translation robotBox{.5 * r2d2::Length::METER,
                               .5 * r2d2::Length::METER,
                               0 * r2d2::Length::METER};
    LockingSharedObject<r2d2::ReadOnlyMap> sharedMap{map};
    r2d2::AStarPathFinder pf(sharedMap, {{}, robotBox});

    r2d2::Coordinate start{.5 * r2d2::Length::METER, .5 * r2d2::Length::METER, 0 * r2d2::Length::METER};
    r2d2::Coordinate goal{49.5 * r2d2::Length::METER, 49.5 * r2d2::Length::METER,
                    0 * r2d2::Length::METER};
    std::vector<r2d2::Coordinate> path;
    ASSERT_TRUE(pf.get_path_to_coordinate(start, goal, path))
                                << start << " " << goal;
    ASSERT_FALSE(path.empty());
}

TEST(PathFinder, with_obstacles) {
    r2d2::Translation robotBox{.5 * r2d2::Length::METER,
                               .5 * r2d2::Length::METER,
                               0 * r2d2::Length::METER};
    r2d2::Coordinate start{.5 * r2d2::Length::METER, .5 * r2d2::Length::METER,
                           0 * r2d2::Length::METER};
    r2d2::Coordinate goal{49.5 * r2d2::Length::METER, 49.5 * r2d2::Length::METER,
                          0 * r2d2::Length::METER};
    std::vector<r2d2::Coordinate> path;
    ASSERT_TRUE(
            std::get<0>(test_until_true(50, 50, robotBox, start, goal, path)))
                                << start << " " << goal;;
    ASSERT_FALSE(path.empty());
}

TEST(PathFinder, consistent) {
    r2d2::Translation robotBox{.5 * r2d2::Length::METER,
                               .5 * r2d2::Length::METER,
                               0 * r2d2::Length::METER};
    r2d2::Coordinate start{.5 * r2d2::Length::METER, .5 * r2d2::Length::METER,
                           0 * r2d2::Length::METER};
    r2d2::Coordinate goal{49.5 * r2d2::Length::METER, 49.5 * r2d2::Length::METER,
                          0 * r2d2::Length::METER};
    std::vector<r2d2::Coordinate> path;
    std::tuple<bool, r2d2::Dummy> result{
            test_until_true(50, 50, robotBox, start, goal, path)
    };
    ASSERT_TRUE(std::get<0>(result)) << start << " " << goal << " first time";
    ASSERT_FALSE(path.empty()) << "path empty";
    std::vector<r2d2::Coordinate> currentpath = path;
    LockingSharedObject<r2d2::ReadOnlyMap> sharedMap{std::get<1>(result)};
    r2d2::AStarPathFinder p2{sharedMap, {{}, robotBox}};
    ASSERT_TRUE(p2.get_path_to_coordinate(start, goal, path))
                                << start << " " << goal << " second time";
    ASSERT_TRUE(equal(currentpath, path));
}

TEST(PathFinder, robot_size) {
    r2d2::Dummy map{100, 100, 0};
    // size 0
    r2d2::Translation robotBox{0 * r2d2::Length::METER, 0 * r2d2::Length::METER,
                         0 * r2d2::Length::METER};
    LockingSharedObject<r2d2::ReadOnlyMap> sharedMap{map};
    r2d2::AStarPathFinder pf(sharedMap, {{}, robotBox});

    r2d2::Coordinate start{10.5 * r2d2::Length::METER,
                           10.5 * r2d2::Length::METER,
                           0 * r2d2::Length::METER};
    r2d2::Coordinate goal{89.5 * r2d2::Length::METER,
                          89.5 * r2d2::Length::METER,
                          0 * r2d2::Length::METER};
    std::vector<r2d2::Coordinate> path;
    ASSERT_FALSE(pf.get_path_to_coordinate(start, goal, path))
                                << start << " " << goal << " robot with size 0";
    ASSERT_TRUE(path.empty());

    // size 5
    r2d2::Translation robotBox2{5 * r2d2::Length::METER,
                                5 * r2d2::Length::METER,
                                0 * r2d2::Length::METER};
    LockingSharedObject<r2d2::ReadOnlyMap> sharedMap2{map};
    r2d2::AStarPathFinder pf1(sharedMap2, {{}, robotBox2});
    ASSERT_TRUE(pf1.get_path_to_coordinate(start, goal, path))
                                << start << " " << goal << " robot with size 5";
    ASSERT_FALSE(path.empty());
}

TEST(PathFinder, obstacle_on_begin) {
    std::vector<std::vector<int>> vector = make_map(1, 50, 50);
    vector[0][0] = 1;
    r2d2::Dummy map(vector);
    r2d2::Translation robotBox{.5 * r2d2::Length::METER,
                               .5 * r2d2::Length::METER,
                               0 * r2d2::Length::METER};
    LockingSharedObject<r2d2::ReadOnlyMap> sharedMap{map};
    r2d2::AStarPathFinder pf(sharedMap, {{}, robotBox});

    r2d2::Coordinate start{.5 * r2d2::Length::METER,
                           .5 * r2d2::Length::METER,
                           0 * r2d2::Length::METER};
    r2d2::Coordinate goal{49.5 * r2d2::Length::METER,
                          49.5 * r2d2::Length::METER,
                          0 * r2d2::Length::METER};
    std::vector<r2d2::Coordinate> path;
    ASSERT_FALSE(pf.get_path_to_coordinate(start, goal, path))
                                << start << " " << goal;
    ASSERT_TRUE(path.empty());
}

TEST(PathFinder, obstacle_on_end) {
    std::vector<std::vector<int>> vector = make_map(1, 50, 50);
    vector[49][49] = 1;
    r2d2::Dummy map(vector);
    r2d2::Translation robotBox{.5 * r2d2::Length::METER,
                               .5 * r2d2::Length::METER,
                               0 * r2d2::Length::METER};
    LockingSharedObject<r2d2::ReadOnlyMap> sharedMap{map};
    r2d2::AStarPathFinder pf(sharedMap, {{}, robotBox});

    r2d2::Coordinate start{.5 * r2d2::Length::METER,
                           .5 * r2d2::Length::METER,
                           0 * r2d2::Length::METER};
    r2d2::Coordinate goal{49.5 * r2d2::Length::METER,
                          49.5 * r2d2::Length::METER,
                          0 * r2d2::Length::METER};
    std::vector<r2d2::Coordinate> path;
    ASSERT_FALSE(pf.get_path_to_coordinate(start, goal, path))
                                << start << " " << goal;
    ASSERT_TRUE(path.empty());
}

TEST(PathFinder, float) {
    r2d2::Dummy map(make_map(2, 50, 50));
    r2d2::Translation robotBox{.5 * r2d2::Length::METER,
                               .5 * r2d2::Length::METER,
                               0 * r2d2::Length::METER};
    LockingSharedObject<r2d2::ReadOnlyMap> sharedMap{map};
    r2d2::AStarPathFinder pf(sharedMap, {{}, robotBox});
    r2d2::Coordinate start{.56 * r2d2::Length::METER,
                           .52 * r2d2::Length::METER,
                           0 * r2d2::Length::METER};
    r2d2::Coordinate goal{49.2 * r2d2::Length::METER,
                          49.2 * r2d2::Length::METER,
                          0 * r2d2::Length::METER};
    std::vector<r2d2::Coordinate> path;
    ASSERT_TRUE(pf.get_path_to_coordinate(start, goal, path))
                                << start << " " << goal;
    ASSERT_FALSE(path.empty());
}

TEST(PathFinder, same_begin_as_end) {
    r2d2::Dummy map(make_map(1, 50, 50));
    r2d2::Translation robotBox{.5 * r2d2::Length::METER,
                               .5 * r2d2::Length::METER,
                               0 * r2d2::Length::METER};
    LockingSharedObject<r2d2::ReadOnlyMap> sharedMap{map};
    r2d2::AStarPathFinder pf(sharedMap, {{}, robotBox});

    r2d2::Coordinate start{1 * r2d2::Length::METER,
                           1 * r2d2::Length::METER,
                           0 * r2d2::Length::METER};
    r2d2::Coordinate goal{1 * r2d2::Length::METER,
                          1 * r2d2::Length::METER,
                          0 * r2d2::Length::METER};
    std::vector<r2d2::Coordinate> path;
    ASSERT_TRUE(pf.get_path_to_coordinate(start, goal, path))
                                << start << " " << goal;
    ASSERT_TRUE(path.empty());
}

TEST(PathFinder, corner_squeezing) {
    std::vector<std::vector<int>> cornerSqueezeMap;
    for (int x = 0; x < 10; x++) {
        std::vector<int> current;
        for (int y = 0; y < 10; y++) {
            if (x == y) {
                current.push_back(1);
            } else {
                current.push_back(0);
            }
        }
        cornerSqueezeMap.push_back(current);
    }
    r2d2::Dummy map(cornerSqueezeMap);
    r2d2::Translation robotBox{.5 * r2d2::Length::METER,
                               .5 * r2d2::Length::METER,
                               0 * r2d2::Length::METER};
    LockingSharedObject<r2d2::ReadOnlyMap> sharedMap{map};
    r2d2::AStarPathFinder pf(sharedMap, {{}, robotBox});

    r2d2::Coordinate start{.5 * r2d2::Length::METER,
                           .5 * r2d2::Length::METER,
                           0 * r2d2::Length::METER};
    r2d2::Coordinate goal{49.5 * r2d2::Length::METER,
                          49.5 * r2d2::Length::METER,
                          0 * r2d2::Length::METER};
    std::vector<r2d2::Coordinate> path;
    ASSERT_FALSE(pf.get_path_to_coordinate(start, goal, path))
                                << start << " " << goal;
    ASSERT_TRUE(path.empty());
}

// we need a specialized integer coordinate class
// to be able to access the image coordinates
struct IntCoord {
public:
    IntCoord() : x{0}, y{0} { }

    IntCoord(int x, int y) : x{x}, y{y} { }

    int x, y;

    bool operator==(const IntCoord &lhs) const {
        return x == lhs.x && y == lhs.y;
    }
};
namespace std {
    template<>
    struct hash<IntCoord> {
        std::size_t operator()(const IntCoord &coord) const {
            return std::hash<int>()(coord.x)
                   ^ (std::hash<int>()(coord.y) << (sizeof(int) / 2));
        }
    };
}

// the pgm printer prints out a multiple of pixels for each tile in the tile
// map, this definition controls how much pixels are printed per tile
#define SCALE 4

// prints out a .pgm (portable grey map)
// image of the map and a calculated path overlaid onto it
// if you want to know how or why this code works
// you should search the pnm format
void printMapWithPath(std::ostream &out, r2d2::Dummy &map,
                      std::unordered_set<IntCoord> path) {
    out << "P3" << std::endl;
    out << map.sizeX * SCALE << " " << map.sizeY * SCALE << " 2" << std::endl;
    for (int i1 = 0; i1 < map.sizeY * SCALE; i1++) {
        for (int i2 = 0; i2 < map.sizeX * SCALE; i2++) {
            if (path.find(IntCoord(i2, i1)) != path.end()) {
                out << "2 0 0 ";
            } else {
                int groundVal = 1 - map.map[i1 / SCALE][i2 / SCALE];
                out << groundVal * 2 << " " << groundVal * 2 << " " <<
                groundVal * 2 << " ";
            }
        }
        out << std::endl;
    }
}

TEST(PathFinder, image_test) {
    // debugging code for visualisation of paths
    int mapX = 50, mapY = 50, mapCount = 0;
    bool done = false;
    while (!done) {
        r2d2::Dummy map = {mapX, mapY, 0.4f};

        LockingSharedObject<r2d2::ReadOnlyMap> sharedMap{map};
        r2d2::AStarPathFinder pathFinder = {sharedMap,
                                            {{},
                                             r2d2::Translation{0.5 * r2d2::Length::METER,
                                              0.5 * r2d2::Length::METER,
                                              0 * r2d2::Length::METER}}};
        std::vector<r2d2::Coordinate> path;
        done = pathFinder.get_path_to_coordinate(
                {5.5f * r2d2::Length::METER,
                 5.5f * r2d2::Length::METER,
                 0.0f * r2d2::Length::METER},
                {(mapX - 5.5f) * r2d2::Length::METER,
                 (mapY - 5.5f) * r2d2::Length::METER,
                 0.0f * r2d2::Length::METER},
                path);
        mapCount++;

        if (done) {
            std::unordered_set<IntCoord> intPath;
            for (r2d2::Coordinate &coord : path) {
                intPath.emplace(coord.get_x() / r2d2::Length::METER * SCALE,
                                coord.get_y() / r2d2::Length::METER * SCALE);
            }

            std::ofstream ofs{"path.pgm"};
            printMapWithPath(ofs, map, intPath);
            ofs.flush();

            std::cout << "searched " << mapCount << " maps" << std::endl;
        }
    }
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}