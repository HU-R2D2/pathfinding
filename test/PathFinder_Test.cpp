#include <iostream>
#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include <fstream>
#include "../source/include/Dummy.hpp"
#include "../source/include/PathFinder.hpp"

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

std::tuple<bool, Map> test_until_true(int mapX, int mapY, r2d2::Translation robotBox,
                                    r2d2::Coordinate start, r2d2::Coordinate goal,
                                    std::vector<r2d2::Coordinate> &path) {
    for (int i = 0; i < MAX_TRIES; i++) {
        Map map(mapX, mapY);
        PathFinder pf(map, robotBox);

        if (pf.get_path_to_coordinate(start, goal, path)) {
            return std::tuple<bool, Map>{true, map};
        }
    }
    return std::tuple<bool, Map>{false, Map{}};
}

TEST(PathFinder, constructor) {
    Map map(50, 50, 0);
    r2d2::Translation robotBox{.5 * r2d2::Length::METER, .5 * r2d2::Length::METER,
                         0 * r2d2::Length::METER};
    PathFinder pf(map, robotBox);

    r2d2::Coordinate start{.5 * r2d2::Length::METER, .5 * r2d2::Length::METER, 0 * r2d2::Length::METER};
    r2d2::Coordinate goal{49.5 * r2d2::Length::METER, 49.5 * r2d2::Length::METER,
                    0 * r2d2::Length::METER};
    std::vector<r2d2::Coordinate> path;
    ASSERT_TRUE(pf.get_path_to_coordinate(start, goal, path))
                                << start << " " << goal;
    ASSERT_FALSE(path.empty());

}

TEST(PathFinder, not_existing_begin) {
    Map map(50, 50, 0);
    r2d2::Translation robotBox{.5 * r2d2::Length::METER, .5 * r2d2::Length::METER,
                         0 * r2d2::Length::METER};
    PathFinder pf(map, robotBox);

    r2d2::Coordinate start{-1 * r2d2::Length::METER, -1 * r2d2::Length::METER, 0 * r2d2::Length::METER};
    r2d2::Coordinate goal{49.5 * r2d2::Length::METER, 49.5 * r2d2::Length::METER,
                    0 * r2d2::Length::METER};
    std::vector<r2d2::Coordinate> path;
    ASSERT_FALSE(pf.get_path_to_coordinate(start, goal, path))
                                << start << " " << goal;
    ASSERT_TRUE(path.empty());
}

TEST(PathFinder, not_existing_end) {
    Map map(50, 50, 0);
    r2d2::Translation robotBox{.5 * r2d2::Length::METER, .5 * r2d2::Length::METER, 0 *
                                                                 r2d2::Length::METER};
    PathFinder pf(map, robotBox);

    r2d2::Coordinate start{.5 * r2d2::Length::METER, .5 * r2d2::Length::METER, 0 *
                                                             r2d2::Length::METER};
    r2d2::Coordinate goal{50 * r2d2::Length::METER, 50 * r2d2::Length::METER, 0 *
                                                            r2d2::Length::METER};
    std::vector<r2d2::Coordinate> path;
    ASSERT_FALSE(pf.get_path_to_coordinate(start, goal, path))
                                << start << " " << goal;
    ASSERT_TRUE(path.empty());
}

TEST(PathFinder, without_obstacles) {
    Map map(50, 50, 0);
    r2d2::Translation robotBox{.5 * r2d2::Length::METER, .5 * r2d2::Length::METER,
                         0 * r2d2::Length::METER};
    PathFinder pf(map, robotBox);

    r2d2::Coordinate start{.5 * r2d2::Length::METER, .5 * r2d2::Length::METER, 0 * r2d2::Length::METER};
    r2d2::Coordinate goal{49.5 * r2d2::Length::METER, 49.5 * r2d2::Length::METER,
                    0 * r2d2::Length::METER};
    std::vector<r2d2::Coordinate> path;
    ASSERT_TRUE(pf.get_path_to_coordinate(start, goal, path))
                                << start << " " << goal;
    ASSERT_FALSE(path.empty());
}

TEST(PathFinder, with_obstacles) {
    r2d2::Translation robotBox{.5 * r2d2::Length::METER, .5 * r2d2::Length::METER,
                         0 * r2d2::Length::METER};
    r2d2::Coordinate start{.5 * r2d2::Length::METER, .5 * r2d2::Length::METER, 0 * r2d2::Length::METER};
    r2d2::Coordinate goal{49.5 * r2d2::Length::METER, 49.5 * r2d2::Length::METER,
                    0 * r2d2::Length::METER};
    std::vector<r2d2::Coordinate> path;
    ASSERT_TRUE(std::get<0>(test_until_true(50, 50, robotBox, start, goal, path)))
                                << start << " " << goal;;
    ASSERT_FALSE(path.empty());
}

TEST(PathFinder, consistent) {
    r2d2::Translation robotBox{.5 * r2d2::Length::METER, .5 * r2d2::Length::METER,
                         0 * r2d2::Length::METER};
    r2d2::Coordinate start{.5 * r2d2::Length::METER, .5 * r2d2::Length::METER, 0 * r2d2::Length::METER};
    r2d2::Coordinate goal{49.5 * r2d2::Length::METER, 49.5 * r2d2::Length::METER,
                    0 * r2d2::Length::METER};
    std::vector<r2d2::Coordinate> path;
    std::tuple<bool, Map> result{
            test_until_true(50, 50, robotBox, start, goal, path)
    };
    ASSERT_TRUE(std::get<0>(result)) << start << " " << goal << " first time";
    ASSERT_FALSE(path.empty()) << "path empty";
    std::vector<r2d2::Coordinate> currentpath = path;
    PathFinder p2{std::get<1>(result), robotBox};
    ASSERT_TRUE(p2.get_path_to_coordinate(start, goal, path))
                                << start << " " << goal << " second time";
    ASSERT_TRUE(equal(currentpath, path));
}

TEST(PathFinder, robot_size) {
    Map map{100, 100, 0};
    // size 0
    r2d2::Translation robotBox{0 * r2d2::Length::METER, 0 * r2d2::Length::METER,
                         0 * r2d2::Length::METER};
    PathFinder pf(map, robotBox);

    r2d2::Coordinate start{10.5 * r2d2::Length::METER, 10.5 * r2d2::Length::METER, 0 *
                                                                 r2d2::Length::METER};
    r2d2::Coordinate goal{89.5 * r2d2::Length::METER, 89.5 * r2d2::Length::METER,
                    0 * r2d2::Length::METER};
    std::vector<r2d2::Coordinate> path;
    ASSERT_FALSE(pf.get_path_to_coordinate(start, goal, path))
                                << start << " " << goal << " robot with size 0";
    ASSERT_TRUE(path.empty());

    // size 5
    r2d2::Translation robotBox2{5 * r2d2::Length::METER, 5 * r2d2::Length::METER,
                         0 * r2d2::Length::METER};
    PathFinder pf1(map, robotBox2);
    ASSERT_TRUE(pf1.get_path_to_coordinate(start, goal, path))
                                << start << " " << goal << " robot with size 5";
    ASSERT_FALSE(path.empty());
}

TEST(PathFinder, obstacle_on_begin) {
    std::vector<std::vector<int>> vector = make_map(1, 50, 50);
    vector[0][0] = 1;
    Map map(vector);
    r2d2::Translation robotBox{.5 * r2d2::Length::METER, .5 * r2d2::Length::METER,
                         0 * r2d2::Length::METER};
    PathFinder pf(map, robotBox);

    r2d2::Coordinate start{.5 * r2d2::Length::METER, .5 * r2d2::Length::METER, 0 * r2d2::Length::METER};
    r2d2::Coordinate goal{49.5 * r2d2::Length::METER, 49.5 * r2d2::Length::METER,
                    0 * r2d2::Length::METER};
    std::vector<r2d2::Coordinate> path;
    ASSERT_FALSE(pf.get_path_to_coordinate(start, goal, path))
                                << start << " " << goal;
    ASSERT_TRUE(path.empty());
}

TEST(PathFinder, obstacle_on_end) {
    std::vector<std::vector<int>> vector = make_map(1, 50, 50);
    vector[49][49] = 1;
    Map map(vector);
    r2d2::Translation robotBox{.5 * r2d2::Length::METER, .5 * r2d2::Length::METER,
                         0 * r2d2::Length::METER};
    PathFinder pf(map, robotBox);

    r2d2::Coordinate start{.5 * r2d2::Length::METER, .5 * r2d2::Length::METER, 0 * r2d2::Length::METER};
    r2d2::Coordinate goal{49.5 * r2d2::Length::METER, 49.5 * r2d2::Length::METER,
                    0 * r2d2::Length::METER};
    std::vector<r2d2::Coordinate> path;
    ASSERT_FALSE(pf.get_path_to_coordinate(start, goal, path))
                                << start << " " << goal;
    ASSERT_TRUE(path.empty());
}

TEST(PathFinder, float) {
    Map map(make_map(2, 50, 50));
    r2d2::Translation robotBox{.5 * r2d2::Length::METER, .5 * r2d2::Length::METER,
                         0 * r2d2::Length::METER};
    PathFinder pf(map, robotBox);
    r2d2::Coordinate start{.56 * r2d2::Length::METER, .52 * r2d2::Length::METER, 0 *
                                                               r2d2::Length::METER};
    r2d2::Coordinate goal{49.2 * r2d2::Length::METER, 49.2 * r2d2::Length::METER,
                    0 * r2d2::Length::METER};
    std::vector<r2d2::Coordinate> path;
    ASSERT_TRUE(pf.get_path_to_coordinate(start, goal, path))
                                << start << " " << goal;
    ASSERT_FALSE(path.empty());
}

TEST(PathFinder, same_begin_as_end) {
    Map map(make_map(1, 50, 50));
    r2d2::Translation robotBox{.5 * r2d2::Length::METER, .5 * r2d2::Length::METER,
                         0 * r2d2::Length::METER};
    PathFinder pf(map, robotBox);

    r2d2::Coordinate start{1 * r2d2::Length::METER, 1 * r2d2::Length::METER, 0 * r2d2::Length::METER};
    r2d2::Coordinate goal{1 * r2d2::Length::METER, 1 * r2d2::Length::METER,
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
            cornerSqueezeMap.push_back(current);
        }
    }
    Map map(cornerSqueezeMap);
    r2d2::Translation robotBox{.5 * r2d2::Length::METER, .5 * r2d2::Length::METER,
                         0 * r2d2::Length::METER};
    PathFinder pf(map, robotBox);

    r2d2::Coordinate start{.5 * r2d2::Length::METER, .5 * r2d2::Length::METER, 0 * r2d2::Length::METER};
    r2d2::Coordinate goal{49.5 * r2d2::Length::METER, 49.5 * r2d2::Length::METER,
                    0 * r2d2::Length::METER};
    std::vector<r2d2::Coordinate> path;
    ASSERT_FALSE(pf.get_path_to_coordinate(start, goal, path))
                                << start << " " << goal;
    ASSERT_TRUE(path.empty());
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}