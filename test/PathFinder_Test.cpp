#include <iostream>
#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include <fstream>
#include "../source/include/PathFinder.hpp"

bool equal(const std::vector<Coordinate> &lhs, const std::vector<Coordinate> &rhs){
	EXPECT_EQ(lhs.size(), rhs.size());
	if (lhs.size() != rhs.size()) return false;
	int size = int(lhs.size());
	for (int i = 0; i < size; i++){
		EXPECT_EQ(lhs[i].x, rhs[i].x);
		if (lhs[i].x != rhs[i].x) return false;
		EXPECT_EQ(lhs[i].y, rhs[i].y);
		if (lhs[i].y != rhs[i].y) return false;
	}
	return true;
}

TEST(PathFinder, Constructor){
	Map map(50, 50, 0);
	Coordinate robotBox(1, 1);
	PathFinder pf(map, robotBox); 
	
	Coordinate start(0, 0);
	Coordinate goal(49, 49);
	std::vector<Coordinate> path;
	ASSERT_TRUE(pf.get_path_to_coordinate(start, goal, path)) << start << " " << goal;
	ASSERT_FALSE(path.empty());
	
}

TEST(PathFinder, not_Existing_Begin){
	Map map(50, 50, 0);
	Coordinate robotBox(1, 1);
	PathFinder pf(map, robotBox);

	Coordinate start(-1, -1);
	Coordinate goal(49, 49);
	std::vector<Coordinate> path;
	ASSERT_FALSE(pf.get_path_to_coordinate(start, goal, path)) << start << " " << goal;
	ASSERT_TRUE(path.empty());
}

TEST(PathFinder, not_Existing_End){
	Map map(50, 50, 0);
	Coordinate robotBox(1, 1);
	PathFinder pf(map, robotBox);

	Coordinate start(0, 0);
	Coordinate goal(50, 50);
	std::vector<Coordinate> path;
	ASSERT_FALSE(pf.get_path_to_coordinate(start, goal, path)) << start << " " << goal;
	ASSERT_TRUE(path.empty());
}

TEST(PathFinder, without_Obstacles){
	Map map(50, 50, 0);
	Coordinate robotBox(1, 1);
	PathFinder pf(map, robotBox);

	Coordinate start(0, 0);
	Coordinate goal(49, 49);
	std::vector<Coordinate> path;
	ASSERT_TRUE(pf.get_path_to_coordinate(start, goal, path)) << start << " " << goal;
	ASSERT_FALSE(path.empty());
}

#define MAX_TRIES 100000 // can be scaled down if it takes too much processing

std::tuple<bool, Map> testUntilTrue(int mapX, int mapY, Coordinate robotBox, Coordinate start, Coordinate goal,
	std::vector<Coordinate> &path) {
	for (int i = 0; i < MAX_TRIES; i++) {
		Map map(mapX, mapY);
		PathFinder pf(map, robotBox);

		if (pf.get_path_to_coordinate(start, goal, path)) {
			return std::tuple<bool, Map>{true, map};
		}
	}

	return std::tuple<bool, Map>{false, Map{}};
}

TEST(PathFinder, with_Obstacles){
    Coordinate robotBox(1, 1);
	Coordinate start(0, 0);
	Coordinate goal(49, 49);
	std::vector<Coordinate> path;
	ASSERT_TRUE(std::get<0>(testUntilTrue(50, 50, robotBox, start, goal, path))) << start << " " << goal;;
	ASSERT_FALSE(path.empty());
}

TEST(PathFinder, consistent){
	Coordinate robotBox(1, 1);
	Coordinate start(0, 0);
	Coordinate goal(49, 49);
	std::vector<Coordinate> path;
	std::tuple<bool, Map> result{ testUntilTrue(50, 50, robotBox, start, goal, path) };
	ASSERT_TRUE(std::get<0>(result)) << start << " " << goal << " first time";
	ASSERT_FALSE(path.empty()) << "path empty";
	std::vector<Coordinate> currentpath = path;
	PathFinder p2{ std::get<1>(result), robotBox };
	ASSERT_TRUE(p2.get_path_to_coordinate(start, goal, path)) << start << " " << goal << " second time";
	ASSERT_TRUE(equal(currentpath, path));
}

TEST(PathFinder, robot_Size){
	std::vector<std::vector<int>> vector;
	for (int i = 0; i < 100; i++) {
		std::vector<int> current;
		for (int j = 0; j < 100; j++) {
			if (i-6 < j && i+6 > j){
				current.push_back(0);
			}
			else {
				current.push_back(1);
			}
		}
		vector.push_back(current);
	}
	Map map(vector);
	// size 0
	Coordinate robotBox(0, 0);
	PathFinder pf(map, robotBox);

	Coordinate start(10, 10);
	Coordinate goal(90, 90);
	std::vector<Coordinate> path;
	ASSERT_FALSE(pf.get_path_to_coordinate(start, goal, path)) << start << " " << goal << " robot with size 0";
	ASSERT_TRUE(path.empty());

	// size 5
	robotBox.x = robotBox.y = 5;
	PathFinder pf1(map, robotBox);
	ASSERT_TRUE(pf1.get_path_to_coordinate(start, goal, path)) << start << " " << goal << " robot with size 5";
	ASSERT_FALSE(path.empty());
}

TEST(PathFinder, obstacle_On_Begin){
	std::vector<std::vector<int>> vector;
	for (int i = 0; i < 50; i++) {
		std::vector<int> current;
		for (int j = 0; j < 50; j++) {
			if ((i - 2 < j && i + 2 > j) && (i != 0 && j != 0)){
				current.push_back(0);
			}
			else {
				current.push_back(1);
			}
		}
		vector.push_back(current);
	}
	Map map(vector);
	//map.printMap();
	Coordinate robotBox(1, 1);
	PathFinder pf(map, robotBox);

	Coordinate start(0, 0);
	Coordinate goal(49, 49);
	std::vector<Coordinate> path;
	ASSERT_FALSE(pf.get_path_to_coordinate(start, goal, path)) << start << " " << goal;
	ASSERT_TRUE(path.empty());
}

TEST(PathFinder, obstacle_On_End){
	std::vector<std::vector<int>> vector;
	for (int i = 0; i < 50; i++) {
		std::vector<int> current;
		for (int j = 0; j < 50; j++) {
			if ( (i - 2 < j && i + 2 > j) && (i != 49 && j != 49) ){
				current.push_back(0);
			}
			else {
				current.push_back(1);
			}
		}
		vector.push_back(current);
	}
	Map map(vector);
	//map.printMap();
	Coordinate robotBox(1, 1);
	PathFinder pf(map, robotBox);

	Coordinate start(0, 0);
	Coordinate goal(49, 49);
	std::vector<Coordinate> path;
	ASSERT_FALSE(pf.get_path_to_coordinate(start, goal, path)) << start << " " << goal;
	ASSERT_TRUE(path.empty());
}

TEST(PathFinder, float){
	std::vector<std::vector<int>> vector;
	for (int i = 0; i < 50; i++) {
		std::vector<int> current;
		for (int j = 0; j < 50; j++) {
			if (i - 2 < j && i + 2 > j){
				current.push_back(0);
			}
			else {
				current.push_back(1);
			}
		}
		vector.push_back(current);
	}
	Map map(vector);
	Coordinate robotBox(1, 1);
	PathFinder pf(map, robotBox);

	Coordinate start(0.06, 0.02);
	Coordinate goal(49.2, 49.2);
	std::vector<Coordinate> path;
	ASSERT_TRUE(pf.get_path_to_coordinate(start, goal, path)) << start << " " << goal;
	ASSERT_FALSE(path.empty());
}

TEST(PathFinder, same_Begin_As_End){
	std::vector<std::vector<int>> vector;
	for (int i = 0; i < 50; i++) {
		std::vector<int> current;
		for (int j = 0; j < 50; j++) {
			if (i - 2 < j && i + 2 > j){
				current.push_back(0);
			}
			else {
				current.push_back(1);
			}
		}
		vector.push_back(current);
	}
	Map map(vector);
	Coordinate robotBox(1, 1);
	PathFinder pf(map, robotBox);

	Coordinate start(1, 1);
	Coordinate goal(1, 1);
	std::vector<Coordinate> path;
	ASSERT_TRUE(pf.get_path_to_coordinate(start, goal, path)) << start << " " << goal;
	ASSERT_TRUE(path.empty());
}

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}