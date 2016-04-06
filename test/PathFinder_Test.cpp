#include <limits.h>
#include <iostream>
#include "gtest\gtest.h"
#include <chrono>
#include <thread>
#include <fstream>
#include "../source/include/PathFinder.hpp"

#ifdef DEBUG
struct IntCoord {
public:
	IntCoord():x{0},y{0}{}
	IntCoord(int x, int y):x{x},y{y}{}
	int x, y;
	bool operator==(const IntCoord &lhs) const {
		return x == lhs.x && y == lhs.y;
	}
};
namespace std {
	template<>
	struct hash<IntCoord> {
		std::size_t operator()(const IntCoord& coord) const {
			return std::hash<int>()(coord.x)
				   ^ (std::hash<int>()(coord.y) << (sizeof(int) / 2));
		}
	};
}
// test function, to be removed
// prints out a .pgm (portable grey map) image of the map and it's path
void printMapWithPath(std::ostream &out, Map &map, std::unordered_set<IntCoord> path) {
	out << "P3" << std::endl;
	out << map.sizeX << " " << map.sizeY << " 2" << std::endl;
	for (int i1 = 0; i1 < map.sizeY; i1++) {
		for (int i2 = 0; i2 < map.sizeX; i2++) {
			int groundVal = 1 - map.map[i1][i2];
			if (path.find(IntCoord(i1, i2)) != path.end()) {
				out << "2 " << groundVal << " " << groundVal << " ";
			} else {
				out << groundVal*2 << " " << groundVal*2 << " " << groundVal*2 << " ";
			}
		}
		out << std::endl;
	}
}
#endif

bool equal(const std::vector<Coordinate> &lhs, const std::vector<Coordinate> &rhs){
	EXPECT_EQ(lhs.size(), rhs.size());
	if (lhs.size() != rhs.size()) return false;
	int size = lhs.size();
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
	int result = RUN_ALL_TESTS();
	if (result != 0) {
		return result;
	}
	/*
	int mapX = 50, mapY = 50;
	Map map{mapX, mapY};
	PathFinder pathFinder{map, {.5, .5}};
	std::vector<Coordinate> path;
	std::cout << (pathFinder.get_path_to_coordinate({5.0f, 5.0f}, {mapX - 5.0f, mapY - 5.0f}, path) ? "found path"
	                                                                                                : "did not find path") <<
	std::endl;
#ifdef DEBUG
	std::unordered_set<IntCoord> intPath;
#endif
	for (Coordinate &coord : path) {
		std::cout << coord << std::endl;
#ifdef DEBUG
		intPath.emplace(coord.x, coord.y);
#endif
	}
	std::cout.flush();

#ifdef DEBUG
	std::ofstream ofs{"path.pgm"};
	printMapWithPath(ofs, map, intPath);
	ofs.flush();
#else
	map.printMap();
#endif
	*/
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	return 0;
}