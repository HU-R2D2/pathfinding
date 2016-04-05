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
	EXPECT_EQ(lhs.size(), rhs.size()) << "size";
	if (lhs.size() != rhs.size()) return false;
	int size = lhs.size();
	for (int i = 0; i < size; i++){
		EXPECT_EQ(lhs[i].x, rhs[i].x) << "content.x @ " << i;
		if (lhs[i].x != rhs[i].x) return false;
		EXPECT_EQ(lhs[i].y, rhs[i].y) << "content.y @ " << i;
		if (lhs[i].y != rhs[i].y) return false;
	}
	return true;
}

TEST(Pathfinding, Constructor){
	Map map(50, 50, false);
	Coordinate robotBox(1, 1);
	PathFinder pf(map, robotBox); 
	
	Coordinate start(0, 0);
	Coordinate goal(49, 49);
	std::vector<Coordinate> path;
	ASSERT_TRUE(pf.get_path_to_coordinate(start, goal, path)) << "path to coordinate";
	ASSERT_FALSE(path.empty()) << "path empty";
}

TEST(Pathfinding, not_Existing_Begin){
	Map map(50, 50, false);
	Coordinate robotBox(1, 1);
	PathFinder pf(map, robotBox);

	Coordinate start(-1, -1);
	Coordinate goal(49, 49);
	std::vector<Coordinate> path;
	ASSERT_FALSE(pf.get_path_to_coordinate(start, goal, path)) << "path to coordinate";
	ASSERT_TRUE(path.empty()) << "path empty";
}

TEST(Pathfinding, not_Existing_End){
	Map map(50, 50, false);
	Coordinate robotBox(1, 1);
	PathFinder pf(map, robotBox);

	Coordinate start(0, 0);
	Coordinate goal(50, 50);
	std::vector<Coordinate> path;
	ASSERT_FALSE(pf.get_path_to_coordinate(start, goal, path)) << "path to coordinate";
	ASSERT_TRUE(path.empty()) << "path empty";
}

TEST(Pathfinding, without_Obstacles){
	Map map(50, 50, false);
	Coordinate robotBox(1, 1);
	PathFinder pf(map, robotBox);

	Coordinate start(0, 0);
	Coordinate goal(49, 49);
	std::vector<Coordinate> path;
	ASSERT_TRUE(pf.get_path_to_coordinate(start, goal, path)) << "path to coordinate";
	ASSERT_FALSE(path.empty()) << "path empty";
}


TEST(Pathfinding, with_Obstacles){
	Map map(50, 50);
	Coordinate robotBox(1, 1);
	PathFinder pf(map, robotBox);

	Coordinate start(0, 0);
	Coordinate goal(49, 49);
	std::vector<Coordinate> path;
	ASSERT_TRUE(pf.get_path_to_coordinate(start, goal, path)) << "path to coordinate";
	ASSERT_FALSE(path.empty()) << "path empty";
	// when function didn't found a path, make sure there wasn't a path, otherwise try again.
}

TEST(Pathfinding, Consistent){
	Map map(50, 50);
	Coordinate robotBox(1, 1);
	PathFinder pf(map, robotBox);

	Coordinate start(0, 0);
	Coordinate goal(49, 49);
	std::vector<Coordinate> path;
	ASSERT_TRUE(pf.get_path_to_coordinate(start, goal, path)) << "path to coordinate";
	ASSERT_FALSE(path.empty()) << "path empty";
	std::vector<Coordinate> currentpath = path;
	ASSERT_TRUE(pf.get_path_to_coordinate(start, goal, path)) << "path to coordinate";
	ASSERT_TRUE(equal(currentpath, path));
}

TEST(Pathfinding, robot_Size){
	// size 0
	Map map(50, 50);
	Coordinate robotBox(0, 0);
	PathFinder pf(map, robotBox);

	Coordinate start(0, 0);
	Coordinate goal(49, 49);
	std::vector<Coordinate> path;
	ASSERT_TRUE(pf.get_path_to_coordinate(start, goal, path)) << "path to coordinate";
	ASSERT_TRUE(path.empty()) << "path empty";

	// size 10
	robotBox.x = robotBox.y = 10;
	PathFinder pf1(map, robotBox);
	ASSERT_TRUE(pf1.get_path_to_coordinate(start, goal, path)) << "path to coordinate";
	ASSERT_FALSE(path.empty()) << "path empty";
}

TEST(Pathfinding, obstacle_On_Begin){
	// 
	Map map(50, 50);
	Coordinate robotBox(0, 0);
	PathFinder pf(map, robotBox);

	Coordinate start(0, 0);
	Coordinate goal(49, 49);
	std::vector<Coordinate> path;
	ASSERT_TRUE(pf.get_path_to_coordinate(start, goal, path)) << "path to coordinate";
	ASSERT_TRUE(path.empty()) << "path empty";

}

TEST(Pathfinding, obstacle_On_End){
	//
	Map map(50, 50);
	Coordinate robotBox(0, 0);
	PathFinder pf(map, robotBox);

	Coordinate start(0, 0);
	Coordinate goal(49, 49);
	std::vector<Coordinate> path;
	ASSERT_TRUE(pf.get_path_to_coordinate(start, goal, path)) << "path to coordinate";
	ASSERT_TRUE(path.empty()) << "path empty";

}

int main(int argc, char **argv) {
	std::cout << "Tests" << std::endl;

	testing::InitGoogleTest(&argc, argv);
	int result = RUN_ALL_TESTS();

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

	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	return 0;
}