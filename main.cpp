#include <iostream>
#include "gtest\gtest.h"
#include "PathFinder.hpp"


using namespace std;

TEST(test_case_name, test_name){
	ASSERT_EQ(1,0) << "length";
	ASSERT_TRUE(true == false) << "not equeal";
}

bool equal(const vector<Coordinate> &lhs, const vector<Coordinate> &rhs){
	EXPECT_EQ(lhs.size(), rhs.size()) << "size";
	if (lhs.size() != rhs.size()) return false;
	for (int i = 0; i < lhs.size(); ++i){
		EXPECT_EQ(lhs[i].x, rhs[i].x) << "content.x @ " << i;
		if (lhs[i].x != rhs[i].x) return false;
		EXPECT_EQ(lhs[i].y, rhs[i].y) << "content.y @ " << i;
		if (lhs[i].y != rhs[i].y) return false;
	}
	return true;
}

TEST(Pathfinding, Constructor){
	Map map(50, 50, false);
	Coordinate robotBox;
	robotBox.x = robotBox.y = 1;
	PathFinder pf(map, robotBox); 
	
	Coordinate start;
	start.x = start.y = 0;
	Coordinate goal;
	goal.x = goal.y = 49;
	vector<Coordinate> path;
	ASSERT_TRUE(pf.get_path_to_coordinate(start, goal, path)) << "path to coordinate";
	ASSERT_FALSE(path.empty()) << "path empty";
}

TEST(Pathfinding, not_Existing_Begin){
	Map map(50, 50, false);
	Coordinate robotBox;
	robotBox.x = robotBox.y = 1;
	PathFinder pf(map, robotBox);

	Coordinate start; 
	start.x = start.y = -1;
	Coordinate goal;
	goal.x = goal.y = 49;
	vector<Coordinate> path;
	ASSERT_FALSE(pf.get_path_to_coordinate(start, goal, path)) << "path to coordinate";
	ASSERT_TRUE(path.empty()) << "path empty";
}

TEST(Pathfinding, not_Existing_End){
	Map map(50, 50, false);
	Coordinate robotBox;
	robotBox.x = robotBox.y = 1;
	PathFinder pf(map, robotBox);

	Coordinate start;
	start.x = start.y = 0;
	Coordinate goal;
	goal.x = goal.y = 50;
	vector<Coordinate> path;
	ASSERT_FALSE(pf.get_path_to_coordinate(start, goal, path)) << "path to coordinate";
	ASSERT_TRUE(path.empty()) << "path empty";
}

TEST(Pathfinding, without_Obstacles){
	Map map(50, 50, false);
	Coordinate robotBox;
	robotBox.x = robotBox.y = 1;
	PathFinder pf(map, robotBox);

	Coordinate start;
	start.x = start.y = 0;
	Coordinate goal;
	goal.x = goal.y = 49;
	vector<Coordinate> path;
	ASSERT_TRUE(pf.get_path_to_coordinate(start, goal, path)) << "path to coordinate";
	ASSERT_FALSE(path.empty()) << "path empty";
}


TEST(Pathfinding, with_Obstacles){
	Map map(50, 50);
	Coordinate robotBox;
	robotBox.x = robotBox.y = 1;
	PathFinder pf(map, robotBox);

	Coordinate start;
	start.x = start.y = 0;
	Coordinate goal;
	goal.x = goal.y = 49;
	vector<Coordinate> path;
	ASSERT_TRUE(pf.get_path_to_coordinate(start, goal, path)) << "path to coordinate";
	ASSERT_FALSE(path.empty()) << "path empty";
	// when function didn't found a path, make sure there wasn't a path, otherwise try again.
}

TEST(Pathfinding, Consistent){
	Map map(50, 50);
	Coordinate robotBox;
	robotBox.x = robotBox.y = 1;
	PathFinder pf(map, robotBox);

	Coordinate start;
	start.x = start.y = 0;
	Coordinate goal;
	goal.x = goal.y = 49;
	vector<Coordinate> path;
	ASSERT_TRUE(pf.get_path_to_coordinate(start, goal, path)) << "path to coordinate";
	ASSERT_FALSE(path.empty()) << "path empty";
	vector<Coordinate> currentpath = path;
	ASSERT_TRUE(pf.get_path_to_coordinate(start, goal, path)) << "path to coordinate";
	ASSERT_TRUE(equal(currentpath, path));
}

TEST(Pathfinding, robot_Size){
	// size 0
	Map map(50, 50);
	Coordinate robotBox;
	robotBox.x = robotBox.y = 0;
	PathFinder pf(map, robotBox);

	Coordinate start;
	start.x = start.y = 0;
	Coordinate goal;
	goal.x = goal.y = 49;
	vector<Coordinate> path;
	ASSERT_TRUE(pf.get_path_to_coordinate(start, goal, path)) << "path to coordinate";
	ASSERT_TRUE(path.empty()) << "path empty";

	// size 10
	robotBox.x = robotBox.y = 10;
	pf = PathFinder(map, robotBox);
	ASSERT_TRUE(pf.get_path_to_coordinate(start, goal, path)) << "path to coordinate";
	ASSERT_FALSE(path.empty()) << "path empty";
}

TEST(Pathfinding, obstacle_On_Begin){
	// 
	Map map(50, 50);
	Coordinate robotBox;
	robotBox.x = robotBox.y = 0;
	PathFinder pf(map, robotBox);

	Coordinate start;
	start.x = start.y = 0;
	Coordinate goal;
	goal.x = goal.y = 49;
	vector<Coordinate> path;
	ASSERT_TRUE(pf.get_path_to_coordinate(start, goal, path)) << "path to coordinate";
	ASSERT_TRUE(path.empty()) << "path empty";

}

TEST(Pathfinding, obstacle_On_End){
	//
	Map map(50, 50);
	Coordinate robotBox;
	robotBox.x = robotBox.y = 0;
	PathFinder pf(map, robotBox);

	Coordinate start;
	start.x = start.y = 0;
	Coordinate goal;
	goal.x = goal.y = 49;
	vector<Coordinate> path;
	ASSERT_TRUE(pf.get_path_to_coordinate(start, goal, path)) << "path to coordinate";
	ASSERT_TRUE(path.empty()) << "path empty";

}

int main(int argc, char **argv) {
	cout << "Tests" << endl;

	testing::InitGoogleTest(&argc, argv);
	int result = RUN_ALL_TESTS();
	(void)result;

	return 0;
}