//
// Created by chiel on 30-3-16.
//

#ifndef R2D2_PATHFINDING_DUMMY_HPP
#define R2D2_PATHFINDING_DUMMY_HPP

#include <iostream>
#include <random>
#include <array>

class Map {
public:
	//0 is clear
	//1 is obstacle
	//2 is unexplored
	std::vector<std::vector<int>> map;
	int sizeX, sizeY;

	Map(int x = 100, int y = 100, bool obstacles = true);
	Map(std::vector < std::vector<int> > map);
	void printMap();
	bool hasObstacle(float x, float y, float sizeX, float sizeY);
	bool hasPassable(float x, float y, float sizeX, float sizeY);

private:
	std::mt19937_64 mersenne;

};


#endif //R2D2_PATHFINDING_DUMMY_HPP
