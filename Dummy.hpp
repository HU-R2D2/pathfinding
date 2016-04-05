//
// Created by chiel on 30-3-16.
//

#ifndef R2D2_PATHFINDING_DUMMY_HPP
#define R2D2_PATHFINDING_DUMMY_HPP


#include <iostream>

class Map {
public:
	//0 is clear
	//1 is obstacle
	//2 is unexplored
	int map[100][100];
	int sizeX = 100;
	int sizeY = 100;

	Map(int x, int y, bool obstacles = true);
	void printMap();
	bool hasObstacle(float x, float y, float sizeX, float sizeY);
	bool hasPassable(float x, float y, float sizeX, float sizeY);

};


#endif //R2D2_PATHFINDING_DUMMY_HPP
