//
// Created by chiel on 30-3-16.
//

#include "Dummy.hpp"

Map::Map(int x, int y, bool obstacles = true)
{
	sizeX = x, sizeY = y;
	if (obstacles){
		for (int i1 = 0; i1 < x; i1++) {
			for (int i2 = 0; i2 < y; i2++) {
				map[i1][i2] = rand() % 3;
			}
		}
	}
	else {
		for (int i1 = 0; i1 < x; i1++) {
			for (int i2 = 0; i2 < y; i2++) {
				map[i1][i2] = 0;
			}
		}
	}
}

void Map::printMap()
{
	for (int i1 = 0; i1 < sizeX; i1++) {
		for (int i2 = 0; i2 < sizeY; i2++) {
			std::cout << map[i1][i2];
		}
		std::cout << std::endl;
	}
}

bool Map::hasObstacle(float x, float y, float sizeX, float sizeY)
{
	for (int i1 = 0; i1 < sizeX; i1++) {
		for (int i2 = 0; i2 < sizeY; i2++) {
			if (map[int(x+i1)][int(y+i2)] == 1) {
				return true;
			}
		}
	}
	return false;
}

bool Map::hasPassable(float x, float y, float sizeX, float sizeY)
{
	for (int i1 = 0; i1 < sizeX; i1++) {
		for (int i2 = 0; i2 < sizeY; i2++) {
			if (map[int(x + i1)][int(y + i2)] == 0) {
				return true;
			}
		}
	}
	return false;
}