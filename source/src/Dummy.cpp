//
// Created by chiel on 30-3-16.
//

#include "../include/Dummy.hpp"

Map::Map(int x, int y, float obstacles) :
		map{},
		mersenne{(unsigned long) (time(0))} {
	map.reserve((unsigned long) (x));
	sizeX = x, sizeY = y;
	for (int i1 = 0; i1 < x; i1++) {
		map.emplace_back();
		map[i1].reserve((unsigned long) (y));
		for (int i2 = 0; i2 < y; i2++) {
			map[i1].emplace_back();
			map[i1][i2] = std::uniform_real_distribution<float>{}(mersenne) < obstacles ? 1 : 0;
		}
	}
}


Map::Map(std::vector<std::vector<int> > map) :
		map{map} {
}

void Map::printMap() {
	for (int i1 = 0; i1 < sizeX; i1++) {
		for (int i2 = 0; i2 < sizeY; i2++) {
			std::cout << map[i1][i2];
		}
		std::cout << std::endl;
	}
}

bool Map::hasObstacle(float x, float y, float boxSizeX, float boxSizeY) {
	for (int i1 = 0; i1 < boxSizeX; i1++) {
		for (int i2 = 0; i2 < boxSizeY; i2++) {
			int getX = int(x + i1), getY = int(y + i2);
			if (getX < 0 || getX >= sizeX ||
			    getY < 0 || getY >= sizeY ||
			    map[getX][getY] == 1 || map[getX][getY] == 2) {
				return true;
			}
		}
	}
	return false;
}

bool Map::hasPassable(float x, float y, float sizeX, float sizeY) {
	for (int i1 = 0; i1 < sizeX; i1++) {
		for (int i2 = 0; i2 < sizeY; i2++) {
			int getX = int(x + i1), getY = int(y + i2);
			if (getX < 0 && getX >= sizeX &&
			    getY < 0 && getY >= sizeY &&
			    map[getX][getY] == 0) {
				return true;
			}
		}
	}
	return false;
}