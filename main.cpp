#include <iostream>
#include <chrono>
#include <thread>
#include <fstream>
#include "PathFinder.hpp"

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

int main() {
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