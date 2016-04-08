#include <iostream>
#include <fstream>
#include <unordered_set>
#include <chrono>
#include <thread>
#include "../include/Dummy.hpp"
#include "../include/PathFinder.hpp"

struct IntCoord {
public:
	IntCoord() : x{0}, y{0} { }

	IntCoord(int x, int y) : x{x}, y{y} { }

	int x, y;

	bool operator==(const IntCoord &lhs) const {
		return x == lhs.x && y == lhs.y;
	}
};
namespace std {
	template<>
	struct hash<IntCoord> {
		std::size_t operator()(const IntCoord &coord) const {
			return std::hash<int>()(coord.x)
			       ^ (std::hash<int>()(coord.y) << (sizeof(int) / 2));
		}
	};
}
// prints out a .pgm (portable grey map) image of the map and it's path
#define SCALE 4

void printMapWithPath(std::ostream &out, Map &map,
                      std::unordered_set<IntCoord> path) {
	out << "P3" << std::endl;
	out << map.sizeX * SCALE << " " << map.sizeY * SCALE << " 2" << std::endl;
	for (int i1 = 0; i1 < map.sizeY * SCALE; i1++) {
		for (int i2 = 0; i2 < map.sizeX * SCALE; i2++) {
			int groundVal = 1 - map.map[i1 / SCALE][i2 / SCALE];
			if (path.find(IntCoord(i1, i2)) != path.end()) {
				out << "2 " << 0 << " " << 0 << " ";
			} else {
				out << groundVal * 2 << " " << groundVal * 2 << " " <<
				groundVal * 2 << " ";
			}
		}
		out << std::endl;
	}
}

int main() {
	// debugging code for visualisation of paths
	int mapX = 300, mapY = 300, mapCount = 0;
	bool done = false;
	while (!done) {

		Map map = {mapX, mapY, 0.3f};
		PathFinder pathFinder = {map, 
				{0.5 * Length::METER, 
				0.5 * Length::METER,
				0 * Length::METER}};
		std::vector<Coordinate> path;
		done = pathFinder.get_path_to_coordinate(
				{5.5f * Length::METER,
				5.5f * Length::METER,
				0.0f * Length::METER},
		        {(mapX - 5.5f)* Length::METER,
				(mapY - 5.5f) * Length::METER,
				0.0f * Length::METER},
		        path);
		mapCount++;

		if (done) {
			std::unordered_set<IntCoord> intPath;
			for (Coordinate &coord : path) {
				std::cout << coord << std::endl;
				intPath.emplace(coord.get_x() / Length::METER * SCALE, coord.get_y() / Length::METER * SCALE);
			}
			std::cout.flush();

			std::ofstream ofs{"path.pgm"};
			printMapWithPath(ofs, map, intPath);
			ofs.flush();

			std::cout << "searched " << mapCount << " maps" << std::endl;
		}
	}

	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	return 0;
}