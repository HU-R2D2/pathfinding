//
// Created by chiel on 30-3-16.
//

#ifndef R2D2_PATHFINDING_PATHFINDER_HPP
#define R2D2_PATHFINDING_PATHFINDER_HPP


#include <vector>
#include "Dummy.hpp"
#include "Astar.hpp"

//value will be replaced by a function in the interface later, but it has not yet been specified in the technote
#define ROBOT_SIZE 1

class Coordinate {
public:
	float x, y;
};

//! interface for a pathfinder module
/*!
 * computes a path between two points on a map
 */
class PathFinder {
public:
	//! Constructor
	/*!
	 * \param map Reference to the world map
	 * \param robotSize Reference to the robot size
	 */
	PathFinder(Map &map, Coordinate robotBox);

	//! Returns a path between two points
	/*!
	 * This function computes a path from start to goal,
	 * based on the map and robot size given in the constructor
	 *
	 * \param start The start coordinate
	 * \param goal The goal coordinate
	 * \param path Vector where the path need to be written to
	 * \return If it was able to find a path
	 */
	virtual bool get_path_to_coordinate(
			Coordinate start,
			Coordinate goal,
			std::vector<Coordinate> &path);

	class CoordNode : public Node<CoordNode> {
	public:
		CoordNode(PathFinder &pathFinder, Coordinate coord,
		          Coordinate &startCoord, float g = std::numeric_limits<float>::infinity(), CoordNode *parent = nullptr);

		virtual bool operator==(const CoordNode &lhs) const override;
		virtual std::vector<CoordNode> getAvailableNodes() override;

		PathFinder *pathFinder; // variables need to be pointers to be able to use assignment
		Coordinate coord, *startNodeCoord;
	};

private:
	Map &map;
	Coordinate robotBox;

	bool canVisit(Coordinate pos);
	static bool overlaps(Coordinate c1, Coordinate c2);
	static float getHeuristic(Coordinate dist); // will be replaced with an offset

	std::vector<CoordNode> getPath(CoordNode &start);
};

namespace std {
	template<>
	struct hash<PathFinder::CoordNode> {
		std::size_t operator()(const PathFinder::CoordNode& node) const {

			return std::hash<float>()(node.coord.x)
			         ^ (std::hash<float>()(node.coord.y) << (sizeof(float) / 2));
		}
	};
}

#endif //R2D2_PATHFINDING_PATHFINDER_HPP
