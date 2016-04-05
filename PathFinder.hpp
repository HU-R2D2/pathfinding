//
// Created by chiel on 30-3-16.
//

#ifndef R2D2_PATHFINDING_PATHFINDER_HPP
#define R2D2_PATHFINDING_PATHFINDER_HPP


#include <vector>
#include <cmath>
#include "Dummy.hpp"
#include "Astar.hpp"

class Coordinate {
public:
	Coordinate(float x, float y) :
			x{x},
			y{y} {
	}
	
	float x, y;

	friend std::ostream &operator<<(std::ostream &lhs, const Coordinate &rhs) {
		return lhs << "(" << rhs.x << ", " << rhs.y << ")";
	}
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
		          Coordinate &startCoord, float g = std::numeric_limits<float>::infinity(),
		          std::weak_ptr<CoordNode> parent = {});

		virtual bool operator==(const CoordNode &lhs) const override;

		virtual std::vector<CoordNode> getAvailableNodes(std::shared_ptr<CoordNode> self) override;

		PathFinder &pathFinder; // variables need to be pointers to be able to use assignment
		Coordinate coord, &startNodeCoord;

		friend std::ostream &operator<<(std::ostream &lhs, const CoordNode &rhs) {
			return lhs << "(" << rhs.coord << ", " << rhs.g << ", " << rhs.h << ", " << rhs.f << ")";
		}
	};

private:
	Map &map;
	Coordinate robotBox;

	bool canVisit(const Coordinate &pos);

	bool overlaps(const Coordinate &c1, const Coordinate &c2);

	static float getHeuristic(const Coordinate &dist); // will be replaced with an offset

	std::vector<CoordNode> getPath(std::shared_ptr<CoordNode> start);
};

namespace std {

	template<>
	struct hash<Coordinate> {
		std::size_t operator()(const Coordinate &coord) const {

			return std::hash<float>()(coord.x)
			       ^ (std::hash<float>()(coord.y) << (sizeof(float) / 2));
		}
	};

	template<>
	struct hash<PathFinder::CoordNode> {
		std::size_t operator()(const PathFinder::CoordNode& node) const {

			return std::hash<Coordinate>()(node.coord);
		}
	};
}

#endif //R2D2_PATHFINDING_PATHFINDER_HPP
