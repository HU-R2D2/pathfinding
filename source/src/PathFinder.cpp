//
// Created by chiel on 30-3-16.
//

#include "../include/PathFinder.hpp"

PathFinder::PathFinder(Map &map, Coordinate robotBox):
		map(map),
		robotBox(robotBox) {
}

bool PathFinder::get_path_to_coordinate(Coordinate start, Coordinate goal, std::vector<Coordinate> &path) {
	// parent is at this point unknown for the start node, so construct it as unknown
	CoordNode endNode{*this, goal, start, 0}, startNode{*this, start, start};
	AStarSearch<CoordNode> search{endNode};
	//std::cout << start << " " << goal << std::endl;

	std::shared_ptr<CoordNode> foundStart = search.search(startNode);
	if (foundStart == nullptr) {
		return false;
	} else {
		std::vector<CoordNode> foundPath{getPath(foundStart)};
		path.clear();
		for (CoordNode &node : foundPath) {
			path.push_back(node.coord);
		}
		return true;
	}
}

PathFinder::CoordNode::CoordNode(
		PathFinder &pathFinder, Coordinate coord,
		Coordinate &startCoord, float g, std::weak_ptr<CoordNode> parent) :
		Node(g, PathFinder::getHeuristic(
				{startCoord.x - coord.x, // minimum distance to the search end
				 startCoord.y - coord.y}),
		     parent),
		pathFinder(pathFinder),
		coord(coord),
		startNodeCoord(startCoord) {
//	std::cout << "create node " << coord << " hash: " << std::hash<CoordNode>()(*this) << std::endl;
}

std::vector<PathFinder::CoordNode> PathFinder::CoordNode::getAvailableNodes(
		std::shared_ptr<PathFinder::CoordNode> self) {
	std::vector<CoordNode> children;
	for (int x = -1; x <= 1; x++) {
		for (int y = -1; y <= 1; y++) {
			if (x != 0 || y != 0) {
				// the grid will be relative to the end position of the search
				Coordinate childPos{coord.x + x, coord.y + y}; // this will be replaced with an adt constructor
				if (pathFinder.overlaps(childPos, startNodeCoord)) { //check whether the successor is the end node
					childPos = {startNodeCoord.x, startNodeCoord.y};
				}
				if (pathFinder.canVisit(childPos)) {
					children.push_back(
							CoordNode{pathFinder, childPos, startNodeCoord,
							          g + PathFinder::getHeuristic({float(x), float(y)}), // distance from the search begin
							          self});
				}
			}
		}
	}
	return children;
}

bool PathFinder::CoordNode::operator==(const PathFinder::CoordNode &lhs) const {
	return coord.x == lhs.coord.x && coord.y == lhs.coord.y;
}

bool PathFinder::canVisit(const Coordinate &pos) {
	return !map.hasObstacle(pos.x, pos.y, robotBox.x, robotBox.y);
}

bool PathFinder::overlaps(const Coordinate &c1, const Coordinate &c2) {
	return std::fabs(c1.x - c2.x) < robotBox.x / 2 && std::fabs(c1.y - c2.y) < robotBox.y / 2;
}

// define a constant as to speed the calculation up, in this case 10 digits is "good enough"
#define SQ_ROOT_2 1.414213562f

float PathFinder::getHeuristic(const Coordinate &dist) {
	// diagonal distance
	float xDist = dist.x < 0 ? -dist.x : dist.x;
	float yDist = dist.y < 0 ? -dist.y : dist.y;
	float shortDist, longDist;
	if (xDist < yDist) {
		shortDist = xDist;
		longDist = yDist;
	} else {
		shortDist = yDist;
		longDist = xDist;
	}
	return shortDist * SQ_ROOT_2 + (longDist - shortDist);
}

std::vector<PathFinder::CoordNode> PathFinder::getPath(std::shared_ptr<PathFinder::CoordNode> start) {
	std::shared_ptr<CoordNode> curNode = start;
	std::vector<CoordNode> path;
	while (!curNode->parent.expired()) {
		curNode = std::shared_ptr<CoordNode>(curNode->parent);
		path.emplace_back(*curNode);
	}
	return path;
}

