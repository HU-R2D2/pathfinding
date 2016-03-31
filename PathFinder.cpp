//
// Created by chiel on 30-3-16.
//

#include "PathFinder.hpp"

PathFinder::PathFinder(Map &map, Coordinate robotBox):
		map(map),
		robotBox(robotBox) {
}

bool PathFinder::get_path_to_coordinate(Coordinate start, Coordinate goal, std::vector<Coordinate> &path) {
	CoordNode endNode{*this, goal, start, 0, nullptr}, startNode{*this, start, start};
	AStarSearch<CoordNode> search{startNode};

	CoordNode *foundStart{search.search(startNode)};
	if (foundStart == nullptr) {
		return false;
	} else {
		std::vector<CoordNode> foundPath{getPath(*foundStart)};
		path.clear();
		for (CoordNode &node : foundPath) {
			path.push_back(node.coord);
		}
		return true;
	}
}

PathFinder::CoordNode::CoordNode(
		PathFinder &pathFinder, Coordinate coord,
		Coordinate &startCoord, float g, CoordNode *parent) :
		Node(g, PathFinder::getHeuristic(
				{startNodeCoord->x - coord.x, // minimum distance to the search end
				 startNodeCoord->y - coord.y}),
		     parent),
		pathFinder(&pathFinder),
		coord(coord),
		startNodeCoord(&startCoord) {
}

std::vector<PathFinder::CoordNode> PathFinder::CoordNode::getAvailableNodes() {
	std::vector<CoordNode> children;
	for (int x = -1; x <= 1; x++) {
		for (int y = -1; y <= 1; y++) {
			if (x != 0 || y != 0) {
				Coordinate childPos;
				// the grid will be relative to the end position of the search
				childPos = coord; // this will be replaced with an adt constructor
				childPos.x += x;
				childPos.y += y;
				if (PathFinder::overlaps(childPos, *startNodeCoord)) { //check whether the successor is the end node
					childPos = *startNodeCoord;
				}
				if (pathFinder->canVisit(childPos)) {
					children.push_back(
							CoordNode{*pathFinder, childPos, *startNodeCoord,
							          g + PathFinder::getHeuristic({float(x), float(y)}), // distance from the search begin
							          this});
				}
			}
		}
	}
	return children;
}

bool PathFinder::CoordNode::operator==(const PathFinder::CoordNode &lhs) const {
	return coord.x == lhs.coord.x && coord.y == lhs.coord.y;
}

bool PathFinder::canVisit(Coordinate pos) {
	return map.hasPassable(pos.x - ROBOT_SIZE, pos.y - ROBOT_SIZE, ROBOT_SIZE * 2, ROBOT_SIZE * 2)
			&& !map.hasObstacle(pos.x - ROBOT_SIZE, pos.y - ROBOT_SIZE, ROBOT_SIZE * 2, ROBOT_SIZE * 2);
}
bool PathFinder::overlaps(Coordinate c1, Coordinate c2) {
	return fabsf(c1.x - c2.x) < ROBOT_SIZE && fabsf(c1.y - c2.y) < ROBOT_SIZE;
}

// define a constant as to speed the calculation up, in this case 10 digits is "good enough"
#define SQ_ROOT_2 1.414213562f
float PathFinder::getHeuristic(Coordinate dist) {
	// diagonal distance
	dist.x = dist.x < 0 ? -dist.x : dist.x;
	dist.y = dist.y < 0 ? -dist.y : dist.y;
	float shortDist, longDist;
	if (dist.x < dist.y) {
		shortDist = dist.x;
		longDist = dist.y;
	} else {
		shortDist = dist.y;
		longDist = dist.x;
	}
	return shortDist * SQ_ROOT_2 + longDist;
}

std::vector<PathFinder::CoordNode> PathFinder::getPath(PathFinder::CoordNode &start) {
	CoordNode *curNode = &start;
	std::vector<CoordNode> path;
	while (curNode->parent != nullptr) {
		curNode = curNode->parent;
		path.emplace_back(*curNode);
	}
	return path;
}

