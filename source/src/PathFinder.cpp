//
// Created by chiel on 30-3-16.
//

#include "../include/PathFinder.hpp"

PathFinder::PathFinder(Map &map, Translation robotBox):
		map(map),
		robotBox(robotBox) {
}

bool PathFinder::get_path_to_coordinate(Coordinate start, Coordinate goal, std::vector<Coordinate> &path) {
	// check for the goal node being at the same coordinate as the start node
	if (overlaps(start, goal)) {
		path.clear();
		return true;
	}
	// do a check for end node accessibility before starting the search
	if (!canTravel(goal, goal)) {
		return false;
	}

	// parent is at this point unknown for the start node, so construct it as unknown
	CoordNode endNode{*this, goal, start, 0 * Length::METER}, startNode{*this, start, start};
	AStarSearch<CoordNode> search{endNode};

	std::shared_ptr<CoordNode> foundStart = search.search(startNode);
	if (foundStart == nullptr) {
		return false;
	} else {
		std::vector<CoordNode> foundPath{getPath(foundStart)};
		path.clear();
		for (CoordNode &node : foundPath) {
			path.push_back(node.coord);
		}
		smoothPath(path, start);
		return true;
	}
}

PathFinder::CoordNode::CoordNode(
		PathFinder &pathFinder, Coordinate coord,
		Coordinate &startCoord, Length g, std::weak_ptr<CoordNode> parent) :
		Node{g, PathFinder::getHeuristic(startCoord - coord),
		     parent},
		pathFinder(pathFinder),
		coord(coord),
		startNodeCoord(startCoord) {
//	std::cout << "create node " << coord << " hash: " << std::hash<CoordNode>()(*this) << std::endl;
}

std::vector<PathFinder::CoordNode> PathFinder::CoordNode::getAvailableNodes(
		std::shared_ptr<PathFinder::CoordNode> &self) {
	std::vector<CoordNode> children;
	for (int x = -1; x <= 1; x++) {
		for (int y = -1; y <= 1; y++) {
			if (x != 0 || y != 0) {
				// the grid will be relative to the end position of the search
				Coordinate childPos{
						coord + (Translation{
								x * pathFinder.robotBox.get_x(),
						        y * pathFinder.robotBox.get_y(),
						        0 * Length::METER
						} / SQUARES_PER_ROBOT)};
				//check whether the successor is the end node
				if (pathFinder.overlaps(childPos, startNodeCoord)) {
					childPos = {startNodeCoord};
				}
				// canTravel is used so that it can be ensured that there is no
				// obstacle in the path
				if (pathFinder.canTravel(coord, childPos)) {
					children.push_back(
							CoordNode{pathFinder, childPos, startNodeCoord,
							          g + PathFinder::getHeuristic(
									          childPos - coord
							          ), // distance from the search begin
							          self});
				}
			}
		}
	}
	return children;
}

bool PathFinder::CoordNode::operator==(const PathFinder::CoordNode &lhs) const {
	return (coord - lhs.coord).get_length() / Length::METER == 0;
}

bool PathFinder::canTravel(const Coordinate &from, const Coordinate &to) {
	Coordinate minCoord{
			(from.get_x() < to.get_x() ? from.get_x() : to.get_x()),
			(from.get_y() < to.get_y() ? from.get_y() : to.get_y()),
			0 * Length::METER};
	Translation size{(Coordinate{
			(from.get_x() > to.get_x() ? from.get_x() : to.get_x()),
			(from.get_y() > to.get_y() ? from.get_y() : to.get_y()),
			0 * Length::METER} - minCoord) + robotBox};
	return !map.hasObstacle(minCoord - (robotBox / 2), size);
}

bool PathFinder::overlaps(const Coordinate &c1, const Coordinate &c2) {
	Translation diff = c1 - c2;
	return (diff.get_x() < 0 * Length::METER ?
	        0 * Length::METER - diff.get_x() :
	        diff.get_x()
	       ) < robotBox.get_x() / 2 &&
	       (diff.get_y() < 0 * Length::METER ?
	        0 * Length::METER - diff.get_y() :
	        diff.get_y()
	       ) < robotBox.get_y() / 2;
}

// define a constant as to speed the calculation up,
// in this case 10 digits is "good enough"
#define SQ_ROOT_2 1.414213562f
Length PathFinder::getHeuristic(Translation coord) {
	// diagonal distance
	Length xDist = (coord.get_x() < 0 * Length::METER) ? (0 * Length::METER - coord.get_x()) : coord.get_x();
	Length yDist = (coord.get_y() < 0 * Length::METER) ? (0 * Length::METER - coord.get_y()) : coord.get_y();
	Length shortDist, longDist;
	if (xDist < yDist) {
		shortDist = xDist;
		longDist = yDist;
	} else {
		shortDist = yDist;
		longDist = xDist;
	}
	return (shortDist * SQ_ROOT_2) + (longDist - shortDist);
}

std::vector<PathFinder::CoordNode> PathFinder::getPath(
		std::shared_ptr<PathFinder::CoordNode> start) {
	std::shared_ptr<CoordNode> curNode = start;
	std::vector<CoordNode> path;
	while (!curNode->parent.expired()) {
		curNode = std::shared_ptr<CoordNode>(curNode->parent);
		path.emplace_back(*curNode);
	}
	return path;
}

void PathFinder::smoothPath(std::vector<Coordinate> &path, Coordinate start) {
	Coordinate lastPos = start;
	auto it = path.begin();
	auto current = it++;
	while (it != path.end()) {
		if (canTravel(lastPos, *it)) {
			path.erase(current);
		} else {
			lastPos = *it;
			current = it++;
		}
	}
}