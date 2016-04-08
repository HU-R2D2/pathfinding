////                                                                                                                                        
// \project Roborescue
// \package Pathfinding
// 
// \file PathFinder.cpp
// \date Created: 01-04-2016
// \version 0.1.0
//
// \author Jasper Schoenmaker 1661818
// \author Chiel Douwens 1666311
// \author Ole Achterberg 1651981 
//
// \section LICENSE
// License: newBSD
//
// Copyright © 2016, HU University of Applied Sciences Utrecht.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
// - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
// - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
// OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
////

#ifndef R2D2_PATHFINDING_PATHFINDER_HPP
#define R2D2_PATHFINDING_PATHFINDER_HPP


#include <vector>
#include <cmath>
#include "Dummy.hpp"
#include "Astar.hpp"
#include "../../../deps/adt/source/include/Coordinate.hpp"
#include "../../../deps/adt/source/include/Length.hpp"
#include "../../../deps/adt/source/include/Translation.hpp"

// defines the amount of nodes that will be visited per length of the robot
// for instance, if the robot has a size of 1m, and this value is 2, a node will
// be opened every .5m
#define SQUARES_PER_ROBOT 2


/**
 * interface for a pathfinder module
 *
 * computes a path between two points on a map
 */
class PathFinder {
public:
	/**
	 * Constructor
	 *
	 * /param map Reference to the world map
	 * /param robotSize Reference to the robot size
	 */
	PathFinder(Map &map, Translation robotBox);

	/**
	 * Returns a path between two points
	 *
	 * This function computes a path from start to goal,
	 * based on the map and robot size given in the constructor
	 *
	 * /param start The start coordinate
	 * /param goal The goal coordinate
	 * /param path Vector where the path need to be written to
	 * /return If it was able to find a path
	 */
	virtual bool get_path_to_coordinate(
			Coordinate start,
			Coordinate goal,
			std::vector<Coordinate> &path);

	/**
	 * implementation of the astar node from Astar.hpp
	 */
	class CoordNode : public Node<CoordNode> {
	public:
		CoordNode(PathFinder &pathFinder, Coordinate coord,
		          Coordinate &startCoord,
		          Length g = Length::METER * std::numeric_limits<double>::infinity(),
		          std::weak_ptr<CoordNode> parent = {});

		virtual bool operator==(const CoordNode &lhs) const override;

		virtual std::vector<CoordNode> get_available_nodes(
				std::shared_ptr<CoordNode> &self) override;

		PathFinder &pathFinder;
		Coordinate coord, &startNodeCoord;

		friend std::ostream &operator<<(std::ostream &lhs,
		                                const CoordNode &rhs) {
			return lhs << "(" << rhs.coord << ", " << rhs.g
			       << ", " << rhs.h << ", " << rhs.f << ")";
		}
	};

private:
	Map &map;
	Translation robotBox;

	/**
	 * test whether it is possible to travel from "from" directly to "to"
	 *
	 * if this function returns true, then it is guaranteed that there is a
	 * direct path between "from" and "to". the function may return false even
	 * if the is a direct connection because it has to absolutely certain.
	 * @param from the coordinate that will be travelled from
	 * @param to the coordinate that will be travelled to from "from"
	 * @return true if it is guaranteed that the robot can travel from "from" to
	 *         "to"
	 */
	bool can_travel(const Coordinate &from, const Coordinate &to);

	/**
	 * check whether a coordinate will be overlapped by the robot when
	 * positioned on a second coordinate
	 *
	 * this could be used to check whether two nodes can possibly be considered
	 * as one
	 * @param c1 the position the robot is on
	 * @param c2 the coordinate that should be checked for overlapping with the
	 *           robot
	 * @return c1 overlaps c2 within the size of the robot
	 */
	bool overlaps(const Coordinate &c1, const Coordinate &c2);

	/**
	 * get the traversed distance if the robot goes from {0, 0} to "coord"
	 *
	 * @param coord the coordinate that should be calculated the length for
	 * @return the distance from orgin to "coord"
	 */
	static Length get_heuristic(Translation coord);

	std::vector<CoordNode> get_path(std::shared_ptr<CoordNode> start);

	void smooth_path(std::vector<Coordinate> &path, Coordinate start);
};

namespace std {

	template<>
	struct hash<Coordinate> {
		std::size_t operator()(const Coordinate &coord) const {
			return std::hash<double>()(coord.get_x() / Length::METER)
			       ^ (std::hash<double>()(coord.get_y() / Length::METER) << (sizeof(double) / 2));
		}
	};

	/**
	 * hash for the coordinate node class, used for set insertion
	 */
	template<>
	struct hash<PathFinder::CoordNode> {
		std::size_t operator()(const PathFinder::CoordNode& node) const {
			return std::hash<Coordinate>()(node.coord);
		}
	};

}

#endif //R2D2_PATHFINDING_PATHFINDER_HPP


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
	if (!can_travel(goal, goal)) {
		return false;
	}

	// parent is at this point unknown for the start node, so construct it as unknown
	CoordNode endNode{*this, goal, start, 0 * Length::METER}, startNode{*this, start, start};
	AStarSearch<CoordNode> search{endNode};

	std::shared_ptr<CoordNode> foundStart = search.search(startNode);
	if (foundStart == nullptr) {
		return false;
	} else {
		std::vector<CoordNode> foundPath{get_path(foundStart)};
		path.clear();
		for (CoordNode &node : foundPath) {
			path.push_back(node.coord);
		}
		smooth_path(path, start);
		return true;
	}
}

PathFinder::CoordNode::CoordNode(
		PathFinder &pathFinder, Coordinate coord,
		Coordinate &startCoord, Length g, std::weak_ptr<CoordNode> parent) :
		Node{g, PathFinder::get_heuristic(startCoord - coord),
		     parent},
		pathFinder(pathFinder),
		coord(coord),
		startNodeCoord(startCoord) {
//	std::cout << "create node " << coord << " hash: " << std::hash<CoordNode>()(*this) << std::endl;
}

std::vector<PathFinder::CoordNode> PathFinder::CoordNode::get_available_nodes(
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
				// can_travel is used so that it can be ensured that there is no
				// obstacle in the path
				if (pathFinder.can_travel(coord, childPos)) {
					children.push_back(
							CoordNode{pathFinder, childPos, startNodeCoord,
							          g + PathFinder::get_heuristic(
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

bool PathFinder::can_travel(const Coordinate &from, const Coordinate &to) {
	Coordinate minCoord{
			(from.get_x() < to.get_x() ? from.get_x() : to.get_x()),
			(from.get_y() < to.get_y() ? from.get_y() : to.get_y()),
			0 * Length::METER};
	Translation size{(Coordinate{
			(from.get_x() > to.get_x() ? from.get_x() : to.get_x()),
			(from.get_y() > to.get_y() ? from.get_y() : to.get_y()),
			0 * Length::METER} - minCoord) + robotBox};
	return !map.has_obstacle(minCoord - (robotBox / 2), size);
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
Length PathFinder::get_heuristic(Translation coord) {
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

std::vector<PathFinder::CoordNode> PathFinder::get_path(
		std::shared_ptr<PathFinder::CoordNode> start) {
	std::shared_ptr<CoordNode> curNode = start;
	std::vector<CoordNode> path;
	while (!curNode->parent.expired()) {
		curNode = std::shared_ptr<CoordNode>(curNode->parent);
		path.emplace_back(*curNode);
	}
	return path;
}

void PathFinder::smooth_path(std::vector<Coordinate> &path, Coordinate start) {
	Coordinate lastPos = start;
	auto it = path.begin();
	auto current = it++;
	while (it != path.end()) {
		if (can_travel(lastPos, *it)) {
			path.erase(current);
		} else {
			lastPos = *it;
			current = it++;
		}
	}
}