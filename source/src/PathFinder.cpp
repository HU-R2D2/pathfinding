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

#include "../include/PathFinder.hpp"

PathFinder::PathFinder(Map &map, r2d2::Translation robotBox):
		map(map),
		robotBox(robotBox) {
}

bool PathFinder::get_path_to_coordinate(r2d2::Coordinate start, r2d2::Coordinate goal, std::vector<r2d2::Coordinate> &path) {
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
	CoordNode endNode{*this, goal, start, 0 * r2d2::Length::METER}, startNode{*this, start, start};
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
		PathFinder &pathFinder, r2d2::Coordinate coord,
		r2d2::Coordinate &startCoord, r2d2::Length g, std::weak_ptr<CoordNode> parent) :
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
				r2d2::Coordinate childPos{
						coord + (r2d2::Translation{
								x * pathFinder.robotBox.get_x(),
						        y * pathFinder.robotBox.get_y(),
						        0 * r2d2::Length::METER
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
	return (coord - lhs.coord).get_length() / r2d2::Length::METER == 0;
}

bool PathFinder::can_travel(const r2d2::Coordinate &from, const r2d2::Coordinate &to) {
	r2d2::Coordinate minCoord{
			(from.get_x() < to.get_x() ? from.get_x() : to.get_x()),
			(from.get_y() < to.get_y() ? from.get_y() : to.get_y()),
			0 * r2d2::Length::METER};
	r2d2::Translation size{(r2d2::Coordinate{
			(from.get_x() > to.get_x() ? from.get_x() : to.get_x()),
			(from.get_y() > to.get_y() ? from.get_y() : to.get_y()),
			0 * r2d2::Length::METER} - minCoord) + robotBox};
	return !map.has_obstacle(minCoord - (robotBox / 2), size);
}

bool PathFinder::overlaps(const r2d2::Coordinate &c1, const r2d2::Coordinate &c2) {
	r2d2::Translation diff = c1 - c2;
	return (diff.get_x() < 0 * r2d2::Length::METER ?
	        0 * r2d2::Length::METER - diff.get_x() :
	        diff.get_x()
	       ) < robotBox.get_x() / 2 &&
	       (diff.get_y() < 0 * r2d2::Length::METER ?
	        0 * r2d2::Length::METER - diff.get_y() :
	        diff.get_y()
	       ) < robotBox.get_y() / 2;
}

// define a constant as to speed the calculation up,
// in this case 10 digits is "good enough"
#define SQ_ROOT_2 1.414213562f
r2d2::Length PathFinder::get_heuristic(r2d2::Translation coord) {
	// diagonal distance
	r2d2::Length xDist = (coord.get_x() < 0 * r2d2::Length::METER) ? (0 * r2d2::Length::METER - coord.get_x()) : coord.get_x();
	r2d2::Length yDist = (coord.get_y() < 0 * r2d2::Length::METER) ? (0 * r2d2::Length::METER - coord.get_y()) : coord.get_y();
	r2d2::Length shortDist, longDist;
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

void PathFinder::smooth_path(std::vector<r2d2::Coordinate> &path, r2d2::Coordinate start) {
	r2d2::Coordinate lastPos = start;
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