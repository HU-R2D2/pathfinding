//! \addtogroup 0007 Pathfinding
//! \brief A pathfinding module
//!
//! A pathfinding module that can be used in the R2D2 project.
//! The module is currently based on the A star algorithm.
//!
//! \file   AStarPathFinder.cpp
//! \author Jasper Schoenmaker 1661818
//! \author Chiel Douwes 1666311
//! \author Ole Achterberg 1651981
//! \date   Created: 01-04-2016
//! \date   Last Modified: 15-4-2016
//! \brief  Find the path and returns the path
//!
//! Takes a starting point and end point and a reference to a path.
//! If a path is possible it returns a path to the end point. If no path is
//! possible it will return false.
//!
//! \copyright Copyright © 2016, HU University of Applied Sciences Utrecht.
//! All rights reserved.
//!
//! License: newBSD
//!
//! Redistribution and use in source and binary forms,
//! with or without modification, are permitted provided that
//! the following conditions are met:
//! - Redistributions of source code must retain the above copyright notice,
//!   this list of conditions and the following disclaimer.
//! - Redistributions in binary form must reproduce the above copyright notice,
//!   this list of conditions and the following disclaimer in the documentation
//!   and/or other materials provided with the distribution.
//! - Neither the name of the HU University of Applied Sciences Utrecht
//!   nor the names of its contributors may be used to endorse or promote
//!   products derived from this software without specific prior written
//!   permission.
//!
//! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//! "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
//! BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
//! AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
//! IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
//! BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//! CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
//! PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
//! OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
//! WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
//! OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
//! EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// ~< HEADER_VERSION 2016 04 12 >~

#include "../include/AStarPathFinder.hpp"

namespace r2d2 {

    AStarPathFinder::AStarPathFinder(Map &map, Translation robotBox) :
            PathFinder{map, robotBox},
            map(map),
            robotBox(robotBox) {
    }

    bool AStarPathFinder::get_path_to_coordinate(Coordinate start,
                                            Coordinate goal,
                                            std::vector<Coordinate>
                                            &path) {
        // check for the goal node being at the same coordinate as the start node
        if (overlaps(start, goal)) {
            path.clear();
            return true;
        }
        // do a check for end node accessibility before starting the search
        if (!can_travel(goal, goal)) {
            return false;
        }

        // parent is at this point unknown for the start node,
        // so construct it as unknown
        CoordNode endNode{*this, goal, start, 0 * Length::METER},
                startNode{*this,
                          start,
                          start};
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

    AStarPathFinder::CoordNode::CoordNode(
            AStarPathFinder &pathFinder, Coordinate coord,
            Coordinate &startCoord, Length g,
            std::weak_ptr<CoordNode> parent) :
            Node{g, AStarPathFinder::get_heuristic(startCoord - coord),
                 parent},
            pathFinder(pathFinder),
            coord(coord),
            startNodeCoord(startCoord) {
    }

    std::vector<AStarPathFinder::CoordNode>
    AStarPathFinder::CoordNode::get_available_nodes(
            std::shared_ptr<AStarPathFinder::CoordNode> &self) {
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
                                          g + AStarPathFinder::get_heuristic(
                                                  childPos - coord
                                          ), // distance from the search begin
                                          self});
                    }
                }
            }
        }
        return children;
    }

    bool AStarPathFinder::CoordNode::operator==(
            const AStarPathFinder::CoordNode &lhs) const {
        return (coord - lhs.coord).get_length() / Length::METER == 0;
    }

    bool AStarPathFinder::can_travel(const Coordinate &from,
                                const Coordinate &to) {
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

    bool AStarPathFinder::overlaps(const Coordinate &c1,
                              const Coordinate &c2) {
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

    Length AStarPathFinder::get_heuristic(Translation coord) {
        // diagonal distance
        Length xDist = (coord.get_x() < 0 * Length::METER) ?
                       (0 * Length::METER - coord.get_x()) : coord.get_x();
        Length yDist = (coord.get_y() < 0 * Length::METER) ?
                       (0 * Length::METER - coord.get_y()) : coord.get_y();
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

    std::vector<AStarPathFinder::CoordNode> AStarPathFinder::get_path(
            std::shared_ptr<AStarPathFinder::CoordNode> start) {
        std::shared_ptr<CoordNode> curNode = start;
        std::vector<CoordNode> path;
        while (!curNode->parent.expired()) { // while next != nullptr
            curNode = std::shared_ptr<CoordNode>(curNode->parent);
            path.emplace_back(*curNode);
        }
        return path;
    }

    void AStarPathFinder::smooth_path(std::vector<Coordinate> &path,
                                 Coordinate start) {
        Coordinate lastPos = start;
        auto it = path.begin();
        auto current = it++;
        while (it != path.end()) {
            // check if the is a path from the current anchor node to the next
            if (can_travel(lastPos, *it)) {
                // if so, remove the node(s) in between
                path.erase(current);
            } else {
                // if not, make the current node the new anchor node
                lastPos = *it;
                current = it++;
            }
        }
    }

}