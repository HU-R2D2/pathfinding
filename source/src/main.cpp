////                                                                                                                                        
// \project Roborescue
// \package Pathfinding
// 
// \file main.cpp
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
#include "../include/Dummy.hpp"
#include "../include/Astar.hpp"
#include "../../../adt/source/include/Coordinate.hpp"
#include "../../../adt/source/include/Length.hpp"
#include "../../../adt/source/include/Translation.hpp"

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
     * \param map Reference to the world map
     * \param robotSize Reference to the robot size
     */
    PathFinder(Map &map, r2d2::Translation robotBox);

    /**
     * Returns a path between two points
     *
     * This function computes a path from start to goal,
     * based on the map and robot size given in the constructor
     *
     * \param start The start coordinate
     * \param goal The goal coordinate
     * \param path Vector where the path need to be written to
     * \return If it was able to find a path
     */
    virtual bool get_path_to_coordinate(
            r2d2::Coordinate start,
            r2d2::Coordinate goal,
            std::vector<r2d2::Coordinate> &path);

    /**
     * implementation of the astar node from Astar.hpp
     */
    class CoordNode : public Node<CoordNode> {
    public:
        CoordNode(PathFinder &pathFinder, r2d2::Coordinate coord,
                  r2d2::Coordinate &startCoord,
                  r2d2::Length g = r2d2::Length::METER * std::numeric_limits<double>::infinity(),
                  std::weak_ptr<CoordNode> parent = {});

        virtual bool operator==(const CoordNode &lhs) const override;

        virtual std::vector<CoordNode> get_available_nodes(
                std::shared_ptr<CoordNode> &self) override;

        PathFinder &pathFinder;
        r2d2::Coordinate coord, &startNodeCoord;

        friend std::ostream &operator<<(std::ostream &lhs,
                                        const CoordNode &rhs) {
            return lhs << "(" << rhs.coord << ", " << rhs.g
                   << ", " << rhs.h << ", " << rhs.f << ")";
        }
    };

private:
    Map &map;
    r2d2::Translation robotBox;

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
    bool can_travel(const r2d2::Coordinate &from, const r2d2::Coordinate &to);

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
    bool overlaps(const r2d2::Coordinate &c1, const r2d2::Coordinate &c2);

    /**
     * get the traversed distance if the robot goes from {0, 0} to "coord"
     *
     * @param coord the coordinate that should be calculated the length for
     * @return the distance from orgin to "coord"
     */
    static r2d2::Length get_heuristic(r2d2::Translation coord);

    std::vector<CoordNode> get_path(std::shared_ptr<CoordNode> start);

    void smooth_path(std::vector<r2d2::Coordinate> &path, r2d2::Coordinate start);
};

namespace std {

    template<>
    struct hash<r2d2::Coordinate> {
        std::size_t operator()(const r2d2::Coordinate &coord) const {
            return std::hash<double>()(coord.get_x() / r2d2::Length::METER)
                   ^ (std::hash<double>()(coord.get_y() / r2d2::Length::METER) << (sizeof(double) / 2));
        }
    };

    /**
     * hash for the coordinate node class, used for set insertion
     */
    template<>
    struct hash<PathFinder::CoordNode> {
        std::size_t operator()(const PathFinder::CoordNode& node) const {
            return std::hash<r2d2::Coordinate>()(node.coord);
        }
    };

}

#endif //R2D2_PATHFINDING_PATHFINDER_HPP


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
                {0.5 * r2d2::Length::METER, 
                0.5 * r2d2::Length::METER,
                0 * r2d2::Length::METER}};
        std::vector<r2d2::Coordinate> path;
        done = pathFinder.get_path_to_coordinate(
                {5.5f * r2d2::Length::METER,
                5.5f * r2d2::Length::METER,
                0.0f * r2d2::Length::METER},
                {(mapX - 5.5f)* r2d2::Length::METER,
                (mapY - 5.5f) * r2d2::Length::METER,
                0.0f * r2d2::Length::METER},
                path);
        mapCount++;

        if (done) {
            std::unordered_set<IntCoord> intPath;
            for (r2d2::Coordinate &coord : path) {
                std::cout << coord << std::endl;
                intPath.emplace(coord.get_x() / r2d2::Length::METER * SCALE, coord.get_y() / r2d2::Length::METER * SCALE);
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