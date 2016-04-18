//! \addtogroup 0007 Pathfinding
//! \brief A pathfinding module
//!
//! A pathfinding module that can be used in the R2D2 project.
//! The module is currently based on the A star algorithm.
//!
//! \file   main.cpp
//! \author Jasper Schoenmaker 1661818
//! \author Chiel Douwes 1666311
//! \author Ole Achterberg 1651981
//! \date   Created: 01-04-2016
//! \date   Last Modified: 15-4-2016
//! \brief  main used for testing
//!
//! Main used for testing
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
// prints out a .pgm (portable grey r2d2::Map) image of the r2d2::Map and it's path
#define SCALE 4

void printMapWithPath(std::ostream &out, r2d2::Map &map,
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

        r2d2::Map map = {mapX, mapY, 0.4f};
        r2d2::PathFinder pathFinder = {map,
                                       {0.5 * r2d2::Length::METER,
                                        0.5 * r2d2::Length::METER,
                                        0 * r2d2::Length::METER}};
        std::vector<r2d2::Coordinate> path;
        done = pathFinder.get_path_to_coordinate(
                {5.5f * r2d2::Length::METER,
                 5.5f * r2d2::Length::METER,
                 0.0f * r2d2::Length::METER},
                {(mapX - 5.5f) * r2d2::Length::METER,
                 (mapY - 5.5f) * r2d2::Length::METER,
                 0.0f * r2d2::Length::METER},
                path);
        mapCount++;

        if (done) {
            std::unordered_set<IntCoord> intPath;
            for (r2d2::Coordinate &coord : path) {
                std::cout << coord << std::endl;
                intPath.emplace(coord.get_x() / r2d2::Length::METER * SCALE,
                                coord.get_y() / r2d2::Length::METER * SCALE);
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