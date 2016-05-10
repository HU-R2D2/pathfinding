//! \addtogroup 0007 Pathfinding
//! \brief A pathfinding module
//!
//! A pathfinding module that can be used in the R2D2 project.
//! The module is currently based on the A star algorithm.
//!
//! \file   AStarPathFinder.hpp
//! \author Chiel Douwes 1666311
//! \date   Created: 30-03-2016
//! \date   Last Modified: 15-4-2016
//! \brief  Implementation of the pathfinder interface
//!
//! Takes a starting point and end point and a reference to a path.
//! If a path is possible it returns a path to the end point. If no path is
//! possible it will return false.
//! This implementation uses A* for the search, which is defined in the
//! "Astar" class.
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

#ifndef R2D2_PATHFINDING_ASTARPATHFINDER_HPP
#define R2D2_PATHFINDING_ASTARPATHFINDER_HPP

#include "PathFinder.hpp"
#include "Astar.hpp"

// defines the amount of nodes that will be visited per length of the robot
// for instance, if the robot has a size of 1m, and this value is 2, a node will
// be opened every .5m
#define SQUARES_PER_ROBOT 2

namespace r2d2 {

    /**
     * interface for a pathfinder module
     *
     * computes a path between two points on a map
     */
    class AStarPathFinder : public PathFinder {
    public:
        AStarPathFinder(Map &map, Translation robotBox);

        virtual bool get_path_to_coordinate(
                Coordinate start,
                Coordinate goal,
                std::vector<Coordinate> &path) override;

    private:

        /**
         * implementation of the astar node from Astar.hpp
         */
        class CoordNode : public Node<CoordNode> {
        public:
            CoordNode(AStarPathFinder &pathFinder, Coordinate coord,
                      Coordinate &startCoord,
                      Length g = r2d2::Length::METER *
                                 std::numeric_limits<double>::infinity(),
                      std::weak_ptr<CoordNode> parent = {});

            virtual bool operator==(const CoordNode &lhs) const override;

            virtual std::vector<CoordNode> get_available_nodes(
                    std::shared_ptr<CoordNode> &self) override;

            AStarPathFinder &pathFinder;
            Coordinate coord, &startNodeCoord;

            friend std::ostream &operator<<(std::ostream &lhs,
                                            const CoordNode &rhs) {
                return lhs << "(" << rhs.coord << ", " << rhs.g
                       << ", " << rhs.h << ", " << rhs.f << ")";
            }
        };

        friend struct std::hash<CoordNode>;

        Map &map;
        Translation robotBox;

        /**
         * test whether it is possible to travel from "from" directly to "to"
         *
         * if this function returns true, then it is guaranteed that there is a
         * direct path between "from" and "to". the function may return false
         * even if the is a direct connection because it has to absolutely
         * certain.
         * \param from the coordinate that will be travelled from
         * \param to the coordinate that will be travelled to from "from"
         * \return true if it is guaranteed that the robot can travel from
         * "from" to "to"
         */
        bool can_travel(const Coordinate &from, const Coordinate &to);

        /**
         * check whether a coordinate will be overlapped by the robot when
         * positioned on a second coordinate
         *
         * this could be used to check whether two nodes can possibly be
         * considered as one
         * \param c1 the position the robot is on
         * \param c2 the coordinate that should be checked for overlapping
         * with the robot
         * \return c1 overlaps c2 within the size of the robot
         */
        bool overlaps(const Coordinate &c1, const Coordinate &c2);

        /**
         * get the traversed distance if the robot goes from {0, 0} to "coord"
         *
         * \param coord the coordinate that should be calculated the length for
         * \return the distance from origin to "coord"
         */
        static Length get_heuristic(Translation coord);

        /**
         * extracts a path from a pathfinder search field
         *
         * \param start the node to start the search from
         * \return the computed path from the supplied node to the goal node
         */
        std::vector<CoordNode> get_path(std::shared_ptr<CoordNode> start);

        /**
         * strips a path of all unnecessary nodes, smoothing the path in the process
         *
         * \param path the path to smooth
         * \param start the original start coordinate
         */
        void smooth_path(std::vector<Coordinate> &path, Coordinate start);
    };

}

namespace std {

    template<>
    struct hash<r2d2::Coordinate> {
        std::size_t operator()(const r2d2::Coordinate &coord) const {
            return std::hash<double>()(coord.get_x() / r2d2::Length::METER)
                   ^ (std::hash<double>()(coord.get_y() / r2d2::Length::METER)
                      << (sizeof(double) / 2));
        }
    };

    /**
     * hash for the coordinate node class, used for set insertion
     */
    template<>
    struct hash<r2d2::AStarPathFinder::CoordNode> {
        std::size_t operator()(const r2d2::AStarPathFinder::CoordNode &node) const {
            return std::hash<r2d2::Coordinate>()(node.coord);
        }
    };

}

#endif //R2D2_PATHFINDING_ASTARPATHFINDER_HPP
