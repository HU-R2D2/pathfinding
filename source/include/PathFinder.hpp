//
// Created by chiel on 30-3-16.
//

#ifndef R2D2_PATHFINDING_PATHFINDER_HPP
#define R2D2_PATHFINDING_PATHFINDER_HPP


#include <vector>
#include <cmath>
#include "Dummy.hpp"
#include "Astar.hpp"
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
