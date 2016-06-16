
#include "../source/include/PathFinder.hpp"
#include "../source/include/AStarPathFinder.hpp"
#include "../source/include/Dummy.hpp"
#include "../source/include/Astar.hpp"
#include "Box.hpp"
#include "Coordinate.hpp"
#include "Length.hpp"
#include "Angle.hpp"
#include "Translation.hpp"
#include "SharedObject.hpp"
#include "LockingSharedObject.hpp"

#include "iostream"


int main(int ac, char*av []){

//Creating a map
std::vector<std::vector<int>> cornerSqueezeMap;
for (int x = 0; x < 50; x++) {
      std::vector<int> current;
      for (int y = 50; y > 0; y--) {
         if (x == y) {
            if (x >= 4 && x <= 6){
               current.push_back(0);
            }
            else{
               current.push_back(1);
            }
         } else {
            current.push_back(0);
         }
      }
      cornerSqueezeMap.push_back(current);
}
//initializing a map
r2d2::Dummy map(cornerSqueezeMap);
map.print_map();

// initializing pathfinding
r2d2::Translation robotBox{ .5 * r2d2::Length::METER,
                            .5 * r2d2::Length::METER,
                            0 * r2d2::Length::METER};
LockingSharedObject<r2d2::ReadOnlyMap> sharedMap{map};
r2d2::AStarPathFinder path_finder(sharedMap, {{}, robotBox});

//Computing a path between coordinates
r2d2::Coordinate c1 {1 * r2d2::Length::METER,  2 * r2d2::Length::METER,
0 * r2d2::Length::METER};
r2d2::Coordinate c2 {4 * r2d2::Length::METER,
                     3 * r2d2::Length::METER,
                     0 * r2d2::Length::METER};
std::vector<r2d2::Coordinate> path_vector{r2d2::Coordinate(
                     1 * r2d2::Length::METER,
                     3 * r2d2::Length::METER,
                     0 * r2d2::Length::METER)};
cout << path_finder.get_path_to_coordinate(c1, c2, path_vector);
}
