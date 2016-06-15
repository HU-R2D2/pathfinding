
#include "../source/include/PathFinder.hpp"
#include "../source/include/AStarPathFinder.hpp"
#include "../source/include/Dummy.hpp"
#include "../source/include/Astar.hpp"
#include "Box.hpp"
#include "Coordinate.hpp"
#include "Length.hpp"
#include "Translation.hpp"
#include "SharedObject.hpp"
#include "LockingSharedObject.hpp"

#include "iostream"

int main(int ac, char*av []){

//initializing a map
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
r2d2::Map map(cornerSqueezeMap);
map.print_map();

// initializing pathfinding
r2d2::Translation robotBox{.5 * r2d2::Length::METER,
                           .5 * r2d2::Length::METER,
                           0 * r2d2::Length::METER};
LockingSharedObject<r2d2::Map> sharedMap{map};
r2d2::AStarPathFinder path_finder(sharedMap, {{}, robotBox});

}
