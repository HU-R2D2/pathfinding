////                                                                                                                                        
// Roborescue
//
// \file Dummy.hpp
// \date Created: 30-3-16
// \version <0.0.0>
//
// \author Chiel Douwes
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

#ifndef R2D2_PATHFINDING_DUMMY_HPP
#define R2D2_PATHFINDING_DUMMY_HPP

#include <iostream>
#include <random>
#include <vector>
#include <ctime>

//! Dummy Map
/*!
* Map to test the pathfinder
*/
class Map {
public:
	//! Implementation of the map, where: 0 = clear, 1 = obstacle, 2 = unexplored
	std::vector<std::vector<int>> map;

	//! The sizes of the map
	int sizeX, sizeY;

	//! Constructor
	/*!
	* /param x The width of the map
	* /param y The height of the map
	* /param obstacles Percentage of obstacles in the map
	*/
	Map(int x = 100, int y = 100, float obstacles = 0.25f);

	//! Constructor
	/*!
	* /param map The map
	*/
	Map(std::vector < std::vector<int> > map);


	//! Print the map
	/*!
	* /return void
	*/
	void printMap();

	//! Returns if position on the map has a obstacle within the robot size
	/*!
	* /param x The x position on the map
	* /param y The y position on the map
	* /param sizeX The width of the robot
	* /param sizeY The height of the robot
	* /return If there is a obstacle found
	*/
	bool hasObstacle(float x, float y, float sizeX, float sizeY);


	//! Returns if position on the map has a passable erea within the robot size
	/*!
	* /param x The x position on the map
	* /param y The y position on the map
	* /param sizeX The width of the robot
	* /param sizeY The height of the robot
	* /return If there is a passable erea
	*/
	bool hasPassable(float x, float y, float sizeX, float sizeY);

private:
	std::mt19937_64 mersenne;

};


#endif //R2D2_PATHFINDING_DUMMY_HPP
