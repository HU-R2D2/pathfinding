////                                                                                                                                        
// \project Roborescue
// \package Pathfinding
// 
// \file Dummy.cpp
// \date Created: 05-04-2016
// \version 0.1.0
//
// \author Jasper Schoenmaker 1661818
// \author Chiel Douwens 1666311
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

#include "../include/Dummy.hpp"

std::mt19937_64 Map::mersenne = std::mt19937_64{(unsigned long) (time(0))};

Map::Map(int x, int y, float obstacles) :
		map{}
//		mersenne{(unsigned long) (time(0))}
{
	map.reserve((unsigned long) (x));
	sizeX = x, sizeY = y;
	for (int i1 = 0; i1 < x; i1++) {
		map.emplace_back();
		map[i1].reserve((unsigned long)(y));
		for (int i2 = 0; i2 < y; i2++) {
			map[i1].emplace_back();
			map[i1][i2] = std::uniform_real_distribution<float>{}(mersenne)
			              < obstacles ? 1 : 0;
		}
	}
}


Map::Map(std::vector<std::vector<int>> map) :
	map{ map },
	sizeX{ int(map.size()) },
	sizeY{ int(map[0].size()) }
{
}

void Map::print_map() {
	for (int i1 = 0; i1 < sizeX; i1++) {
		for (int i2 = 0; i2 < sizeY; i2++) {
			std::cout << map[i1][i2];
		}
		std::cout << std::endl;
	}
}

bool Map::has_obstacle(r2d2::Coordinate coord, r2d2::Translation size) {
	for (int i1 = int(coord.get_x() / r2d2::Length::METER); i1 <= int((coord.get_x() + size.get_x()) / r2d2::Length::METER); i1++) {
		for (int i2 = int(coord.get_y() / r2d2::Length::METER); i2 <= int((coord.get_y() + size.get_y()) / r2d2::Length::METER); i2++) {
			if (i1 < 0 || i1 >= sizeX ||
			    i2 < 0 || i2 >= sizeY ||
			    map[i1][i2] == 1 || map[i1][i2] == 2) {
				return true;
			}
		}
	}
	return false;
}

bool Map::has_passable(r2d2::Coordinate coord, r2d2::Translation size) {
	for (int i1 = int(coord.get_x() / r2d2::Length::METER); i1 <= int((coord.get_x() + size.get_x()) / r2d2::Length::METER); i1++) {
		for (int i2 = int(coord.get_y() / r2d2::Length::METER); i2 <= int((coord.get_y() + size.get_y()) / r2d2::Length::METER); i2++) {
			if (i1 < 0 && i1 >= sizeX &&
			    i2 < 0 && i2 >= sizeY &&
			    map[i1][i2] == 0) {
				return true;
			}
		}
	}
	return false;
}