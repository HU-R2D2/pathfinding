//! \addtogroup 0007 Pathfinding
//! \brief A pathfinding module
//!
//! A pathfinding module that can be used in the R2D2 project.
//! The module is currently based on the A star algorithm.
//!
//! \file   Dummy.cpp
//! \author Jasper Schoenmaker 1661818
//! \author Chiel Douwes 1666311
//! \date   Created: 05-04-2016
//! \date   Last Modified: 15-4-2016
//! \brief  Dummy map for pathfinding
//!
//! A dummy implementation of the map, without it the pathfinding module could
//! not be tested.
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

#include "../include/Dummy.hpp"

namespace r2d2 {

    std::mt19937_64 Dummy::mersenne = std::mt19937_64{
            (unsigned long) (time(0))};

    Dummy::Dummy(int x, int y, float obstacles) :
            map{},
            sizeX{x},
            sizeY{y} {
        map.reserve((unsigned long) (y));
        for (int i1 = 0; i1 < y; i1++) {
            map.emplace_back();
            map[i1].reserve((unsigned long) (x));
            for (int i2 = 0; i2 < x; i2++) {
                map[i1].emplace_back();
                map[i1][i2] = std::uniform_real_distribution<float>{}(mersenne)
                              < obstacles ? 1 : 0;
            }
        }
    }


    Dummy::Dummy(std::vector<std::vector<int>> map) :
            map{map},
            sizeX{int(map[0].size())},
            sizeY{int(map.size())} {
    }

    void Dummy::print_map() {
        for (int i1 = 0; i1 < sizeY; i1++) {
            for (int i2 = 0; i2 < sizeX; i2++) {
                std::cout << map[i1][i2];
            }
            std::cout << std::endl;
        }
    }

    const BoxInfo Dummy::get_box_info(const Box box) {
        bool obstacle = false, navigable = false, unknown = false;
        for (int i1 = int(box.get_bottom_left().get_y() / Length::METER);
             i1 <= int(box.get_top_right().get_y() / Length::METER); i1++) {
            for (int i2 = int(box.get_bottom_left().get_x() / Length::METER);
                 i2 <= int(box.get_top_right().get_x() / Length::METER); i2++) {
                if (i1 < 0 || i1 >= sizeY ||
                    i2 < 0 || i2 >= sizeX) {
                    unknown = true;
                } else {
                    switch (map[i1][i2]) {
                        case 0:
                            navigable = true;
                            break;
                        case 1:
                            obstacle = true;
                            break;
                        case 2:
                            unknown = true;
                            break;
                        default:;
                    }
                }
            }
        }
        return {obstacle, navigable, unknown};
    }

    const Box Dummy::get_map_bounding_box() {
        return {};
    }

}