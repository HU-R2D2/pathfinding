//! \addtogroup 0007 Pathfinding
//! \brief A pathfinding module
//!
//! A pathfinding module that can be used in the R2D2 project.
//! The module is currently based on the A star algorithm.
//!
//! \file   Dummy.hpp
//! \author Jasper Schoenmaker 1661818
//! \author Chiel Douwes 1666311
//! \date   Created: 29-03-2016
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

#ifndef R2D2_PATHFINDING_DUMMY_HPP
#define R2D2_PATHFINDING_DUMMY_HPP

#include <iostream>
#include <random>
#include <vector>
#include <ctime>
#include <MapInterface.hpp>
#include <Coordinate.hpp>
#include <Translation.hpp>

namespace r2d2 {
    //! Dummy Map
    /*!
    * Map for testing the pathfinder
    */
    class Dummy : public ReadOnlyMap {
    public:
        //! Implementation of the map, where: 0 = clear, 1 = obstacle, 2 = unexplored
        std::vector<std::vector<int>> map;

        //! The sizes of the map
        int sizeX, sizeY;

        //! Constructor
        /*!
        * \param x The width of the map
        * \param y The height of the map
        * \param obstacles Percentage of obstacles in the map
        */
        Dummy(int x = 100, int y = 100, float obstacles = 0.25f);

        //! Constructor
        /*!
        * \param map The map
        */
        Dummy(std::vector<std::vector<int> > map);

        //! Print the map
        /*!
        * \return void
        */
        void print_map();

        virtual const BoxInfo get_box_info(const Box box) override;

        virtual const Box get_map_bounding_box() override;

    private:
        static std::mt19937_64 mersenne;

    };

}

#endif //R2D2_PATHFINDING_DUMMY_HPP
