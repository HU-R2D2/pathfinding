//! \addtogroup 0007 Pathfinding
//! \brief A pathfinding module
//!
//! A pathfinding module that can be used in the R2D2 project.
//! The module is currently based on the A star algorithm.
//!
//! \file   PathFinder.hpp
//! \author Chiel Douwes 1666311
//! \date   Created: 30-03-2016
//! \date   Last Modified: 15-4-2016
//! \brief  Find the path and returns the path
//!
//! Takes a starting point and end point and a reference to a path.
//! If a path is possible it returns a path to the end point. If no path is
//! possible it will return false.
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

#ifndef R2D2_PATHFINDING_PATHFINDER_HPP
#define R2D2_PATHFINDING_PATHFINDER_HPP
#include <vector>
#include <memory>
#include "../../../adt/source/include/Box.hpp"
#include "../../../sharedobjects/source/include/SharedObject.hpp"
// TODO replace this with the actual interface
#include "Dummy.hpp"

namespace r2d2 {

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
		PathFinder(SharedObject<Map> &map, Box robotBox) {};

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
				Coordinate start,
				Coordinate goal,
				std::vector<Coordinate> &path) = 0;
	};

}

#endif //R2D2_PATHFINDING_PATHFINDER_HPP
