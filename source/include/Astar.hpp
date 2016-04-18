//! \addtogroup 0007 Pathfinding
//! \brief A pathfinding module
//!
//! A pathfinding module that can be used in the R2D2 project.
//! The module is currently based on the A star algorithm.
//!
//! \file   Astar.hpp
//! \author Jasper Schoenmaker 1661818
//! \author Chiel Douwes 1666311
//! \date   Created: 29-03-2016
//! \date   Last Modified: 15-4-2016
//! \brief  A star algorithm
//!
//! Implementation of the A star algorithm, used for finding the actual path.
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
// ~< HEADER_VERSION

#ifndef R2D2_PATHFINDING_ASTAR_HPP
#define R2D2_PATHFINDING_ASTAR_HPP

#include <vector>
#include <algorithm>
#include <unordered_set>
#include <iostream>
#include <memory>

namespace r2d2 {
// value of 1,000,000 will support completely seaching a map of 1000x1000 nodes
#define MAX_SEARCH_NODES 1000000

    /**
     * abstract class to be implemented by the a star implementation
     *
     * represents a single node in the a star algorithm
     */
    template<typename T>
    class Node {
    public:
        /**
         * creates a new node with predefined values as the variables
         */
        Node(Length g = Length::METER * 0, Length h = Length::METER * 0,
             std::weak_ptr<T> parent = std::weak_ptr<T>()) :
                g{g},
                h{h},
                f{g + h},
                parent{parent} {
        }

        const Length g;
        Length h, f;
        const std::weak_ptr<T> parent;

        /**
         * checks if two nodes can be considered equal for the algorithm
         *
         * \return true if the two nodes are approximately equal
         */
        virtual bool operator==(const T &n) const = 0;

        /**
         * get the list of nodes that can be reached from this node
         *
         * \param self this
         */
        virtual std::vector<T> get_available_nodes(
                std::shared_ptr<T> &self) = 0;

        /**
         * compares two nodes for checking which should be evaluated first
         */
        bool operator>(const T &compare) const {
            return f > compare.f;
        }
    };

    /**
     * class used as an instantiation of a generic a star search
     *
     * this class exists because of the possibility of caching the result, or for
     * implementing things such as D* lite
     */
    template<typename T>
    class AStarSearch {
    public:
        // template functions have to be defined in the header itself
        /**
         * begins a new a star search
         *
         * the endpoint is to be given, because this cannot be changed at runtime
         * \param end the node the user wants to reach
         */
        AStarSearch(T &end) :
                closed{},
                open{} {
            std::shared_ptr<T> endPtr{std::make_shared<T>(end)};
            closed.emplace(endPtr);
            open.push_back(endPtr);
        }

        /**
         * start the actual search towards the start node
         *
         * \param start the node the user wants to reach the end from
         * \return a pointer to the newly created start node if it was found,
         *         otherwise return nullptr
         */
        std::shared_ptr<T> search(T &start) {
            int giveUpCount = MAX_SEARCH_NODES;

            while (!open.empty() && --giveUpCount >= 0) {
                std::shared_ptr<T> curOpen{open[0]};
                std::pop_heap(open.begin(), open.end(),
                              [](std::shared_ptr<T> &n1,
                                 std::shared_ptr<T> &n2) {
                                  return *n1 > *n2;
                              });
                open.pop_back();

                for (T &c : curOpen->get_available_nodes(curOpen)) {
                    std::shared_ptr<T> child{std::make_shared<T>(c)};

                    auto result = closed.emplace(child);
                    if (result.second) {
                        open.emplace_back(*result.first);
                        std::push_heap(
                                open.begin(), open.end(),
                                [](std::shared_ptr<T> &n1,
                                   std::shared_ptr<T> &n2) {
                                    return *n1 > *n2;
                                });
                    }
                    if (*child == start) {
                        return *closed.find(child);
                    }
                }
            }
            return nullptr;
        }

    private:
        /**
         * method used by the shared pointers for inserting into the set
         */
        struct TPtrEqual : std::binary_function<std::shared_ptr<T>,
                std::shared_ptr<T>, bool> {
            bool operator()(std::shared_ptr<T> n1,
                            std::shared_ptr<T> n2) const {
                return *n1 == *n2;
            }
        };

        /**
         * method used by the shared pointers for inserting into the set
         */
        struct THasher {
            std::size_t operator()(std::shared_ptr<T> n) const {
                return std::hash<T>()(*n);
            }
        };

        std::unordered_set<std::shared_ptr<T>, THasher, TPtrEqual> closed;
        std::vector<std::shared_ptr<T>> open;
    };
}

#endif //R2D2_PATHFINDING_ASTAR_HPP
