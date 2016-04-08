//
// Created by chiel on 30-3-16.
//

#ifndef R2D2_PATHFINDING_ASTAR_HPP
#define R2D2_PATHFINDING_ASTAR_HPP

#include <vector>
#include <algorithm>
#include <unordered_set>
#include <iostream>
#include <memory>

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
	     std::weak_ptr<T> parent = std::weak_ptr<T>()):
			g{ g },
			h{ h },
			f{ g + h },
			parent{ parent } {
	}

	const Length g;
	Length h, f;
	const std::weak_ptr<T> parent;

	/**
	 * checks if two nodes can be considered equal for the algorithm
	 *
	 * /return true if the two nodes are approximately equal
	 */
	virtual bool operator==(const T &n) const = 0;

	/**
	 * get the list of nodes that can be reached from this node
	 *
	 * /param self this
	 */
	virtual std::vector<T> get_available_nodes(std::shared_ptr<T> &self) = 0;

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
	 * /param end the node the user wants to reach
	 */
	AStarSearch(T &end):
			closed{},
			open{} {
		std::shared_ptr<T> endPtr{std::make_shared<T>(end)};
		closed.emplace(endPtr);
		open.push_back(endPtr);
	}

	/**
	 * start the actual search towards the start node
	 *
	 * /param start the node the user wants to reach the end from
	 * /return a pointer to the newly created start node if it was found,
	 *         otherwise return nullptr
	 */
	std::shared_ptr<T> search(T &start) {
		int giveUpCount = MAX_SEARCH_NODES;

		while (!open.empty() && --giveUpCount >= 0) {
			std::shared_ptr<T> curOpen{open[0]};
			std::pop_heap(open.begin(), open.end(),
			              [](std::shared_ptr<T> &n1, std::shared_ptr<T> &n2) {
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
							[](std::shared_ptr<T> &n1, std::shared_ptr<T> &n2) {
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
		bool operator()(std::shared_ptr<T> n1, std::shared_ptr<T> n2) const {
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

#endif //R2D2_PATHFINDING_ASTAR_HPP
