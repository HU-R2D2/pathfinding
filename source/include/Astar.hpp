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
#include "PathFinder.hpp"

#define MAX_SEARCH_NODES 1000000 // value of 1,000,000 will support at most a map of 1000x1000 nodes

template<typename T>
class Node {
public:
	Node(float g, float h, std::weak_ptr<T> parent) :
			g{ g },
			h{ h },
			f{ g + h },
			parent{ parent } {
	}

	float g, h, f;
	std::weak_ptr<T> parent;

	virtual bool operator==(const T &n) const = 0;

	virtual std::vector<T> getAvailableNodes(std::shared_ptr<T> self) = 0;

	bool operator>(const T &compare) const {
		return f > compare.f;
	}
};

template<typename T>
class AStarSearch {
public:
	// template functions have to be defined in the header itself
	AStarSearch(T &end):
			closed{},
			open{} {
		std::shared_ptr<T> endPtr{std::make_shared<T>(end)};
		closed.emplace(endPtr);
		open.push_back(endPtr);
	}

	std::shared_ptr<T> search(T &start) {
		int giveUpCount = MAX_SEARCH_NODES;

		while (!open.empty() && --giveUpCount >= 0) {
			std::shared_ptr<T> curOpen{open[0]};
			std::pop_heap(open.begin(), open.end(),
			              [](std::shared_ptr<T> &n1, std::shared_ptr<T> &n2) { return *n1 > *n2; });
			open.pop_back();

			for (T &c : curOpen->getAvailableNodes(curOpen)) {
				std::shared_ptr<T> child{std::make_shared<T>(c)};

				auto result = closed.emplace(child);
				if (result.second) {
					open.emplace_back(*result.first);
					std::push_heap(open.begin(), open.end(),
					               [](std::shared_ptr<T> &n1, std::shared_ptr<T> &n2) { return *n1 > *n2; });
				}
				if (*child == start) {
					return *closed.find(child);
				}
			}
		}
		return nullptr;
	}

private:
	struct TPtrEqual : std::binary_function<std::shared_ptr<T>, std::shared_ptr<T>, bool> {
		bool operator()(std::shared_ptr<T> n1, std::shared_ptr<T> n2) const {
			return *n1 == *n2;
		}
	};

	struct THasher {
		std::size_t operator()(std::shared_ptr<T> n) const {
			return std::hash<T>()(*n);
		}
	};

	std::unordered_set<std::shared_ptr<T>, THasher, TPtrEqual> closed;
	std::vector<std::shared_ptr<T>> open;
};

#endif //R2D2_PATHFINDING_ASTAR_HPP
