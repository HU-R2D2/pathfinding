//
// Created by chiel on 30-3-16.
//

#ifndef R2D2_PATHFINDING_ASTAR_HPP
#define R2D2_PATHFINDING_ASTAR_HPP

#include<vector>
#include<algorithm>
#include<unordered_set>

#define MAX_SEARCH_NODES 10000 // value has to be tested

template<typename T>
class Node {
public:
	Node(float g, float h, T *parent):
			g{ g },
			h{ h },
			f{ g + h },
			parent{ parent } {
	}

	float f, g, h;
	T* parent;

	virtual std::vector<T> getAvailableNodes() = 0;
	virtual bool operator==(const T &n) const = 0;
	bool operator<(Node & compare) {
		return f < compare.f;
	}
};

template<typename T>
class AStarSearch {
public:
	// template functions have to be defined in the header itself
	AStarSearch(T &end):
			open{ end },
			openSet{ &end },
			closed{} {
	}

	T* search(T &start) {
		int giveUpCount = MAX_SEARCH_NODES;

		while (!open.empty() && --giveUpCount >= 0) {
			T &curOpen = open[0];
			std::pop_heap(open.begin(), open.end());
			open.pop_back();
			openSet.erase(&curOpen);
			closed.emplace(curOpen);

			for (T &child : curOpen.getAvailableNodes()) {
				if (child == start) {
					return &child;
				}

				if (openSet.find(&child) == openSet.end()
				    && closed.find(child) == closed.end()) {
					open.emplace_back(child);
					std::push_heap(open.begin(), open.end());
				}
			}
		}
		return nullptr;
	}

private:
	std::vector<T> open;
	std::unordered_set<T*> openSet;
	std::unordered_set<T> closed;
};

#endif //R2D2_PATHFINDING_ASTAR_HPP
