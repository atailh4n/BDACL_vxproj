#pragma once
#include <iostream>
#include <optional>
#include <vector>
#include <unordered_map>
#include <queue>
#include <functional>
#include <limits>
#include <algorithm>
#include <unordered_set>

namespace KGL
{
	template <typename NodeType, typename CostType = float>
	class AStar
	{
	public:
		struct PathNode
		{
			NodeType id;
			CostType g_cost; // Cost from start
			CostType h_cost; // Heuristic cost to goal
			CostType f_cost; // g_cost + h_cost
			NodeType parent;

			PathNode(NodeType id, CostType g, CostType h, NodeType parent)
				: id(id), g_cost(g), h_cost(h), f_cost(g + h), parent(parent)
			{
			}

			// For priority queue comparison
			bool operator>(const PathNode& other) const
			{
				return f_cost > other.f_cost;
			}
		};

		using NodeList = std::vector<NodeType>;
		using HeuristicFunc = std::function<CostType(const NodeType&, const NodeType&)>;
		using NeighborFunc = std::function<std::vector<std::pair<NodeType, CostType>>(const NodeType&)>;

		std::optional<NodeList> findPath(
			const NodeType& start,
			const NodeType& goal,
			const NeighborFunc& getNeighbors,
			const HeuristicFunc& heuristic,
			CostType maxCost = std::numeric_limits<CostType>::max())
		{
			std::priority_queue<PathNode, std::vector<PathNode>, std::greater<PathNode>> openSet;
			std::unordered_map<NodeType, CostType> gCosts;
			std::unordered_map<NodeType, NodeType> cameFrom;
			std::unordered_set<NodeType> closedSet;

			// Add start node to open set
			openSet.emplace(start, CostType(0), heuristic(start, goal), start);
			gCosts[start] = CostType(0);

			while (!openSet.empty())
			{
				auto current = openSet.top();
				openSet.pop();

				// Skip if we already found a better path to this node
				if (closedSet.find(current.id) != closedSet.end())
				{
					continue;
				}

				// If we've reached the goal, reconstruct the path
				if (current.id == goal)
				{
					return reconstructPath(cameFrom, current.id, start);
				}

				closedSet.insert(current.id);

				// Check all neighbors
				for (const auto& [neighbor, cost] : getNeighbors(current.id))
				{
					// Skip already evaluated nodes
					if (closedSet.find(neighbor) != closedSet.end())
					{
						continue;
					}

					// Calculate new cost
					CostType tentativeGCost = gCosts[current.id] + cost;

					// Skip if cost exceeds maximum
					if (tentativeGCost > maxCost)
					{
						continue;
					}

					// If this path is better than any previous one, update
					if (gCosts.find(neighbor) == gCosts.end() || tentativeGCost < gCosts[neighbor])
					{
						cameFrom[neighbor] = current.id;
						gCosts[neighbor] = tentativeGCost;

						// Add to open set
						CostType hCost = heuristic(neighbor, goal);
						openSet.emplace(neighbor, tentativeGCost, hCost, current.id);
					}
				}
			}

			// No path found
			return std::nullopt;
		};

	private:
		// Reconstruct path from cameFrom map
		NodeList reconstructPath(
			const std::unordered_map<NodeType, NodeType>& cameFrom,
			const NodeType& current,
			const NodeType& start)
		{
			NodeList path;
			NodeType currentNode = current;

			while (currentNode != start)
			{
				path.push_back(currentNode);
				currentNode = cameFrom.at(currentNode);
			}

			path.push_back(start);
			std::reverse(path.begin(), path.end());

			return path;
		}
	};
}