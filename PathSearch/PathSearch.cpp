#include "PathSearch.h"
#include <iostream>
namespace fullsail_ai { namespace algorithms {

	PathSearch::PathSearch()
	{
	}

	PathSearch::~PathSearch()
	{
	}
    void PathSearch::buildSearchGraph()
    {
        // Clear existing graph
        for (auto& pair : nodes) {
            delete pair.second;
        }
        nodes.clear();

        // Build nodes for each valid tile
        for (int row = 0; row < tileMap->getRowCount(); row++) {
            for (int col = 0; col < tileMap->getColumnCount(); col++) {
                Tile* tile = tileMap->getTile(row, col);
                if (tile && tile->getWeight() > 0) {  // Skip obstacles
                    SearchNode* node = new SearchNode();
                    node->tile = tile;
                    nodes[tile] = node;
                }
            }
        }

        // Connect neighbors
        for (auto& pair : nodes) {
            Tile* tile = pair.first;
            SearchNode* node = pair.second;

            // Get valid neighbors for this tile
            std::vector<Tile*> neighborTiles = getValidNeighbors(tile);

            // Connect to neighbor nodes
            for (Tile* neighborTile : neighborTiles) {
                auto it = nodes.find(neighborTile);
                if (it != nodes.end()) {
                    node->neighbors.push_back(it->second);
                }
            }
        }
    }
    void PathSearch::outputSearchGraph() const
    {
        int count = 0;
        for (const auto& pair : nodes) {
            if (count >= 100) break;
            const Tile* tile = pair.first;
            const SearchNode* node = pair.second;

            std::cout << "Node " << count << " (Tile: [" << tile->getRow() << ", " << tile->getColumn() << "])\n";
            std::cout << "Neighbors:\n";
            for (const SearchNode* neighbor : node->neighbors) {
                const Tile* neighborTile = neighbor->tile;
                std::cout << "  - Tile: [" << neighborTile->getRow() << ", " << neighborTile->getColumn() << "]\n";
            }
            std::cout << std::endl;
            count++;
        }
    }

	void PathSearch::initialize(TileMap* _tileMap)
	{
		tileMap = _tileMap;
		buildSearchGraph();
      //  outputSearchGraph();
	}

	void PathSearch::enter(int startRow, int startColumn, int goalRow, int goalColumn)
	{
	}

	void PathSearch::update(long timeslice)
	{
	}

	void PathSearch::exit()
	{
	}
    void PathSearch::addNeighborsFromDirections(Tile* tile, const std::pair<int, int>* directions,
        int directionCount, std::vector<Tile*>& neighbors)
    {
        int row = tile->getRow();
        int col = tile->getColumn();

        for (int i = 0; i < directionCount; i++) {
            int newRow = row + directions[i].first;
            int newCol = col + directions[i].second;

            Tile* neighbor = tileMap->getTile(newRow, newCol);
            if (neighbor && neighbor->getWeight() > 0) {  // Valid tile and not an obstacle
                neighbors.push_back(neighbor);
            }
        }
    }
    std::vector<Tile*> PathSearch::getValidNeighbors(Tile* tile)
    {
        std::vector<Tile*> neighbors;
        int row = tile->getRow();

        // Define directions based on row parity (even/odd)
        if (row % 2 == 0) {
            // Directions for even rows
            const std::pair<int, int> directions[] = {
                {-1, -1}, // top-left
                {-1, 0},  // top-right
                {0, -1},  // left
                {0, 1},   // right
                {1, -1},  // bottom-left
                {1, 0}    // bottom-right
            };
            addNeighborsFromDirections(tile, directions, 6, neighbors);
        }
        else {
            // Directions for odd rows
            const std::pair<int, int> directions[] = {
                {-1, 0},  // top-left
                {-1, 1},  // top-right
                {0, -1},  // left
                {0, 1},   // right
                {1, 0},   // bottom-left
                {1, 1}    // bottom-right
            };
            addNeighborsFromDirections(tile, directions, 6, neighbors);
        }

        return neighbors;
    }

    void PathSearch::shutdown()
    {
        // Clean up the graph
        for (auto& pair : nodes) {
            delete pair.second;
        }
        nodes.clear();
        visited.clear();
        tileMap = nullptr;
    }

	bool PathSearch::isDone() const
	{
		return true;
	}

	std::vector<Tile const*> const PathSearch::getSolution() const
	{
		std::vector<Tile const*> temp;
		return temp;
	}
}}  // namespace fullsail_ai::algorithms

