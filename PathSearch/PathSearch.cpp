#include "PathSearch.h"
#include <iostream>
namespace fullsail_ai { namespace algorithms {
#define VISITED_COLOR  0xFF0000FF    // Red with full luminance
#define OPEN_COLOR    0xFF00FF00    // Green with full luminance
#define PATH_COLOR    0xFFFF0000    // Blue with full luminance
#define CURRENT_COLOR 0xFFFFFF00    // Yellow with full luminance
	PathSearch::PathSearch()
	{
	}

	PathSearch::~PathSearch()
	{
	}
    void PathSearch::buildSearchGraph()
    {
        for (auto& pair : nodes) {
            delete pair.second;
        }
        nodes.clear();

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

        for (auto& pair : nodes) {
            Tile* tile = pair.first;
            SearchNode* node = pair.second;

            std::vector<Tile*> neighborTiles = getValidNeighbors(tile);

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
        while (!openList.empty()) {
            openList.pop();
        }
        visited.clear();
        pathFound = false;

        Tile* startTile = tileMap->getTile(startRow, startColumn);
        goalTile = tileMap->getTile(goalRow, goalColumn);

        if (!startTile || !goalTile ||
            startTile->getWeight() == 0 ||
            goalTile->getWeight() == 0) {
            return;
        }

        auto startNodeIt = nodes.find(startTile);
        if (startNodeIt == nodes.end()) return;

        PlannerNode* start = new PlannerNode();
        start->searchNode = startNodeIt->second;
        start->parent = nullptr;

        openList.push(start);
        visited[startNodeIt->second] = start;

        startTile->setFill(OPEN_COLOR);
    }
    std::vector<Tile const*> PathSearch::reconstructPath(PlannerNode* goalNode) const
    {
        std::vector<Tile const*> path;
        PlannerNode* current = goalNode;

        while (current != nullptr) {
            path.push_back(current->searchNode->tile);
            current = current->parent;
        }

        std::reverse(path.begin(), path.end());
        return path;
    }

    void PathSearch::update(long timeslice)
    {
        if (pathFound || openList.empty()) return;

        PlannerNode* current = openList.front();
        openList.pop();

        // Visual debugging - mark as visited
        current->searchNode->tile->setFill(VISITED_COLOR);

        auto currentPath = reconstructPath(current);
        for (size_t i = 0; i < currentPath.size() - 1; ++i) {
            const_cast<Tile*>(currentPath[i])->addLineTo(
                const_cast<Tile*>(currentPath[i + 1]),
                VISITED_COLOR  // Using red color for the current path
            );
        }

        if (current->searchNode->tile == goalTile) {
            pathFound = true;
            auto path = reconstructPath(current);

            for (size_t i = 0; i < path.size(); ++i) {
                const_cast<Tile*>(path[i])->setFill(PATH_COLOR);

                if (i < path.size() - 1) {
                    const_cast<Tile*>(path[i])->addLineTo(
                        const_cast<Tile*>(path[i + 1]),
                        PATH_COLOR
                    );
                }
            }
            return;
        }

        for (SearchNode* neighborNode : current->searchNode->neighbors) {
            if (visited.find(neighborNode) != visited.end()) {
                continue;
            }

            PlannerNode* newNode = new PlannerNode();
            newNode->searchNode = neighborNode;
            newNode->parent = current;

            openList.push(newNode);
            visited[neighborNode] = newNode;

            neighborNode->tile->setFill(OPEN_COLOR);

        }
    }
    void PathSearch::exit()
    {
        while (!openList.empty()) {
            delete openList.front();
            openList.pop();
        }

        pathFound = false;
        goalTile = nullptr;
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

        if (row % 2 == 0) {
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
        for (auto& pair : nodes) {
            delete pair.second;
        }
        nodes.clear();

        for (auto& pair : visited) {
            delete pair.second;
        }
        visited.clear();

        tileMap = nullptr;
        goalTile = nullptr;
        pathFound = false;
    }


    bool PathSearch::isDone() const
    {
        return pathFound || openList.empty();
    }

    std::vector<Tile const*> const PathSearch::getSolution() const
    {
        if (!pathFound) return std::vector<Tile const*>();

        auto goalNodeIt = visited.find(nodes.at(goalTile));
        if (goalNodeIt == visited.end()) return std::vector<Tile const*>();

        return reconstructPath(goalNodeIt->second);
    }
}}  // namespace fullsail_ai::algorithms

