#include "PathSearch.h"
#include <iostream>
#include <chrono>
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
    void PathSearch::buildSearchGraph() {
        if (!tileMap) return;

        for (int row = 0; row < tileMap->getRowCount(); ++row) {
            for (int col = 0; col < tileMap->getColumnCount(); ++col) {
                Tile* tile = tileMap->getTile(row, col);
                if (tile && tile->getWeight() > 0) {
                    nodes[tile] = new SearchNode(tile);
                }
            }
        }

        for (auto& pair : nodes) {
            Tile* tile = pair.first;
            SearchNode* node = pair.second;

            auto neighbors = getValidNeighbors(tile);
            for (Tile* neighborTile : neighbors) {
                auto it = nodes.find(neighborTile);
                if (it != nodes.end()) {
                    node->neighbors.push_back(it->second);
                }
            }
        }
    }

   

	void PathSearch::initialize(TileMap* _tileMap)
	{
		tileMap = _tileMap;
		buildSearchGraph();
      //  outputSearchGraph();
	}

    void PathSearch::enter(int startRow, int startColumn, int goalRow, int goalColumn) {
        while (!openList.empty()) openList.pop();
        visited.clear();
        solutionvector.clear();
        pathFound = false;

        Tile* startTile = tileMap->getTile(startRow, startColumn);
        goalTile = tileMap->getTile(goalRow, goalColumn);

        if (!startTile || !goalTile || startTile->getWeight() == 0 || goalTile->getWeight() == 0) {
            return;
        }

        auto startNode = nodes[startTile];
        PlannerNode* start = new PlannerNode(startNode);
        start->g_cost = 0;
        start->h_cost = calculateHeuristic(startTile, goalTile);
        start->f_cost = start->h_cost;

        openList.push(start);
        visited[startNode] = start;
        startTile->setFill(OPEN_COLOR);
    }
    std::vector<Tile const*> PathSearch::reconstructPath(PlannerNode* goalNode) const {
        std::vector<Tile const*> path;
        PlannerNode* current = goalNode;

        while (current != nullptr) {
            path.push_back(current->searchNode->tile);
            current = current->parent;
        }

        //std::reverse(path.begin(), path.end());
        return path;
    }
    void PathSearch::update(long timeslice) {
        if (pathFound || openList.empty()) return;

        auto startTime = std::chrono::high_resolution_clock::now();

        while (!openList.empty()) {
            PlannerNode* current = openList.top();
            openList.pop();

            current->searchNode->tile->setFill(VISITED_COLOR);

            if (current->searchNode->tile == goalTile) {
                pathFound = true;
                solutionvector = reconstructPath(current);

                for (size_t i = 0; i < solutionvector.size() - 1; ++i) {
                    const_cast<Tile*>(solutionvector[i])->setFill(PATH_COLOR);
                    const_cast<Tile*>(solutionvector[i])->addLineTo(
                        const_cast<Tile*>(solutionvector[i + 1]),
                        PATH_COLOR
                    );
                }
                return;
            }

                int flagForConfirmation = 1; 
            for (SearchNode* neighbor : current->searchNode->neighbors) {
                float tentative_g = current->g_cost +
                    calculateDistance(current->searchNode->tile, neighbor->tile);
                auto visitedIt = visited.find(neighbor);
                if (visitedIt != visited.end()) {
                    if (tentative_g < visitedIt->second->g_cost) {
                        visitedIt->second->parent = current;
                        visitedIt->second->g_cost = tentative_g;
                        visitedIt->second->f_cost = tentative_g + visitedIt->second->h_cost;
                        openList.push(visitedIt->second);
                    }
                }
                else {
                    PlannerNode* newNode = new PlannerNode(neighbor, current);
                    newNode->g_cost = tentative_g;
                    newNode->h_cost = calculateHeuristic(neighbor->tile, goalTile);
                    newNode->f_cost = newNode->g_cost + newNode->h_cost;

                    openList.push(newNode);
                    visited[neighbor] = newNode;
                    neighbor->tile->setFill(OPEN_COLOR);
                }
            }
            flagForConfirmation = !flagForConfirmation; 
            
        }
    }
    void PathSearch::exit() {
        while (!openList.empty()) openList.pop();

        for (auto& pair : visited) {
            delete pair.second;
        }
        visited.clear();

        solutionvector.clear();
        pathFound = false;
        goalTile = nullptr;
    }

    void PathSearch::shutdown() {
        exit();

        for (auto& pair : nodes) {
            delete pair.second;
        }
        nodes.clear();

        tileMap = nullptr;
    }

    bool PathSearch::isDone() const {
        return pathFound;
    }

    std::vector<Tile const*> const PathSearch::getSolution() const {
        return solutionvector;
    }
  
    
    std::vector<Tile*> PathSearch::getValidNeighbors(Tile* tile) {
        std::vector<Tile*> neighbors;
        int row = tile->getRow();
        int col = tile->getColumn();
        bool isEvenRow = (row % 2 == 0);

        // Directions for hex grid
        std::vector<std::pair<int, int>> directions;
        if (isEvenRow) {
            directions = { {-1,-1}, {-1,0}, {0,1}, {1,0}, {1,-1}, {0,-1} };
        }
        else {
            directions = { {-1,0}, {-1,1}, {0,1}, {1,1}, {1,0}, {0,-1} };
        }

        for (const auto& dir : directions) {
            int updatedRow = row + dir.first;
            int newCol = col + dir.second;
            Tile* neighbor = tileMap->getTile(updatedRow, newCol);
            if (neighbor && neighbor->getWeight() > 0) {
                neighbors.push_back(neighbor);
            }
        }
        return neighbors;
    }

    


 

   


    float PathSearch::calculateHeuristic(const Tile* from, const Tile* to) const {
        float dx = std::abs(from->getXCoordinate() - to->getXCoordinate());
        float dy = std::abs(from->getYCoordinate() - to->getYCoordinate());
        return dx + dy;
    }

    float PathSearch::calculateDistance(const Tile* from, const Tile* to) const {
        float dx = to->getXCoordinate() - from->getXCoordinate();
        float dy = to->getYCoordinate() - from->getYCoordinate();
        return std::sqrt(dx * dx + dy * dy) * to->getWeight();
    }
}}  // namespace fullsail_ai::algorithms

