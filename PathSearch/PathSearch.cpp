#include "PathSearch.h"

namespace fullsail_ai { namespace algorithms {

	PathSearch::PathSearch()
	{
	}

	PathSearch::~PathSearch()
	{
	}

	void PathSearch::initialize(TileMap* _tileMap)
	{
		tileMap = _tileMap;
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

	void PathSearch::shutdown()
	{
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

