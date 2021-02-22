#include "SingleAgentSolver.h"


list<int> SingleAgentSolver::getNextLocations(int curr) const // including itself and its neighbors
{
	list<int> rst = instance.getNeighbors(curr);
	rst.emplace_back(curr);
	return rst;
}
