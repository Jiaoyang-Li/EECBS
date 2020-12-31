//#pragma warning(disable: 4996) //Jiaoyang: I added this line to disable error C4996 caused by CPLEX
#include "CBSHeuristic.h"
#include "CBS.h"
#include <queue>
//#include <ilcplex/ilocplex.h>


void CBSHeuristic::updateInadmissibleHeuristics(HLNode& curr)
{
	int h = curr.getName() == "CBS Node"? curr.h_val : 0;
	double cost_error = 0, distance_error = 0;
	double c;
	switch (inadmissible_heuristic)
	{
	case heuristics_type::PATH:
		// update errors along the path to the node
		num_of_errors[0] = 0;
		sum_distance_errors[0] = 0;
		sum_cost_errors[0] = 0;
		for (auto ptr = curr.parent; ptr  != nullptr; ptr = ptr->parent)
		{
			if (ptr->fully_expanded)
			{
				num_of_errors[0]++;
				sum_distance_errors[0] += ptr->distance_error;
				sum_cost_errors[0] += ptr->cost_error;
			}
		}
	case heuristics_type::GLOBAL: // Note: there is no "break" in the previous line, so here we compute heuristics for both GLOBAL and PATH
		if (num_of_errors[0] < 1)
		{
			curr.cost_to_go = max(0, curr.getFVal() - curr.getFHatVal()); // ensure that f <= f^
			return;
		}
		if (num_of_errors[0] <= sum_distance_errors[0])
		{
			c =  sum_cost_errors[0] / num_of_errors[0] * 10;
		}
		else
		{
			c = sum_cost_errors[0] / (num_of_errors[0] - sum_distance_errors[0]);
		}
		curr.cost_to_go = h + (int)(curr.distance_to_go * c);
		break;
	case heuristics_type::LOCAL:
		if (std::abs(1 - sum_distance_errors[0]) < 0.001)
			curr.cost_to_go = h + max(0, (int)(curr.distance_to_go) * 1000);
		else
			curr.cost_to_go = h + max(0, (int)(curr.distance_to_go * sum_cost_errors[0] / (1 - sum_distance_errors[0])));
		/*if (num_of_errors % 100 == 0)
		cout << std::setprecision(3)
		<< sum_cost_error  << ","
		<< sum_distance_error << ","
		<< sum_cost_error  / (1 - sum_distance_error) << endl;*/
		break;
	case heuristics_type::CONFLICT:
		if (curr.conflicts.empty() && curr.unknownConf.empty())
			return;
		for (const auto& conflict : curr.conflicts)
		{
			int id = conflict->getConflictId();
			cost_error += getCostError(id);
			distance_error += getDistanceError(id);
		}

		/*set<pair<int, int>> conflicting_agents;
		for (const auto& conflict : curr.unknownConf)
		{
			auto agents = make_pair(min(conflict->a1, conflict->a2), max(conflict->a1, conflict->a2));
			if (conflicting_agents.find(agents) != conflicting_agents.end())
				continue; // we do not recount the conflict between the same pair of agents
			int id = conflict->getConflictId();
			cost_error += getCostError(id);
			distance_error += getDistanceError(id);
		}*/
		cost_error /= (double)(curr.conflicts.size()); // + conflicting_agents.size());
		distance_error /= (double)(curr.conflicts.size()); // + conflicting_agents.size());
		// curr.cost_to_go = h + (int)(curr.distance_to_go * cost_error);
		if ( distance_error >= 1)
			curr.cost_to_go = (int)(curr.distance_to_go * cost_error);
		else
			curr.cost_to_go = (int)(curr.distance_to_go * cost_error / (1 - distance_error));
		// cout << std::setprecision(3) << (double)curr.cost_to_go / curr.distance_to_go << ",";
		curr.cost_to_go = max(min(MAX_COST, curr.cost_to_go), 0);
		curr.cost_to_go += h;
		break;
	default:
		break;
	}
    if (curr.getFVal() > curr.getFHatVal())
        curr.cost_to_go += curr.getFVal() - curr.getFHatVal(); // ensure that f <= f^
}


void CBSHeuristic::updateOnlineHeuristicErrors(CBSNode& curr)
{
    if ((inadmissible_heuristic == heuristics_type::GLOBAL ||
         inadmissible_heuristic == heuristics_type::PATH ||
         inadmissible_heuristic == heuristics_type::LOCAL ||
         inadmissible_heuristic == heuristics_type::CONFLICT) && curr.parent != nullptr)
	{
		curr.parent->fully_expanded = true;
		HLNode* best = &curr;
		for (auto child : curr.parent->children)
		{
			if (!child->h_computed)
			{
				curr.parent->fully_expanded = false;
				break;
			}
			if (best->getFVal() > child->getFVal() ||
				(best->getFVal() == child->getFVal() && best->distance_to_go > child->distance_to_go))
				best = child;
		}
		if (curr.parent->fully_expanded) // update error
		{
			curr.parent->distance_error = 1 + best->distance_to_go - curr.parent->distance_to_go;
			curr.parent->cost_error = best->g_val + best->h_val - curr.parent->g_val - curr.parent->h_val;
			if (inadmissible_heuristic == heuristics_type::GLOBAL) // update global error
			{
				sum_distance_errors[0] += curr.parent->distance_error;
				sum_cost_errors[0] += curr.parent->cost_error;
				num_of_errors[0]++;
			}
			else if (inadmissible_heuristic == heuristics_type::LOCAL) // update local error
			{
				num_of_errors[0]++;
				double learning_rate = 0.001;
				if (num_of_errors[0] * learning_rate < 1)
				{
					learning_rate = 1.0 / num_of_errors[0];
				}
				sum_distance_errors[0] = sum_distance_errors[0] *(1 - learning_rate) + curr.parent->distance_error * learning_rate;
				sum_cost_errors[0] = sum_cost_errors[0] * (1 - learning_rate) + curr.parent->cost_error * learning_rate;
			}
			else if (inadmissible_heuristic == heuristics_type::CONFLICT) // update conflict error
            {
			    int id = curr.parent->conflict->getConflictId();
                sum_distance_errors[id] += curr.parent->distance_error;
                sum_cost_errors[id] += curr.parent->cost_error;
                num_of_errors[id]++;
            }
		}
	}
}


void CBSHeuristic::updateOnlineHeuristicErrors(ECBSNode& parent)
{
    if (inadmissible_heuristic == heuristics_type::ZERO)
        return;
	// Find the best child
	const HLNode* best_child = parent.children.front();
	assert(parent.children.size() <= 2);
	if (parent.children.size() == 2)
	{
		const HLNode* other = parent.children.back();
		if (best_child->getFHatVal() > other->getFHatVal() ||
			(best_child->getFHatVal() == other->getFHatVal() && best_child->distance_to_go > other->distance_to_go))
			best_child = other;
	}
    // Update the errors
    parent.distance_error = 1 + best_child->distance_to_go - parent.distance_to_go;
    parent.cost_error = best_child->getFHatVal() - best_child->cost_to_go  - parent.sum_of_costs;
    if (inadmissible_heuristic == heuristics_type::GLOBAL) // update global error
    {
        sum_distance_errors[0] += parent.distance_error;
        sum_cost_errors[0] += parent.cost_error;
        num_of_errors[0]++;
    }
    else if (inadmissible_heuristic == heuristics_type::LOCAL) // update local error
    {
        num_of_errors[0]++;
        double learning_rate = 0.001;
        if (num_of_errors[0] * learning_rate < 1)
        {
            learning_rate = 1.0 / num_of_errors[0];
        }
        sum_distance_errors[0] = sum_distance_errors[0] *(1 - learning_rate) + parent.distance_error * learning_rate;
        sum_cost_errors[0] = sum_cost_errors[0] * (1 - learning_rate) + parent.cost_error * learning_rate;
    }
    else if (inadmissible_heuristic == heuristics_type::CONFLICT) // update conflict error
    {
        int id = parent.conflict->getConflictId();
        sum_cost_errors[id] += parent.cost_error;
        num_of_errors[id]++;
        sum_distance_errors[id] += parent.distance_error;
    }
}


void CBSHeuristic::computeQuickHeuristics(HLNode& node)
{
	/*if (parent->h_val  == 0);
	else if (parent->conflictGraph.empty())
	{
	node->h_val = parent->h_val - 1; // stronger pathmax
	}
	else
	{
	int maxWeight = 0;
	boost::unordered_map<int, int>::iterator got;
	for (auto e : parent->conflictGraph)
	{
	if ((e.first / num_of_agents == agent || e.first % num_of_agents == agent) && e.second > maxWeight)
	{
	maxWeight = e.second;
	if (maxWeight >= parent->h_val)
	break;
	}
	}
	if (maxWeight < parent->h_val)
	node->h_val = parent->h_val - maxWeight; // stronger pathmax
	}*/
	if (node.parent != nullptr)
	    node.h_val = max(0, node.parent->g_val + node.parent->h_val - node.g_val); // pathmax

    //vector<bool> HG(num_of_agents * num_of_agents, false);
    //buildConflictGraph(HG, node);
    //node.distance_to_go = greedyMatching(HG, num_of_agents);
	node.updateDistanceToGo();
    if (node.parent != nullptr)
	    updateInadmissibleHeuristics(node); // compute inadmissible heuristics
	// node.cost_to_go = max(node.getFVal() - node.getFHatVal(), 0);
	// copyConflictGraph(node, *node.parent);
}

bool CBSHeuristic::computeInformedHeuristics(CBSNode& curr, double _time_limit)
{
    curr.h_computed = true;
	// create conflict graph
	start_time = clock();
	this->time_limit = _time_limit;
	int num_of_CGedges;
	vector<int> HG(num_of_agents * num_of_agents, 0); // heuristic graph
	int h = -1;

	// compute admissible heuristics
	switch (type)
	{
	case heuristics_type::ZERO:
		h = 0;
		break;
	case heuristics_type::CG:
		buildCardinalConflictGraph(curr, HG, num_of_CGedges);
		// Minimum Vertex Cover
		if (curr.parent == nullptr || num_of_CGedges > ILP_edge_threshold || // root node of CBS tree or the graph is too large
			target_reasoning || disjoint_splitting) // when we are allowed to replan for multiple agents, the incremental method is not correct any longer.
			h = minimumVertexCover(HG);
		else
			h = minimumVertexCover(HG, curr.parent->h_val, num_of_agents, num_of_CGedges);
		break;
	case heuristics_type::DG:
		if (!buildDependenceGraph(curr, HG, num_of_CGedges))
			return false;
		// Minimum Vertex Cover
		if (curr.parent == nullptr || num_of_CGedges > ILP_edge_threshold || // root node of CBS tree or the graph is too large
			target_reasoning || disjoint_splitting) // when we are allowed to replan for multiple agents, the incremental method is not correct any longer.
			h = minimumVertexCover(HG);
		else
			h = minimumVertexCover(HG, curr.parent->h_val, num_of_agents, num_of_CGedges);
		break;
	case heuristics_type::WDG:
		if (!buildWeightedDependencyGraph(curr, HG))
			return false;
		h = minimumWeightedVertexCover(HG);
		break;
	default:
		break;
	}
	if (h < 0)
		return false;
	curr.h_val = max(h, curr.h_val);

	/*if (type == heuristics_type::WDG) //compute distance-to-go heuristics
	{
		curr.distance_to_go = 0;
		for (const auto& edge : curr.conflictGraph)
		{
			curr.distance_to_go += edge.second.second;
		}
	}
	if (screen == 2)
		curr.printConflictGraph(num_of_agents);*/
	return true;
}


bool CBSHeuristic::computeInformedHeuristics(ECBSNode& curr, const vector<int>& min_f_vals, double _time_limit)
{
    curr.h_computed = true;
	// create conflict graph
	start_time = clock();
	this->time_limit = _time_limit;
	int num_of_CGedges;
	vector<int> HG(num_of_agents * num_of_agents, 0); // heuristic graph
	int h = -1;

	/* compute admissible heuristics */
	switch (type)
	{
	case heuristics_type::ZERO:
		h = 0;
		break;
	//TODO:: add CG and DG for ECBS
	/*case heuristics_type::CG:
		buildCardinalConflictGraph(curr, HG, num_of_CGedges);
		// Minimum Vertex Cover
		if (curr.parent == nullptr || num_of_CGedges > ILP_edge_threshold) // root node of CBS tree or the graph is too large
			h = minimumVertexCover(HG);
		else
			h = minimumVertexCover(HG, curr.parent->h_val, num_of_agents, num_of_CGedges);
		break;
	case heuristics_type::DG:
		if (!buildDependenceGraph(curr, HG, num_of_CGedges))
			return false;
		// Minimum Vertex Cover
		if (curr.parent == nullptr || num_of_CGedges > ILP_edge_threshold) // root node of CBS tree or the graph is too large
			h = minimumVertexCover(HG);
		else
			h = minimumVertexCover(HG, curr.parent->h_val, num_of_agents, num_of_CGedges);
		break;*/
	case heuristics_type::WDG:
	    int delta_g;
		if (!buildWeightedDependencyGraph(curr, min_f_vals, HG, delta_g))
			return false;
		assert(delta_g >= 0);
		// cout << curr.g_val << "+" << delta_g << endl;
		h = minimumWeightedVertexCover(HG) + delta_g;

		break;
	default:
		cerr << "ERROR in computing informed heurisctis" << endl;
	}
	if (h < 0)
		return false;
	curr.h_val = max(h, curr.h_val);
    curr.cost_to_go = max(curr.cost_to_go, curr.getFVal() - curr.sum_of_costs); // ensure that f <= f^
	//if (screen == 2)
	//	curr.printConflictGraph(num_of_agents);

	return true;
}


/*int CBSHeuristic::MVConAllConflicts(CBSNode& curr)
{
	auto G = buildConflictGraph(curr);
	return  minimumVertexCover(G);
}*/

/*int CBSHeuristic::minimumConstrainedWeightedVertexCover(const vector<int>& HG)
{
	int rst = 0;
	std::vector<bool> done(num_of_agents, false);
	for (int i = 0; i < num_of_agents; i++)
	{
		if (done[i])
			continue;
		std::vector<int> range;
		std::vector<int> indices;
		range.reserve(num_of_agents);
		indices.reserve(num_of_agents);
		int num = 0;
		std::queue<int> Q;
		Q.push(i);
		done[i] = true;
		while (!Q.empty())
		{
			int j = Q.front(); Q.pop();
			range.push_back(0);
			indices.push_back(j);
			for (int k = 0; k < num_of_agents; k++)
			{
				if (HG[j * num_of_agents + k] > 0)
				{
					range[num] = std::max(range[num], HG[j * num_of_agents + k]);
					if (!done[k])
					{
						Q.push(k);
						done[k] = true;
					}
				}
				else if (HG[k * num_of_agents + j] > 0)
				{
					range[num] = std::max(range[num], HG[k * num_of_agents + j]);
					if (!done[k])
					{
						Q.push(k);
						done[k] = true;
					}
				}
			}
			num++;
		}
		if (num == 1) // no edges
			continue;
		else if (num == 2) // only one edge
		{
			rst += std::max(HG[indices[0] * num_of_agents + indices[1]], HG[indices[1] * num_of_agents + indices[0]]); // add edge weight
			continue;
		}
		std::vector<int> G(num * num, 0);
		for (int j = 0; j < num; j++)
		{
			for (int k = j + 1; k < num; k++)
			{
				G[j * num + k] = std::max(HG[indices[j] * num_of_agents + indices[k]], HG[indices[k] * num_of_agents + indices[j]]);
			}
		}
		if (num > ILP_node_threshold)
		{
			rst += ILPForConstrainedWMVC(G, range);
		}
		else
		{
			std::vector<bool> x(num, false);
			int best_so_far = INT_MAX;
			rst += DPForConstrainedWMVC(x, 0, 0, G, range, best_so_far);
		}
		
		double runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
		if (runtime > time_limit)
			return -1; // run out of time
	}
	return rst;
}
*/
/*int CBSHeuristic::greedyWDG(CBSNode& curr, double time_limit)
{
	this->start_time = clock();
	this->time_limit = time_limit;
	vector<int> HG(num_of_agents * num_of_agents, 0); // heuristic graph
	buildWeightedDependencyGraph(curr, HG);
	return minimumConstrainedWeightedVertexCover(HG);
}*/


/*void CBSHeuristic::buildConflictGraph(vector<bool>& G, const HLNode& curr)
{
	for (const auto& conflict : curr.conflicts)
	{
		int a1 = conflict->a1;
		int a2 = conflict->a2;
		if (!G[a1 * num_of_agents + a2])
		{
			G[a1 * num_of_agents + a2] = true;
			G[a2 * num_of_agents + a1] = true;
		}
	}
    for (const auto& conflict : curr.unknownConf)
    {
        int a1 = conflict->a1;
        int a2 = conflict->a2;
        if (!G[a1 * num_of_agents + a2])
        {
            G[a1 * num_of_agents + a2] = true;
            G[a2 * num_of_agents + a1] = true;
        }
    }
}*/


void CBSHeuristic::buildCardinalConflictGraph(CBSNode& curr, vector<int>& CG, int& num_of_CGedges)
{
	num_of_CGedges = 0;
	for (const auto& conflict : curr.conflicts)
	{
		if (conflict->priority == conflict_priority::CARDINAL)
		{
			int a1 = conflict->a1;
			int a2 = conflict->a2;
			if (!CG[a1 * num_of_agents + a2])
			{
				CG[a1 * num_of_agents + a2] = true;
				CG[a2 * num_of_agents + a1] = true;
				num_of_CGedges++;
			}
		}
	}
	runtime_build_dependency_graph += (double)(clock() - start_time) / CLOCKS_PER_SEC;
}


bool CBSHeuristic::buildDependenceGraph(CBSNode& node, vector<int>& CG, int& num_of_CGedges)
{
    num_of_CGedges = 0;
	for (auto& conflict : node.conflicts)
	{
		int a1 = min(conflict->a1, conflict->a2);
		int a2 = max(conflict->a1, conflict->a2);
		int idx = a1 * num_of_agents + a2;
        if (CG[a1 * num_of_agents + a2] > 0)
            continue;
		if (conflict->priority == conflict_priority::CARDINAL)
		{
            CG[a1 * num_of_agents + a2] = 1;
            CG[a2 * num_of_agents + a1] = 1;
            num_of_CGedges++;
            continue;
		}
        auto got = lookupTable[a1][a2].find(HTableEntry(a1, a2, &node));
        if (got != lookupTable[a1][a2].end()) // check the lookup table first
        {
            CG[idx] = get<0>(got->second) > 0 ? 1 : 0;
            CG[a2 * num_of_agents + a1] = CG[idx];
            num_memoization++;
        }
        else
        {
            CG[idx] = dependent(a1, a2, node)? 1 : 0;
            CG[a2 * num_of_agents + a1] = CG[idx];
            lookupTable[a1][a2][HTableEntry(a1, a2, &node)] = make_tuple(CG[idx], 1, 0);
            if ((clock() - start_time) / CLOCKS_PER_SEC > time_limit) // run out of time
            {
                runtime_build_dependency_graph += (double)(clock() - start_time) / CLOCKS_PER_SEC;
                return false;
            }
        }
		if (CG[idx])
		{
            num_of_CGedges++;
			conflict->priority = conflict_priority::PSEUDO_CARDINAL; // the two agents are dependent, although resolving this conflict might not increase the cost
		}
	}
	runtime_build_dependency_graph += (double)(clock() - start_time) / CLOCKS_PER_SEC;
	return true;
}


bool CBSHeuristic::buildWeightedDependencyGraph(CBSNode& node, vector<int>& CG)
{
	for (const auto& conflict : node.conflicts)
	{
		int a1 = min(conflict->a1, conflict->a2);
		int a2 = max(conflict->a1, conflict->a2);
		int idx = a1 * num_of_agents + a2;
		auto got = lookupTable[a1][a2].find(HTableEntry(a1, a2, &node));
		if (got != lookupTable[a1][a2].end()) // check the lookup table first
		{
			num_memoization++;
            CG[idx] = get<0>(got->second);
            CG[a2 * num_of_agents + a1] = CG[idx];
		}
		else if (rectangle_reasoning)
		{
			auto rst = solve2Agents(a1, a2, node, false);
			assert(rst.first >= 0);
			lookupTable[a1][a2][HTableEntry(a1, a2, &node)] = make_tuple(rst.first, rst.second, 1);
			CG[idx] = rst.first;
			CG[a2 * num_of_agents + a1] = rst.first;
		}
		else
		{
			bool cardinal = conflict->priority == conflict_priority::CARDINAL;
			if (!cardinal && !mutex_reasoning) // using merging MDD methods before runing 2-agent instance
			{
				cardinal = dependent(a1, a2, node);
			}
			if (cardinal) // run 2-agent solver only for dependent agents
			{
				auto rst = solve2Agents(a1, a2, node, cardinal);
				assert(rst.first >= 1);
				lookupTable[a1][a2][HTableEntry(a1, a2, &node)] = make_tuple(rst.first, rst.second, 1);
				CG[idx] = rst.first;
				CG[a2 * num_of_agents + a1] = rst.first;
			}
			else
			{
				lookupTable[a1][a2][HTableEntry(a1, a2, &node)]  = make_tuple(0, 1, 0); // h=0, #CT nodes = 1
				CG[idx] = 0;
				CG[a2 * num_of_agents + a1] = 0;
			}
		}
		if ((clock() - start_time) / CLOCKS_PER_SEC > time_limit) // run out of time
		{
			runtime_build_dependency_graph += (double)(clock() - start_time) / CLOCKS_PER_SEC;
			return false;
		}
		if (CG[idx] == MAX_COST) // no solution
		{
            return false;
		}
		if (conflict->priority != conflict_priority::CARDINAL && CG[idx] > 0)
		{
			conflict->priority = conflict_priority::PSEUDO_CARDINAL; // the two agents are dependent, although resolving this conflict might not increase the cost
		}
	}

	runtime_build_dependency_graph += (double)(clock() - start_time) / CLOCKS_PER_SEC;
	return true;
}


bool CBSHeuristic::buildWeightedDependencyGraph(ECBSNode& node, const vector<int>& min_f_vals, vector<int>& CG, int& delta_g)
{
    delta_g = 0;
    vector<bool> counted(num_of_agents, false); // record the agents whose delta_g has been counted
	for (const auto& conflict : node.conflicts)
	{
		int a1 = min(conflict->a1, conflict->a2);
		int a2 = max(conflict->a1, conflict->a2);
		int idx = a1 * num_of_agents + a2;
		auto got = lookupTable[a1][a2].find(HTableEntry(a1, a2, &node));
		if (got != lookupTable[a1][a2].end()) // check the lookup table first
		{
			num_memoization++;
            CG[idx]  = get<0>(got->second);
            CG[a2 * num_of_agents + a1] = CG[idx];
            if (!counted[a1])
            {
                assert(get<1>(got->second) >= min_f_vals[a1]);
                delta_g += get<1>(got->second) - min_f_vals[a1];
                counted[a1] = true;
            }
            if (!counted[a2])
            {
                assert(get<2>(got->second) >= min_f_vals[a2]);
                delta_g += get<2>(got->second) - min_f_vals[a2];
                counted[a2] = true;
            }
		}
		else
		{
			auto rst = solve2Agents(a1, a2, node);
            lookupTable[a1][a2][HTableEntry(a1, a2, &node)] = rst;
            if ((clock() - start_time) / CLOCKS_PER_SEC > time_limit) // run out of time
            {
                runtime_build_dependency_graph += (double)(clock() - start_time) / CLOCKS_PER_SEC;
                return false;
            }
            CG[idx]  = get<0>(rst);
            CG[a2 * num_of_agents + a1] = CG[idx];
            if (!counted[a1])
            {
                assert(get<1>(rst) >= min_f_vals[a1]);
                delta_g += get<1>(rst) - min_f_vals[a1];
                counted[a1] = true;
            }
            if (!counted[a2])
            {
                assert(get<2>(rst) >= min_f_vals[a2]);
                delta_g += get<2>(rst) - min_f_vals[a2];
                counted[a2] = true;
            }
		}

        if (CG[idx] == MAX_COST) // no solution
            return false;
	}
    for (const auto& conflict : node.unknownConf)
    {
        int a1 = min(conflict->a1, conflict->a2);
        int a2 = max(conflict->a1, conflict->a2);
        int idx = a1 * num_of_agents + a2;
        auto got = lookupTable[a1][a2].find(HTableEntry(a1, a2, &node));
        if (got != lookupTable[a1][a2].end()) // check the lookup table first
        {
            num_memoization++;
            CG[idx]  = get<0>(got->second);
            CG[a2 * num_of_agents + a1] = CG[idx];
            if (!counted[a1])
            {
                assert(get<1>(got->second) >= min_f_vals[a1]);
                delta_g += get<1>(got->second) - min_f_vals[a1];
                counted[a1] = true;
            }
            if (!counted[a2])
            {
                assert(get<2>(got->second) >= min_f_vals[a2]);
                delta_g += get<2>(got->second) - min_f_vals[a2];
                counted[a2] = true;
            }
        }
        else
        {
            auto rst = solve2Agents(a1, a2, node);
            lookupTable[a1][a2][HTableEntry(a1, a2, &node)] = rst;
            if ((clock() - start_time) / CLOCKS_PER_SEC > time_limit) // run out of time
            {
                runtime_build_dependency_graph += (double)(clock() - start_time) / CLOCKS_PER_SEC;
                return false;
            }
            CG[idx]  = get<0>(rst);
            CG[a2 * num_of_agents + a1] = CG[idx];
            if (!counted[a1])
            {
                assert(get<1>(rst) >= min_f_vals[a1]);
                delta_g += get<1>(rst) - min_f_vals[a1];
                counted[a1] = true;
            }
            if (!counted[a2])
            {
                assert(get<2>(rst) >= min_f_vals[a2]);
                delta_g += get<2>(rst) - min_f_vals[a2];
                counted[a2] = true;
            }
        }

        if (CG[idx] == MAX_COST) // no solution
            return false;
    }
	runtime_build_dependency_graph += (double)(clock() - start_time) / CLOCKS_PER_SEC;
	return true;
}

// return optimal f - root g and #HL nodes
pair<int, int> CBSHeuristic::solve2Agents(int a1, int a2, const CBSNode& node, bool cardinal)
{
	vector<SingleAgentSolver*> engines{search_engines[a1],   search_engines[a2]};
	vector<vector<PathEntry>> initial_paths{*paths[a1], *paths[a2]};
	vector<ConstraintTable> constraints{ConstraintTable(initial_constraints[a1]), ConstraintTable(initial_constraints[a2]) };
	constraints[0].build(node, a1);
	constraints[1].build(node, a2);
	CBS cbs(engines, constraints, initial_paths, screen);
	// setUpSubSolver(cbs);
	cbs.setPrioritizeConflicts(PC);
	cbs.setHeuristicType(heuristics_type::CG, heuristics_type::ZERO);
	cbs.setDisjointSplitting(disjoint_splitting);
	cbs.setBypass(false); // I guess that bypassing does not help two-agent path finding???
	cbs.setRectangleReasoning(rectangle_reasoning);
	cbs.setCorridorReasoning(corridor_reasoning);
	cbs.setTargetReasoning(target_reasoning);
	cbs.setMutexReasoning(mutex_reasoning);
	cbs.setConflictSelectionRule(conflict_seletion_rule);
	cbs.setNodeSelectionRule(node_selection_rule);
	cbs.setHighLevelSolver(high_level_solver_type::ASTAR, 1); // solve the sub problem optimally
	cbs.setNodeLimit(node_limit);

	double runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
	int root_g = (int)initial_paths[0].size() - 1 + (int)initial_paths[1].size() - 1;
	int lowerbound = root_g;
	int upperbound = MAX_COST;
	if (cardinal)
		lowerbound += 1;
	cbs.solve(time_limit - runtime, lowerbound, upperbound);
	num_solve_2agent_problems++;

	pair<int, int> rst;
	if (cbs.runtime >= time_limit - runtime || cbs.num_HL_expanded > node_limit) // time out or node out
		rst.first = cbs.getLowerBound() - root_g; // using lowerbound to approximate
	else if (cbs.solution_cost  < 0) // no solution
		rst.first = MAX_COST;
	else
	{
		assert(cbs.solution_cost >= root_g);
		rst.first = cbs.solution_cost - root_g;
	}
	rst.second = (int)cbs.num_HL_expanded;
	// For statistic study!!!
	if (save_stats)
	{
		sub_instances.emplace_back(a1, a2, &node, cbs.num_HL_expanded, rst.second);
	}
	return rst;
}

// return optimal f and a1_shortestpath * #agents + a2_shortestpath
tuple<int, int, int> CBSHeuristic::solve2Agents(int a1, int a2, const ECBSNode& node)
{
	vector<SingleAgentSolver*> engines{ search_engines[a1],   search_engines[a2] };
	vector<vector<PathEntry>> initial_paths;
	vector<ConstraintTable> constraints{ ConstraintTable(initial_constraints[a1]), ConstraintTable(initial_constraints[a2]) };
	constraints[0].build(node, a1);
	constraints[1].build(node, a2);
	CBS cbs(engines, constraints, initial_paths, screen);
	// setUpSubSolver(cbs);
	cbs.setPrioritizeConflicts(PC);
	cbs.setHeuristicType(heuristics_type::CG, heuristics_type::ZERO);
	cbs.setDisjointSplitting(disjoint_splitting);
	cbs.setBypass(false); // I guess that bypassing does not help two-agent path finding???
	cbs.setRectangleReasoning(rectangle_reasoning);
	cbs.setCorridorReasoning(corridor_reasoning);
	cbs.setTargetReasoning(target_reasoning);
	cbs.setMutexReasoning(mutex_reasoning);
	cbs.setConflictSelectionRule(conflict_seletion_rule);
	cbs.setNodeSelectionRule(node_selection_rule);
	cbs.setHighLevelSolver(high_level_solver_type::ASTAR, 1); // solve the sub problem optimally
	cbs.setNodeLimit(node_limit);

	double runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
	cbs.solve(time_limit - runtime, 0, MAX_COST);
	num_solve_2agent_problems++;
	
	// For statistic study!!!
	if (save_stats)
	{
		sub_instances.emplace_back(a1, a2, &node, cbs.num_HL_expanded, (int)cbs.num_HL_expanded);
	}

	if (cbs.runtime >= time_limit - runtime || cbs.num_HL_expanded > node_limit) // time out or node out
		return make_tuple(cbs.getLowerBound() - cbs.dummy_start->g_val,
		        cbs.getInitialPathLength(0), cbs.getInitialPathLength(1)); // using lowerbound to approximate
	else if (cbs.solution_cost  < 0) // no solution
		return make_tuple(MAX_COST, cbs.getInitialPathLength(0), cbs.getInitialPathLength(1));
	else
		return make_tuple(cbs.solution_cost - cbs.dummy_start->g_val,
		        cbs.getInitialPathLength(0), cbs.getInitialPathLength(1));
}

/*
int CBSHeuristic::getEdgeWeight(int a1, int a2, CBSNode& node, bool cardinal)
{
	HTableEntry newEntry(a1, a2, &node);
	if (type != heuristics_type::CG)
	{
		HTable::const_iterator got = lookupTable[a1][a2].find(newEntry);

		if (got != lookupTable[a1][a2].end())
		{
			num_memoization++;
			return got->second;
		}

	}

	int cost_shortestPath = (int)paths[a1]->size() + (int)paths[a2]->size() - 2;
	// runtime = (double)(clock() - start) / CLOCKS_PER_SEC;
	if (screen > 2)
	{
		cout << "Agents " << a1 << " and " << a2 << " in node " << node.time_generated << " : ";
	}
	int rst = 0;
	if (cardinal)
		rst = 1;
	else if (!mutex_reasoning && // no mutex reasoning, so we might miss some cardinal conflicts
		(type == heuristics_type::DG || type == heuristics_type::WDG))
	{
		// get mdds

		const MDD* mdd1 = mdd_helper.getMDD(node, a1, paths[a1]->size());
		const MDD* mdd2 = mdd_helper.getMDD(node, a2, paths[a2]->size());
		if (mdd1->levels.size() > mdd2->levels.size()) // swap
		{
			const MDD* temp = mdd1;
			mdd1 = mdd2;
			mdd2 = temp;
		}
		if (!SyncMDDs(*mdd1, *mdd2))
			rst = 1;
		else
			rst = 0;
		num_merge_MDDs++;
	}
	if (type == heuristics_type::WDG && rst > 0)
	{
		vector<SingleAgentSolver*> engines(2);
		engines[0] = search_engines[a1];
		engines[1] = search_engines[a2];
		vector<vector<PathEntry>> initial_paths(2);
		initial_paths[0] = *paths[a1];
		initial_paths[1] = *paths[a2];
		int upperbound = (int)initial_paths[0].size() + (int)initial_paths[1].size() + 10;
		vector<ConstraintTable> constraints{
			ConstraintTable(initial_constraints[a1]),
			ConstraintTable(initial_constraints[a2]) };
		constraints[0].build(node, a1);
		constraints[1].build(node, a2);
		CBS cbs(engines, constraints, initial_paths, upperbound, screen);
		cbs.setPrioritizeConflicts(PC);
		cbs.setHeuristicType(heuristics_type::CG);
		cbs.setDisjointSplitting(disjoint_splitting);
		cbs.setBypass(false); // I guess that bypassing does not help two-agent path finding???
		cbs.setRectangleReasoning(rectangle_reasoning);
		cbs.setCorridorReasoning(corridor_reasoning);
		cbs.setTargetReasoning(target_reasoning);
		cbs.setMutexReasoning(mutex_reasoning);
		cbs.setConflictSelectionRule(conflict_seletion_rule);
		cbs.setNodeSelectionRule(node_selection_fule);

		double runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
		cbs.solve(time_limit - runtime, max(rst, 0));
		if (cbs.runtime >= time_limit - runtime) // time out
			rst = (int)cbs.min_f_val - cost_shortestPath; // using lowerbound to approximate
		else if (cbs.solution_cost  < 0) // no solution
			rst = cbs.solution_cost;
		else
			rst = cbs.solution_cost - cost_shortestPath;
		num_solve_2agent_problems++;
	}
	lookupTable[a1][a2][newEntry] = rst;
	return rst;
}
*/

/*void CBSHeuristic::setUpSubSolver(CBS& cbs) const
{
	cbs.setPrioritizeConflicts(PC);
	cbs.setHeuristicType(heuristics_type::CG, heuristics_type::ZERO);
	cbs.setDisjointSplitting(disjoint_splitting);
	cbs.setBypass(false); // I guess that bypassing does not help two-agent path finding???
	cbs.setRectangleReasoning(rectangle_reasoning);
	cbs.setCorridorReasoning(corridor_reasoning);
	cbs.setTargetReasoning(target_reasoning);
	cbs.setMutexReasoning(mutex_reasoning);
	cbs.setConflictSelectionRule(conflict_seletion_rule);
	cbs.setNodeSelectionRule(node_selection_rule);
	cbs.setHighLevelSolver(high_level_solver_type::ASTAR, 1); // solve the sub problem optimally
	cbs.setNodeLimit(node_limit);
}*/

int CBSHeuristic::minimumVertexCover(const vector<int>& CG)
{
	clock_t t = clock();
	int rst = 0;
	std::vector<bool> done(num_of_agents, false);
	for (int i = 0; i < num_of_agents; i++)
	{
		if (done[i])
			continue;
		std::vector<int> indices;
		indices.reserve(num_of_agents);
		std::queue<int> Q;
		Q.push(i);
		done[i] = true;
		while (!Q.empty())
		{
			int j = Q.front(); Q.pop();
			indices.push_back(j);
			for (int k = 0; k < num_of_agents; k++)
			{
				if (CG[j * num_of_agents + k] > 0 || CG[k * num_of_agents + j] > 0)
				{
					if (!done[k])
					{
						Q.push(k);
						done[k] = true;
					}
				}
			}
		}
		if ((int) indices.size() == 1) //one node -> no edges -> mvc = 0
			continue;
		else if ((int)indices.size() == 2) // two nodes -> only one edge -> mvc = 1
		{
			rst += 1; // add edge weight
			continue;
		}

		std::vector<int> subgraph(indices.size()  * indices.size(), 0);
		int num_edges = 0;
		for (int j = 0; j < (int) indices.size(); j++)
		{
			for (int k = j + 1; k < (int)indices.size(); k++)
			{
				subgraph[j * indices.size() + k] = CG[indices[j] * num_of_agents + indices[k]];
				subgraph[k * indices.size() + j] = CG[indices[k] * num_of_agents + indices[j]];
				if (subgraph[j * indices.size() + k] > 0)
					num_edges++;
			}
		}

		if (num_edges > ILP_edge_threshold)
		{
			rst += greedyMatching(subgraph, (int)indices.size());
			double runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
			if (runtime > time_limit)
				return -1; // run out of time
		}
		else
		{
			for (int k = 1; i < (int)indices.size(); k++)
			{
				if (KVertexCover(subgraph, (int)indices.size(), num_edges, i=k, (int)indices.size()))
				{
					rst += k;
					break;
				}
				double runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
				if (runtime > time_limit)
					return -1; // run out of time
			}
		}
	}
	num_solve_MVC++;
	runtime_solve_MVC += (double)(clock() - t) / CLOCKS_PER_SEC;
	return rst;
}

int CBSHeuristic::minimumVertexCover(const std::vector<int>& CG, int old_mvc, int cols, int num_of_CGedges)
{
	clock_t t = clock();
	int rst = 0;
	if (num_of_CGedges < 2)
		return num_of_CGedges;
	// Compute #CG nodes that have edges
	int num_of_CGnodes = 0;
	for (int i = 0; i <  cols; i++)
	{
		for (int j = 0; j <  cols; j++)
		{
			if (CG[i * cols + j] > 0)
			{
				num_of_CGnodes++;
				break;
			}
		}
	}

	if (old_mvc == -1)
	{
		for (int i = 1; i < num_of_CGnodes; i++)
		{
			if (KVertexCover(CG, num_of_CGnodes, num_of_CGedges, i, cols))
			{
				rst = i;
				break;
			}
		}
        assert(rst>0);
	}
	else
	{
		if (KVertexCover(CG, num_of_CGnodes, num_of_CGedges, old_mvc - 1, cols))
			rst = old_mvc - 1;
		else if (KVertexCover(CG, num_of_CGnodes, num_of_CGedges, old_mvc, cols))
			rst = old_mvc;
		else
			rst = old_mvc + 1;
	}
	num_solve_MVC++;
	runtime_solve_MVC += (double)(clock() - t) / CLOCKS_PER_SEC;
	return rst;
}

// Whether there exists a k-vertex cover solution
bool CBSHeuristic::KVertexCover(const std::vector<int>& CG, int num_of_CGnodes, int num_of_CGedges, int k, int cols)
{
	double runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
	if (runtime > time_limit)
		return true; // run out of time
	if (num_of_CGedges == 0)
		return true;
	else if (num_of_CGedges > k * num_of_CGnodes - k)
		return false;

	std::vector<int> node(2);
	bool flag = true;
	for (int i = 0; i < cols - 1 && flag; i++) // to find an edge
	{
		for (int j = i + 1; j < cols && flag; j++)
		{
			if (CG[i * cols + j] > 0)
			{
				node[0] = i;
				node[1] = j;
				flag = false;
			}
		}
	}
	for (int i = 0; i < 2; i++)
	{
		std::vector<int> CG_copy(CG.size());
		CG_copy.assign(CG.cbegin(), CG.cend());
		int num_of_CGedges_copy = num_of_CGedges;
		for (int j = 0; j < cols; j++)
		{
			if (CG_copy[node[i] * cols + j] > 0)
			{
				CG_copy[node[i] * cols + j] = 0;
				CG_copy[j * cols + node[i]] = 0;
				num_of_CGedges_copy--;
			}
		}
		if (KVertexCover(CG_copy, num_of_CGnodes - 1, num_of_CGedges_copy, k - 1, cols))
			return true;
	}
	return false;
}


int CBSHeuristic::greedyMatching(const std::vector<int>& CG,  int cols)
{
    int rst = 0;
    std::vector<bool> used(cols, false);
    for (int i = 0; i < cols; i++)
    {
        if(used[i])
            continue;
        for (int j = i + 1; j < cols; j++)
        {
            if (used[j])
                continue;
            if (CG[i * cols + j] > 0)
            {
                rst+=1;
                used[i] = true;
                used[j] = true;
                break;
            }
        }
    }
    return rst;
}

int CBSHeuristic::greedyWeightedMatching(const std::vector<int>& CG,  int cols)
{
    int rst = 0;
    std::vector<bool> used(cols, false);
    while(true)
    {
        int maxWeight = 0;
        int ep1, ep2;
        for (int i = 0; i < cols; i++)
        {
            if(used[i])
                continue;
            for (int j = i + 1; j < cols; j++)
            {
                if (used[j])
                    continue;
                else if (maxWeight < CG[i * cols + j])
                {
                    maxWeight = CG[i * cols + j];
                    ep1 = i;
                    ep2 = j;
                }
            }
        }
        if (maxWeight == 0)
            return rst;
        rst += maxWeight;
        used[ep1] = true;
        used[ep2] = true;
    }
}

int CBSHeuristic::minimumWeightedVertexCover(const vector<int>& HG)
{
	clock_t t = clock();
	int rst = weightedVertexCover(HG);
	num_solve_MVC++;
	runtime_solve_MVC += (double)(clock() - t) / CLOCKS_PER_SEC;
	return rst;
}

// branch and bound
// enumerate all possible assignments and return the best one
int CBSHeuristic::weightedVertexCover(const std::vector<int>& CG)
{
	int rst = 0;
	std::vector<bool> done(num_of_agents, false);
	for (int i = 0; i < num_of_agents; i++)
	{
		if (done[i])
			continue;
		std::vector<int> range;
		std::vector<int> indices;
		range.reserve(num_of_agents);
		indices.reserve(num_of_agents);
		int num = 0;
		std::queue<int> Q;
		Q.push(i);
		done[i] = true;
		while (!Q.empty())
		{
			int j = Q.front(); Q.pop();
			range.push_back(0);
			indices.push_back(j);
			for (int k = 0; k < num_of_agents; k++)
			{
				if (CG[j * num_of_agents + k] > 0)
				{
					range[num] = std::max(range[num], CG[j * num_of_agents + k]);
					if (!done[k])
					{
						Q.push(k);
						done[k] = true;
					}
				}		
				else if (CG[k * num_of_agents + j] > 0)
				{
					range[num] = std::max(range[num], CG[k * num_of_agents + j]);
					if (!done[k])
					{
						Q.push(k);
						done[k] = true;
					}
				}
			}
			num++;
		}
		if (num  == 1) // no edges
			continue;
		else if (num == 2) // only one edge
		{
			rst += std::max(CG[indices[0] * num_of_agents + indices[1]], CG[indices[1] * num_of_agents + indices[0]]); // add edge weight
			continue;
		}
		std::vector<int> G(num * num, 0);
		for (int j = 0; j < num; j++)
		{
			for (int k = j + 1; k < num; k++)
			{
				G[j * num + k] = std::max(CG[indices[j] * num_of_agents + indices[k]], CG[indices[k] * num_of_agents + indices[j]]);
			}
		}
		if (num > ILP_node_threshold)
		{
		    rst += greedyWeightedMatching(G, num);
			// rst += ILPForWMVC(G, range);
		}
		else
		{
			std::vector<int> x(num);
			int best_so_far = MAX_COST;
			rst += DPForWMVC(x, 0, 0, G, range, best_so_far);
		}
		double runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
		if (runtime > time_limit)
			return -1; // run out of time
	}

	//test
	/*std::vector<int> x(N, 0);
	std::vector<int> range(N, 0);
	for (int i = 0; i < N; i++)
	{
		for (int j = i + 1; j < N; j++)
		{
			range[i] = std::max(range[i], CG[i * N + j]);
			range[j] = std::max(range[j], CG[i * N + j]);
		}
	}
	int best_so_far = INT_MAX;
	int rst2 = weightedVertexCover(x, 0, 0, CG, range, best_so_far);
	if( rst != rst2)
		std::cout << "ERROR" <<std::endl;*/

	return rst;
}

// recusive component of weighted vertex cover
int CBSHeuristic::DPForWMVC(std::vector<int>& x, int i, int sum, const std::vector<int>& CG,
	const std::vector<int>& range, int& best_so_far)
{
	if (sum >= best_so_far)
		return MAX_COST;
	double runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
	if (runtime > time_limit)
		return -1; // run out of time
	else if (i == (int)x.size())
	{
		best_so_far = sum;
		return sum;
	}
	else if (range[i] == 0) // vertex i does not have any edges.
	{
		int rst = DPForWMVC(x, i + 1, sum, CG, range, best_so_far);
		if (rst < best_so_far)
		{
			best_so_far = rst;
		}
		return best_so_far;
	}
	
	int cols = x.size();
	
	// find minimum cost for this vertex
	int min_cost = 0;
	for (int j = 0; j < i; j++)
	{
		if (min_cost + x[j] < CG[j * cols + i]) // infeasible assignment
		{
			min_cost = CG[j * cols + i] - x[j]; // cost should be at least CG[i][j] - x[j];
		}
	}


	int best_cost = -1;
	for (int cost = min_cost; cost <= range[i]; cost++)
	{
		x[i] = cost;
		int rst = DPForWMVC(x, i + 1, sum + x[i], CG, range, best_so_far);
		if (rst < best_so_far)
		{
			best_so_far = rst;
			best_cost = cost;
		}
	}
	if (best_cost >= 0)
	{
		x[i] = best_cost; 
	}

	return best_so_far;
}

/*int CBSHeuristic::ILPForWMVC(const vector<int>& CG, const vector<int>& node_max_value) const
{
		int N = (int)node_max_value.size();
		IloEnv env = IloEnv();
		IloModel model = IloModel(env);
		IloExpr sum_obj = IloExpr(env);
		IloNumVarArray var(env);
		IloRangeArray con(env);
		for (int i = 0; i < N; i++)
		{
			var.add(IloNumVar(env, 0, node_max_value[i] + 1, ILOINT));
			sum_obj += var[i];
		}
		model.add(IloMinimize(env, sum_obj));
		for (int i = 0; i < N; i++)
		{
			for (int j = i + 1; j < N; j++)
			{
				if (CG[i * N + j] > 0)
				{
					con.add(var[i] + var[j] >= CG[i * N + j]);
				}
			}
		}
		model.add(con);
		IloCplex cplex(env);
		double runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
		if (time_limit - runtime <= 0)
			return 0;
		cplex.setParam(IloCplex::TiLim, time_limit - runtime);
		cplex.extract(model);
		cplex.setOut(env.getNullStream());
		int rst=0;
		if (cplex.solve())
			rst = (int)cplex.getObjValue();
		env.end();
		return rst;
}*/

/*int CBSHeuristic::ILPForConstrainedWMVC(const std::vector<int>& CG, const std::vector<int>& node_weights)
{
	int N = (int)node_weights.size();
	IloEnv env = IloEnv();
	IloModel model = IloModel(env);
	IloExpr sum_obj = IloExpr(env);
	IloNumVarArray var(env);
	IloRangeArray con(env);
	for (size_t i = 0; i < N; i++)
	{
		var.add(IloNumVar(env, 0, 1, ILOBOOL));
		sum_obj += var[i] * node_weights[i];
	}
	model.add(IloMinimize(env, sum_obj));
	for (int i = 0; i < N; i++)
	{
		for (int j = i + 1; j < N; j++)
		{
			if (CG[i * N + j] > 0)
			{
				con.add(var[i] + var[j] >= 1);
			}
		}
	}
	model.add(con);
	IloCplex cplex(env);
	double runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
	cplex.setParam(IloCplex::TiLim, time_limit - runtime); // time limit = 300 sec
	int solution_cost = -1;
	cplex.extract(model);
	cplex.setOut(env.getNullStream());
	int rst = 0;
	if (cplex.solve())
		rst = (int)cplex.getObjValue();
	else
	{
		std::cout << "ERROR" << endl;
		cplex.exportModel("error.lp");
	}
	env.end();
	return rst;
}*/

// recusive component of weighted vertex cover
int CBSHeuristic::DPForConstrainedWMVC(std::vector<bool>& x, int i, int sum, const std::vector<int>& CG, const std::vector<int>& range, int& best_so_far)
{
	if (sum >= best_so_far)
		return INT_MAX;
	double runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
	if (runtime > time_limit)
		return -1; // run out of time
	else if (i == (int)x.size())
	{
		best_so_far = sum;
		return sum;
	}
	else if (range[i] == 0) // vertex i does not have any edges.
	{
		int rst = DPForConstrainedWMVC(x, i + 1, sum, CG, range, best_so_far);
		if (rst < best_so_far)
		{
			best_so_far = rst;
		}
		return best_so_far;
	}

	int cols = x.size();

	// find minimum cost for this vertex
	int min_cost = 0;
	for (int j = 0; j < i; j++)
	{
		if (min_cost + x[j] * range[j] < CG[j * cols + i]) // infeasible assignment
		{
			min_cost = CG[j * cols + i] - x[j] * range[j]; // cost should be at least CG[i][j] - x[j];
		}
	}
	if (min_cost == 0)
	{
		x[i] = 0;
		int rst = DPForConstrainedWMVC(x, i + 1, sum, CG, range, best_so_far);
		if (rst < best_so_far)
		{
			best_so_far = rst;
		}
	}
	if (min_cost < range[i])
	{
		x[i] = 1;
		int rst = DPForConstrainedWMVC(x, i + 1, sum + x[i] * range[i], CG, range, best_so_far);
		if (rst < best_so_far)
		{
			best_so_far = rst;
		}
	}
	return best_so_far;
}



/*void CBSHeuristic::copyConflictGraph(HLNode& child, const HLNode& parent)
{
	//copy conflict graph
	if (type == heuristics_type::DG || type == heuristics_type::WDG)
	{
		unordered_set<int> changed;
		for (auto a : child.getReplannedAgents())
			changed.insert(a);
		for (auto e : parent.conflictGraph)
		{
			if (changed.find(e.first / num_of_agents) == changed.end() &&
				changed.find(e.first % num_of_agents) == changed.end())
				child.conflictGraph[e.first] = e.second;
		}

	}
}*/

bool CBSHeuristic::dependent(int a1, int a2, HLNode& node) // return true if the two agents are dependent
{
	const MDD* mdd1 = mdd_helper.getMDD(node, a1, paths[a1]->size()); // get mdds
	const MDD* mdd2 = mdd_helper.getMDD(node, a2, paths[a2]->size());
	if (mdd1->levels.size() > mdd2->levels.size()) // swap
		std::swap(mdd1, mdd2);
	num_merge_MDDs++;
	return !SyncMDDs(*mdd1, *mdd2);
}

// return true if the joint MDD exists.
bool CBSHeuristic::SyncMDDs(const MDD &mdd, const MDD& other) // assume mdd.levels <= other.levels
{
	if (other.levels.size() <= 1) // Either of the MDDs was already completely pruned already
		return false;

	SyncMDD copy(mdd);
	if (copy.levels.size() < other.levels.size())
	{
		size_t i = copy.levels.size();
		copy.levels.resize(other.levels.size());
		for (; i < copy.levels.size(); i++)
		{
			SyncMDDNode* parent = copy.levels[i - 1].front();
			auto node = new SyncMDDNode(parent->location, parent);
			parent->children.push_back(node);
			copy.levels[i].push_back(node);

		}
	}
	// Cheaply find the coexisting nodes on level zero - all nodes coexist because agent starting points never collide
	copy.levels[0].front()->coexistingNodesFromOtherMdds.push_back(other.levels[0].front());

	// what if level.size() = 1?
	for (size_t i = 1; i < copy.levels.size(); i++)
	{
		for (auto node = copy.levels[i].begin(); node != copy.levels[i].end();)
		{
			// Go over all the node's parents and test their coexisting nodes' children for co-existance with this node
			for (auto parent = (*node)->parents.begin(); parent != (*node)->parents.end(); parent++)
			{
				//bool validParent = false;
				for (const MDDNode* parentCoexistingNode : (*parent)->coexistingNodesFromOtherMdds)
				{
					for (const MDDNode* childOfParentCoexistingNode : parentCoexistingNode->children)
					{
						if ((*node)->location == childOfParentCoexistingNode->location ||// vertex conflict
							((*node)->location == parentCoexistingNode->location && (*parent)->location == childOfParentCoexistingNode->location)) // edge conflict
							continue;

						auto it = (*node)->coexistingNodesFromOtherMdds.cbegin();
						for (; it != (*node)->coexistingNodesFromOtherMdds.cend(); ++it)
						{
							if (*it == childOfParentCoexistingNode)
								break;
						}
						if (it == (*node)->coexistingNodesFromOtherMdds.cend())
						{
							(*node)->coexistingNodesFromOtherMdds.push_back(childOfParentCoexistingNode);
						}
					}
				}
			}
			if ((*node)->coexistingNodesFromOtherMdds.empty())
			{
				// delete the node, and continue up the levels if necessary
				SyncMDDNode* p = *node;
				node++;
				copy.deleteNode(p, i);
			}
			else
				node++;
		}
		if (copy.levels[i].empty())
		{
			copy.clear();
			return false;
		}
	}
	copy.clear();
	return true;
}
