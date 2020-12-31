#include "RectangleReasoning.h"


shared_ptr<Conflict> RectangleReasoning::run(const vector<Path*>& paths, int timestep,
	int a1, int a2, const MDD* mdd1, const MDD* mdd2)
{
	clock_t t = clock();
	auto rectangle = findRectangleConflictByRM(paths, timestep, a1, a2, mdd1, mdd2);
	accumulated_runtime += (double)(clock() - t) / CLOCKS_PER_SEC;
	return rectangle;
}


shared_ptr<Conflict> RectangleReasoning::findRectangleConflictByGR(const vector<Path*>& paths, int timestep,
	int a1, int a2, const MDD* mdd1, const MDD* mdd2)
{
	assert(timestep > 0);
	int from1 = (*paths[a1])[timestep - 1].location;
	int from2 = (*paths[a2])[timestep - 1].location;
	int loc = paths[a1]->at(timestep).location;
	if (from1 == from2 || // same direction
		from1 == loc || from2 == loc || //wait actions
		abs(from1 - from2) == 2 || abs(from1 - from2) == instance.getCols() * 2) //opposite direction
		return nullptr;

	list<Constraint> B1;
	int t_start = getStartCandidate(*paths[a1], loc - from1, loc - from2, timestep);
	int t_end = getGoalCandidate(*paths[a1], loc - from1, loc - from2, timestep);
	bool haveBarriers = ExtractBarriers(*mdd1, loc, timestep, loc - from1, loc - from2, 
		paths[a1]->at(t_start).location, paths[a1]->at(t_end).location, t_start, B1);
	if (!haveBarriers)
		return nullptr;

	list<Constraint> B2;
	t_start = getStartCandidate(*paths[a2], loc - from1, loc - from2, timestep);
	t_end = getGoalCandidate(*paths[a2], loc - from1, loc - from2, timestep);

	haveBarriers = ExtractBarriers(*mdd2, loc, timestep, loc - from2, loc - from1,
		paths[a2]->at(t_start).location, paths[a2]->at(t_end).location, t_start, B2);
	if (!haveBarriers)
		return nullptr;

	// Try all possible combinations
	int type = -1;
	pair<int, int> Rs, Rg;
	generalizedRectangle(*paths[a1], *paths[a2], *mdd1, *mdd2, B1, B2, timestep, type, Rs, Rg);

	if (type < 0)
		return nullptr;
	int Rg_t = timestep + instance.getManhattanDistance(instance.getCoordinate(loc), Rg);
	
	list<Constraint> constraint1;
	list<Constraint> constraint2;
	bool succ;
	if (abs(loc - from1) == 1 || abs(loc - from2) > 1) // first agent moves horizontally and second agent moves vertically
	{
		succ = addModifiedVerticalBarrierConstraint(a1, mdd1, Rg.second, Rs.first, Rg.first, Rg_t, constraint1);
		assert(succ);
		succ = addModifiedHorizontalBarrierConstraint(a2, mdd2, Rg.first, Rs.second, Rg.second, Rg_t, constraint2);
		assert(succ);
	}
	else
	{
		succ = addModifiedHorizontalBarrierConstraint(a1, mdd1, Rg.first, Rs.second, Rg.second, Rg_t, constraint1);
		assert(succ);
		succ = addModifiedVerticalBarrierConstraint(a2, mdd2, Rg.second, Rs.first, Rg.first, Rg_t, constraint2);
		assert(succ);
	}

	if (!blocked(*paths[a1], constraint1) || !blocked(*paths[a2], constraint2))
		return nullptr;
	auto rectangle = shared_ptr<Conflict>(new Conflict());
	rectangle->rectangleConflict(a1, a2, Rs, Rg, Rg_t, constraint1, constraint2);
	if (type == 2)
		rectangle->priority = conflict_priority::CARDINAL;
	else if (type == 1) // && !findRectangleConflict(parent.parent, *conflict))
		rectangle->priority = conflict_priority::SEMI;
	else //if (type == 0 && !findRectangleConflict(parent.parent, *conflict))
		rectangle->priority = conflict_priority::NON;
	return rectangle;
}


shared_ptr<Conflict> RectangleReasoning::findRectangleConflictByRM(const vector<Path*>& paths, int timestep,
        int a1, int a2, const MDD* mdd1, const MDD* mdd2)
{
    shared_ptr<Conflict> rectangle = nullptr;
    //Rectangle reasoning for semi and non cardinal vertex conflicts
    list<int>	s1s = getStartCandidates(*paths[a1], *mdd1,  timestep);
    list<int>	g1s = getGoalCandidates(*paths[a1], *mdd1, timestep);
    list<int>	s2s = getStartCandidates(*paths[a2], *mdd2, timestep);
    list<int>	g2s = getGoalCandidates(*paths[a2], *mdd2, timestep);
	pair<int, int> location = instance.getCoordinate(paths[a1]->at(timestep).location);

    // Try all possible combinations
    int type = -1;
    int area = 0;
    for (int t1_start : s1s)
    {
        for (int t1_end : g1s)
        {
			auto s1 = instance.getCoordinate(paths[a1]->at(t1_start).location);
			auto g1 = instance.getCoordinate(paths[a1]->at(t1_end).location);
            if (instance.getManhattanDistance(s1, g1) !=  t1_end - t1_start)
                continue;
            for (int t2_start : s2s)
            {
                for (int t2_end : g2s)
                {
                    auto s2 = instance.getCoordinate(paths[a2]->at(t2_start).location);
                    auto g2 = instance.getCoordinate(paths[a2]->at(t2_end).location);
                    if (instance.getManhattanDistance(s2, g2) != t2_end - t2_start)
                        continue;
                    if (!isRectangleConflict(s1, s2, g1, g2))
                        continue;
                    auto Rg = getRg(s1, g1, g2);
                    auto Rs = getRs(s1, s2, g1);
                    int new_area = (abs(Rs.first - Rg.first) + 1) * (abs(Rs.second - Rg.second) + 1);
                    int new_type = classifyRectangleConflict(s1, s2, g1, g2, Rg);
                    if (new_type > type || (new_type == type && new_area > area))
                    {
                        int Rg_t = timestep + abs(Rg.first - location.first) + abs(Rg.second - location.second);
						list<Constraint> constraint1;
						list<Constraint> constraint2;
						bool succ = addModifiedBarrierConstraints(a1, a2, Rs, Rg, s1, s2, 
							Rg_t, mdd1, mdd2, constraint1, constraint2);
                        if (succ && blocked(*paths[a1], constraint1) && blocked(*paths[a2], constraint2))
                        {
							type = new_type;
							area = new_area;
							rectangle = shared_ptr<Conflict>(new Conflict());
							rectangle->rectangleConflict(a1, a2, Rs, Rg, Rg_t, constraint1, constraint2);
                            if (type == 2)
							{
								rectangle->priority = conflict_priority::CARDINAL;
								return rectangle;
							}
                            else if (type == 1) // && !findRectangleConflict(parent.parent, *conflict))
								rectangle->priority = conflict_priority::SEMI;
                            else //if (type == 0 && !findRectangleConflict(parent.parent, *conflict))
								rectangle->priority = conflict_priority::NON;
                        }
                    }
                }
            }
        }
    }

    return rectangle;
}



bool RectangleReasoning::ExtractBarriers(const MDD& mdd, int loc, int timestep,
					int dir1, int dir2, int start, int goal, int start_time, list<Constraint>& B)
{
	int num_barrier;
	int sign1 = dir1 / abs(dir1);
	int sign2 = dir2 / abs(dir2);
	if (abs(dir1) == 1) //vertical barriers
	{
		num_barrier = sign1 * (instance.getColCoordinate(goal) - instance.getColCoordinate(start)) + 1;
	}
	else
	{
		num_barrier = sign1 * (instance.getRowCoordinate(goal) - instance.getRowCoordinate(start)) + 1;
	}

	vector<int> extent_L(num_barrier, MAX_TIMESTEP);
	vector<int> extent_U(num_barrier, -1);

	unordered_map<MDDNode*, vector<bool>> blocking;

	auto n = mdd.levels[0].front();
	vector<bool> block(num_barrier, false);
	//int hasStart = false;
	int barrier_time;
	if (abs(dir1) == 1) //vertical barriers
	{
		barrier_time = timestep + sign1 * (instance.getColCoordinate(n->location) - instance.getColCoordinate(loc))
			+ sign2 * (instance.getRowCoordinate(n->location) - instance.getRowCoordinate(loc));
	}
	else
	{
		barrier_time = timestep + sign1 * (instance.getRowCoordinate(n->location) - instance.getRowCoordinate(loc))
			+ sign2 * (instance.getColCoordinate(n->location) - instance.getColCoordinate(loc));
	}
	if (barrier_time == 0)
	{
		extent_L[0] = 0;
		extent_U[0] = 0;
		block[0] = true;
		//hasStart = true;
	}
	blocking[n] = block;

	for (size_t t = 1; t < mdd.levels.size(); t++)
	{
		for (auto n : mdd.levels[t])
		{
			vector<bool> block(num_barrier, true);
			for (auto parent : n->parents)
			{
				vector<bool> parent_block = blocking[parent];
				for (int i = 0; i < num_barrier; i++)
				{
					if (!parent_block[i])
						block[i] = false;
				}
			}
			int barrier_id, barrier_time;
			if (abs(dir1) == 1) //vertical barriers
			{
				barrier_id = sign1 * (instance.getColCoordinate(n->location) - instance.getColCoordinate(start));
				barrier_time = timestep + sign1 * (instance.getColCoordinate(n->location) - instance.getColCoordinate(loc)) 
															+ sign2 * (instance.getRowCoordinate(n->location) - instance.getRowCoordinate(loc));
			}
			else
			{
				barrier_id = sign1 * (instance.getRowCoordinate(n->location) - instance.getRowCoordinate(start));
				barrier_time = timestep + sign1 * (instance.getRowCoordinate(n->location) - instance.getRowCoordinate(loc))
															+ sign2 * (instance.getColCoordinate(n->location) - instance.getColCoordinate(loc));
			}
			if (0 <= barrier_id && barrier_id < num_barrier && !block[barrier_id] && barrier_time == n->level)
			{
				if (n->children.size() == 1 && extent_L[barrier_id] == MAX_TIMESTEP &&
					abs(dir1) * abs(n->location - n->children.front()->location) == instance.getCols());// the only child node is on the same barrier
				else
				{
					extent_L[barrier_id] = min(extent_L[barrier_id], n->level);
					extent_U[barrier_id] = max(extent_U[barrier_id], n->level);
					block[barrier_id] = true;
				}
			}
			blocking[n] = block;
		}
	}

	n = mdd.levels.back().front();
	block = blocking[n];
	for (int i = 0; i < num_barrier; i++)
	{
		if (block[i])
		{
			int barrier_start_x, barrier_start_y, barrier_end_x, barrier_end_y, barrier_end_time;
			if (abs(dir1) == 1) //vertical barriers
			{
				barrier_start_y = instance.getColCoordinate(start) + sign1 * i;
				barrier_end_y = barrier_start_y;
				int time_offset = timestep + i - sign1 * (instance.getColCoordinate(loc) - instance.getColCoordinate(start));
				barrier_start_x = instance.getRowCoordinate(loc) + sign2 * (extent_L[i] - time_offset);
				barrier_end_x = instance.getRowCoordinate(loc) + sign2 * (extent_U[i] - time_offset);				
			}
			else
			{
				barrier_start_x = instance.getRowCoordinate(start) + sign1 * i;
				barrier_end_x = barrier_start_x;
				int time_offset = timestep + i - sign1 * (instance.getRowCoordinate(loc) - instance.getRowCoordinate(start));
				barrier_start_y = instance.getColCoordinate(loc) + sign2 * (extent_L[i] - time_offset);
				barrier_end_y = instance.getColCoordinate(loc) + sign2 * (extent_U[i] - time_offset);
			}
			barrier_end_time = extent_U[i];
			B.emplace_back(-1,  // for now, the agent index is not important,  so we just use -1 for simplexity.
				instance.linearizeCoordinate(barrier_start_x, barrier_start_y),
				instance.linearizeCoordinate(barrier_end_x, barrier_end_y), 
				barrier_end_time, constraint_type::BARRIER);
		}
	}
	return !B.empty();
}

bool RectangleReasoning::isEntryBarrier(const Constraint& b1, const Constraint& b2, int dir1)
{
	pair<int, int> b1_l = instance.getCoordinate(get<1>(b1));
	pair<int, int> b1_u = instance.getCoordinate(get<2>(b1));
	pair<int, int> b2_l = instance.getCoordinate(get<1>(b2));
	pair<int, int> b2_u = instance.getCoordinate(get<2>(b2));

	if (dir1 == instance.getCols() && b1_u.first >= b2_l.first && b2_l.first >= b1_l.first)
		return true;
	else if (dir1 == -instance.getCols() && b1_u.first <= b2_l.first && b2_l.first <= b1_l.first)
		return true;
	else if (dir1 == 1 && b1_u.second >= b2_l.second && b2_l.second >= b1_l.second)
		return true;
	else if (dir1 == -1 && b1_u.second <= b2_l.second && b2_l.second <= b1_l.second)
		return true;
	else
		return false;
}

bool RectangleReasoning::isExitBarrier(const Constraint& b1, const Constraint& b2, int dir1)
{

	pair<int, int> b1_l = instance.getCoordinate(get<1>(b1));
	pair<int, int> b1_u = instance.getCoordinate(get<2>(b1));
	pair<int, int> b2_l = instance.getCoordinate(get<1>(b2));
	pair<int, int> b2_u = instance.getCoordinate(get<2>(b2));

	if (dir1 == instance.getCols() && b2_u.first <= b1_l.first)
		return true;
	else if (dir1 == -instance.getCols() && b2_u.first >= b1_l.first)
		return true;
	else if (dir1 == 1 && b2_u.second <= b1_l.second)
		return true;
	else if (dir1 == -1 && b2_u.second >= b1_l.second)
		return true;
	else
		return false;
}

pair<int, int> RectangleReasoning::getIntersection(const Constraint& b1, const Constraint& b2)
{
	pair<int, int> b1_l = instance.getCoordinate(get<1>(b1));
	pair<int, int> b1_u = instance.getCoordinate(get<2>(b1));
	pair<int, int> b2_l = instance.getCoordinate(get<1>(b2));
	pair<int, int> b2_u = instance.getCoordinate(get<2>(b2));

	if (b1_l.first == b1_u.first && b2_l.second == b2_u.second)
		return make_pair(b1_l.first, b2_l.second);
	else
		return make_pair(b2_l.first, b1_l.second);
}

bool RectangleReasoning::blockedNodes(const vector<PathEntry>& path, 
	const pair<int, int>& Rs, const pair<int, int>& Rg, int Rg_t, int dir)
{
	pair<int, int> b_l;
	if (abs(dir) == 1)
	{
		b_l.first = Rg.first;
		b_l.second = Rs.second;
	}
	else
	{
		b_l.first = Rs.first;
		b_l.second = Rg.second;
	}

	int t_max = min(Rg_t, (int)path.size() - 1);
	int t_b_l = Rg_t - abs(b_l.first - Rg.first) - abs(b_l.second - Rg.second);
	int t_min = max(0, t_b_l);

	for (int t = t_min; t <= t_max; t++)
	{
		int loc = instance.linearizeCoordinate(b_l.first, b_l.second) + (t - t_b_l) * dir;
		if (path[t].location == loc)
		{
			return true;
		}
	}
	return false;
}

bool RectangleReasoning::isCut(const Constraint b, const pair<int, int>& Rs, const pair<int, int>& Rg)
{
	pair<int, int> b_l = instance.getCoordinate(get<1>(b));
	pair<int, int> b_u = instance.getCoordinate(get<2>(b));
	if (b_l == b_u)
	{
		if (((Rs.first <= b_l.first && b_u.first <= Rg.first) || (Rs.first >= b_l.first && b_u.first >= Rg.first)) &&
			((Rs.second <= b_l.second && b_u.second <= Rg.second) || (Rs.second >= b_l.second && b_u.second >= Rg.second)))
			return true;
		else
			return false;
	}
	if (Rs.first <= b_l.first && b_l.first <= b_u.first && b_u.first <= Rg.first && b_l.second == b_u.second)
		return true;
	else if (Rs.second <= b_l.second && b_l.second <= b_u.second && b_u.second <= Rg.second && b_l.first == b_u.first)
		return true;
	else if (Rs.first >= b_l.first && b_l.first >= b_u.first && b_u.first >= Rg.first && b_l.second == b_u.second)
		return true;
	else if (Rs.second >= b_l.second && b_l.second >= b_u.second && b_u.second >= Rg.second && b_l.first == b_u.first)
		return true;
	else
		return false;
}


void RectangleReasoning::generalizedRectangle(const vector<PathEntry>& path1, const vector<PathEntry>& path2, 
	const MDD& mdd1, const MDD& mdd2,
	const list<Constraint>& B1, const list<Constraint>& B2, int timestep,
	int& best_type, pair<int, int>& best_Rs, pair<int, int>& best_Rg)
{
	int loc = path1[timestep].location;
	int dir1 = loc - path1[timestep - 1].location;
	int dir2 = loc - path2[timestep - 1].location;
	for (const auto& b1_entry : B1)
	{
		for (const auto& b2_entry : B2)
		{
			if (isEntryBarrier(b1_entry, b2_entry, dir1) && isEntryBarrier(b2_entry, b1_entry, dir2))
			{
				pair<int, int> Rs = getIntersection(b1_entry, b2_entry);
				list<Constraint>::const_reverse_iterator  b1_exit = B1.rbegin();
				list<Constraint>::const_reverse_iterator  b2_exit = B2.rbegin();
				while (b1_exit != B1.rend() && b2_exit != B2.rend())
				{
					if (!isExitBarrier(*b1_exit, b2_entry, dir1))
					{
						break;
					}
					if (!isExitBarrier(*b2_exit, b1_entry, dir2))
					{
						break;
					}
					pair<int, int> Rg = getIntersection(*b1_exit, *b2_exit);
					int Rg_t = timestep + instance.getManhattanDistance(Rg, instance.getCoordinate(loc));
					if (!blockedNodes(path1, Rs, Rg, Rg_t, dir2))
					{
						++b1_exit;
						continue;
					}
					if (!blockedNodes(path2, Rs, Rg, Rg_t, dir1))
					{
						++b2_exit;
						continue;
					}

					bool cut1 = isCut(*b1_exit, Rs, Rg);
					bool cut2 = isCut(*b2_exit, Rs, Rg);
					int type = (int)(cut1)+(int)(cut2);
					if (type > best_type)
					{
						best_Rs = Rs;
						best_Rg = Rg;
						best_type = type;
						if (best_type == 2)
							return;
					}
					if (!cut1)
						++b1_exit;
					else if (!cut2)
						++b2_exit;
				}
			}
		}
	}
	return;
}


//Identify rectangle conflicts for CR/R
bool RectangleReasoning::isRectangleConflict(const pair<int, int>& s1, const pair<int, int>& s2,
	const pair<int, int>& g1, const pair<int, int>& g2, int g1_t, int g2_t)
{
	return g1_t == abs(s1.first - g1.first) + abs(s1.second - g1.second) &&  // Manhattan-optimal
		        g2_t == abs(s2.first - g2.first) + abs(s2.second - g2.second) && // Manhattan-optimal
			(s1.first - g1.first) * (s2.first - g2.first) >= 0 &&  //Move in the same direction
			(s1.second - g1.second) * (s2.second - g2.second) >= 0; //Move in the same direction
}

//Identify rectangle conflicts for RM
bool RectangleReasoning::isRectangleConflict(const pair<int, int>& s1, const pair<int, int>& s2, const pair<int, int>& g1, const pair<int, int>& g2) const
{
	if (s1 == s2) // A standard cardinal conflict
		return false;
	else if (s1 == g1 || s2 == g2) // s1 = g1 or  s2 = g2
		return false;

	if ((s1.first - g1.first) * (s2.first - g2.first) < 0 || (s1.second - g1.second) * (s2.second - g2.second) < 0) // Not move in the same direction
		return false;
	/*else if ((S2.first - S1.first) * (S1.first - G1.first) > 0 && (S2.second - S1.second) * (S1.second - G1.second) > 0) // s1 always in the middle
		return false;
	else if ((S1.first - S2.first) * (S2.first - G2.first) > 0 && (S1.second - S2.second) * (S2.second - G2.second) > 0) // s2 always in the middle
		return false;*/
	else return !((s1.first == g1.first && s2.second == g2.second) || (s1.second == g1.second && s2.first == g2.first)); // not a cardinal vertex conflict
}

//Classify rectangle conflicts for CR/R
// Return 2 if it is a cardinal rectangle conflict
// Return 1 if it is a semi-cardinal rectangle conflict
// Return 0 if it is a non-cardinal rectangle conflict
int RectangleReasoning::classifyRectangleConflict(const pair<int, int>& s1, const pair<int, int>& s2,
	const pair<int, int>& g1, const pair<int, int>& g2)
{
		int cardinal1 = 0, cardinal2 = 0;
		if ((s1.first - s2.first) * (g1.first - g2.first) <= 0)
			cardinal1++;
		if ((s1.second - s2.second) * (g1.second - g2.second) <= 0)
			cardinal2++;
		return cardinal1 + cardinal2;
}

//Classify rectangle conflicts for RM
// Return 2 if it is a cardinal rectangle conflict
// Return 1 if it is a semi-cardinal rectangle conflict
// Return 0 if it is a non-cardinal rectangle conflict
int RectangleReasoning::classifyRectangleConflict(const pair<int, int>& s1, const pair<int, int>& s2, const pair<int, int>& g1, const pair<int, int>& g2, const pair<int, int>& Rg)
{
	if ((s2.first - s1.first) * (s1.first - g1.first) < 0 && (s2.second - s1.second) * (s1.second - g1.second) < 0) // s1 in the middle
		return 0;
	else if ((s1.first - s2.first) * (s2.first - g2.first) < 0 && (s1.second - s2.second) * (s2.second - g2.second) < 0) // s2 in the middle
		return 0; 

	int cardinal1 = 0, cardinal2 = 0;
	if ((s1.first == s2.first && (s1.second - s2.second) * (s2.second - Rg.second) >= 0) ||
		(s1.first != s2.first && (s1.first - s2.first)*(s2.first - Rg.first) < 0))
	{
		if (Rg.first == g1.first)
			cardinal1 = 1;
		if (Rg.second == g2.second)
			cardinal2 = 1;
	}
	else
	{
		if (Rg.second == g1.second)
			cardinal1 = 1;
		if (Rg.first == g2.first)
			cardinal2 = 1;
	}

	return cardinal1 + cardinal2;
}

//Compute rectangle corner Rs
pair<int, int> RectangleReasoning::getRs(const pair<int, int>& s1, const pair<int, int>& s2, const pair<int, int>& g1)
{
	int x, y;
	if (s1.first == g1.first)
		x = s1.first;
	else if (s1.first < g1.first)
		x = max(s1.first, s2.first);
	else
		x = min(s1.first, s2.first);
	if (s1.second == g1.second)
		y = s1.second;
	else if (s1.second < g1.second)
		y = max(s1.second, s2.second);
	else
		y = min(s1.second, s2.second);
	return make_pair(x, y);
}

//Compute rectangle corner Rg
pair<int, int> RectangleReasoning::getRg(const pair<int, int>& s1, const pair<int, int>& g1, const pair<int, int>& g2)
{
	int x, y;
	if (s1.first == g1.first)
		x = g1.first;
	else if (s1.first < g1.first)
		x = min(g1.first, g2.first);
	else
		x = max(g1.first, g2.first);
	if (s1.second == g1.second)
		y = g1.second;
	else if (s1.second < g1.second)
		y = min(g1.second, g2.second);
	else
		y = max(g1.second, g2.second);
	return make_pair(x, y);
}

//Compute start candidates for RM
list<int> RectangleReasoning::getStartCandidates(const Path& path, const MDD& mdd, int timestep)
{
	list<int> starts;
	for (int t = 0; t <= timestep; t++) //Find start that is single and Manhattan-optimal to conflicting location
	{
		if (mdd.levels[t].size() == 1 &&
			mdd.levels[t].front()->location == path[t].location &&
			instance.getManhattanDistance(path[t].location, path[timestep].location) == timestep - t)
			starts.push_back(t);
	}
	return starts;
}

//Compute goal candidates for RM
list<int> RectangleReasoning::getGoalCandidates(const Path& path, const MDD& mdd, int timestep)
{
	list<int> goals;
	for (int t = (int) path.size() - 1; t >= timestep; t--) //Find end that is single and Manhattan-optimal to conflicting location
	{
		if (mdd.levels[t].size() == 1 &&
			mdd.levels[t].front()->location == path[t].location &&
			instance.getManhattanDistance(path[t].location, path[timestep].location) == t - timestep)
			goals.push_back(t);
	}
	return goals;
}

int RectangleReasoning::getStartCandidate(const Path& path, int dir1, int dir2, int timestep)
{
	for (int t = timestep; t > 0; t--) //Find the earliest start that is single and Manhattan-optimal to conflicting location
	{
		if (path[t].location - path[t - 1].location != dir1 && path[t].location - path[t - 1].location != dir2)
			return t;
	}
	return 0;
}

int RectangleReasoning::getGoalCandidate(const Path& path, int dir1, int dir2, int timestep)
{
	for (int t = timestep; t < (int)path.size() - 1; t++) //Find the latest end that is single and Manhattan-optimal to conflicting location
	{
		if (path[t + 1].location - path[t].location != dir1 && path[t + 1].location - path[t].location != dir2)
			return t;
	}
	return (int)path.size() - 1;
}

/*int getRectangleTime(const Conflict& conflict, const std::vector<std::vector<PathEntry>*>& paths, int num_col)
{
	int s1 = paths[conflict.a1]->at(std::get<3>(conflict)).location;
	int s2 = paths[conflict.a2]->at(std::get<4>(conflict)).location;
	std::pair<int, int>  S1 = std::make_pair(s1 / num_col, s1 % num_col);
	std::pair<int, int> S2 = std::make_pair(s2 / num_col, s2 % num_col);
	std::pair<int, int> Rg = std::make_pair(std::get<2>(conflict) / num_col, std::get<2>(conflict) % num_col);
	std::pair<int, int> Rs = getRs(S1, S2, Rg);
	return std::get<3>(conflict) - abs(S1.first - Rs.first) - abs(S1.second - Rs.second);
}*/

bool RectangleReasoning::addModifiedBarrierConstraints(int a1, int a2, 
	const pair<int, int>& Rs, const pair<int, int>& Rg,
	const pair<int, int>& s1, const pair<int, int>& s2, int Rg_t,
	const MDD* mdd1, const MDD* mdd2,
	list<Constraint>& constraint1, list<Constraint>& constraint2)
{
	if ((s2.first - s1.first) * (s1.first - Rg.first) > 0 && (s2.second - s1.second) * (s1.second - Rg.second) > 0) // s1 in the middle
	{
		int Rs_t = Rg_t - instance.getManhattanDistance(Rs, Rg);
		// try horizontal first
		int offset = Rs.first > Rg.first? 1 : -1;
		bool found = hasNodeOnBarrier(mdd2, Rs.second, Rg.second, Rs.first + offset, Rs_t - 1, true);
		if (!found)
		{
			// first agent moves vertically and second agent moves horizontally
			if (!addModifiedHorizontalBarrierConstraint(a1, mdd1, Rg.first, Rs.second, Rg.second, Rg_t, constraint1))
				return false;
			if (!addModifiedVerticalBarrierConstraint(a2, mdd2, Rg.second, Rs.first, Rg.first, Rg_t, constraint2))
				return false;
			return true;
		}
		// try vertical then 
		offset = Rs.second > Rg.second ? 1 : -1;
		found = hasNodeOnBarrier(mdd2, Rs.first, Rg.first, Rs.second + offset, Rs_t - 1, false);
		if (!found)
		{
			// first agent moves horizontally and second agent moves vertically
			if (!addModifiedVerticalBarrierConstraint(a1, mdd1, Rg.second, Rs.first, Rg.first, Rg_t, constraint1))
				return false;
			if (!addModifiedHorizontalBarrierConstraint(a2, mdd2, Rg.first, Rs.second, Rg.second, Rg_t, constraint2))
				return false;
			return true;
		}
	}
	else if ((s1.first - s2.first) * (s2.first - Rg.first) > 0 && (s1.second - s2.second) * (s2.second - Rg.second) > 0) // s2 in the middle
	{
		int Rs_t = Rg_t - instance.getManhattanDistance(Rs, Rg);
		// try horizontal first
		int offset = Rs.first > Rg.first ? 1 : -1;
		bool found = hasNodeOnBarrier(mdd1, Rs.second, Rg.second, Rs.first + offset, Rs_t - 1, true);
		if (!found)
		{
			// first agent moves horizontally and second agent moves vertically
			if (!addModifiedVerticalBarrierConstraint(a1, mdd1, Rg.second, Rs.first, Rg.first, Rg_t, constraint1))
				return false;
			if (!addModifiedHorizontalBarrierConstraint(a2, mdd2, Rg.first, Rs.second, Rg.second, Rg_t, constraint2))
				return false;
			return true;
		}
		// try vertical then 
		offset = Rs.second > Rg.second ? 1 : -1;
		found = hasNodeOnBarrier(mdd1, Rs.first, Rg.first, Rs.second + offset, Rs_t - 1, false);
		if (!found)
		{
			// first agent moves vertically and second agent moves horizontally
			if (!addModifiedHorizontalBarrierConstraint(a1, mdd1, Rg.first, Rs.second, Rg.second, Rg_t, constraint1))
				return false;
			if (!addModifiedVerticalBarrierConstraint(a2, mdd2, Rg.second, Rs.first, Rg.first, Rg_t, constraint2))
				return false;
			return true;
		}
	}
	else if (s1.first == s2.first)
	{
		if ((s1.second - s2.second) * (s2.second - Rg.second) >= 0)
		{
			// first agent moves horizontally and second agent moves vertically
			if (!addModifiedVerticalBarrierConstraint(a1, mdd1, Rg.second, Rs.first, Rg.first, Rg_t, constraint1))
				return false;
			if (!addModifiedHorizontalBarrierConstraint(a2, mdd2, Rg.first, Rs.second, Rg.second, Rg_t, constraint2))
				return false;
		}
		else
		{
			// first agent moves vertically and second agent moves horizontally
			if (!addModifiedHorizontalBarrierConstraint(a1, mdd1, Rg.first, Rs.second, Rg.second, Rg_t, constraint1))
			{
				return false;
			}
			if (!addModifiedVerticalBarrierConstraint(a2, mdd2, Rg.second, Rs.first, Rg.first, Rg_t, constraint2))
			{
				return false;
			}
		}
	}
	else if ((s1.first - s2.first)*(s2.first - Rg.first) >= 0)
	{
		// first agent moves vertically and second agent moves horizontally
		if (!addModifiedHorizontalBarrierConstraint(a1, mdd1, Rg.first, Rs.second, Rg.second, Rg_t, constraint1))
		{
			return false;
		}
		if (!addModifiedVerticalBarrierConstraint(a2, mdd2, Rg.second, Rs.first, Rg.first, Rg_t, constraint2))
		{
			return false;
		}
	}
	else
	{
		// first agent moves horizontally and second agent moves vertically
		if (!addModifiedVerticalBarrierConstraint(a1, mdd1, Rg.second, Rs.first, Rg.first, Rg_t, constraint1))
		{
			return false;
		}
		if (!addModifiedHorizontalBarrierConstraint(a2, mdd2, Rg.first, Rs.second, Rg.second, Rg_t, constraint2))
		{
			return false;
		}
	}
	return true;
}

// return true if the  barrier has MDD nodes
// Assume the barrier is horizontal (otherwise switch x and y)
// (x, y_start) and (x, y_end) are the endpoints of the barrier, and
// t_min and t_min + abs(y_start - y_end) are their corresponding timesteps.
bool RectangleReasoning::hasNodeOnBarrier(const MDD* mdd, int y_start, int y_end, int x, int t_min, bool horizontal) const
{
	int sign = y_start < y_end ? 1 : -1;
	int t_max = t_min + abs(y_start - y_end);
	int loc;
	for (int t2 = t_min + 1; t2 <= min(t_max, (int)mdd->levels.size() - 1); t2++)
	{
		if (horizontal)
			loc = instance.linearizeCoordinate(x, y_start + (t2 - t_min) * sign);
		else
			loc = instance.linearizeCoordinate(y_start + (t2 - t_min) * sign, x);
		MDDNode* it = nullptr;
		for (MDDNode* n : mdd->levels[t2])
		{
			if (n->location == loc)
			{
				return true;
			}
		}
	}
	return false;
}


// add a horizontal modified barrier constraint
bool RectangleReasoning::addModifiedHorizontalBarrierConstraint(int agent, const MDD* mdd, int x,
	int Ri_y, int Rg_y, int Rg_t, list<Constraint>& constraints)
{
	int sign = Ri_y < Rg_y ? 1 : -1;
	int Ri_t = Rg_t - abs(Ri_y - Rg_y);
	int t1 = -1;
	int t_min = max(Ri_t, 0);
	int t_max = min(Rg_t, (int)mdd->levels.size() - 1);
	for (int t2 = t_min; t2 <= t_max; t2++)
	{
		int loc = instance.linearizeCoordinate(x,  (Ri_y + (t2 - Ri_t) * sign));
		MDDNode* it = nullptr;
		for (MDDNode* n : mdd->levels[t2])
		{
			if (n->location == loc)
			{
				it = n;
				break;
			}
		}
		if (it == nullptr && t1 >= 0) // add constraints [t1, t2)
		{
			int loc1 = instance.linearizeCoordinate(x, (Ri_y + (t1 - Ri_t) * sign));
			int loc2 = instance.linearizeCoordinate(x, (Ri_y + (t2 - 1 - Ri_t) * sign));
			constraints.emplace_back(agent, loc1, loc2, t2 - 1, constraint_type::BARRIER);
			t1 = -1;
			continue;
		}
		else if (it != nullptr && t1 < 0)
		{
			t1 = t2;
		}
		if (it != nullptr && t2 == t_max)
		{
			int loc1 = instance.linearizeCoordinate(x, (Ri_y + (t1 - Ri_t) * sign));
			constraints.emplace_back(agent, loc1, loc, t2, constraint_type::BARRIER); // add constraints [t1, t2]
		}
	}
	if (constraints.empty())
	{
		// std::cout << "Fail to add modified barrier constraints!" << std::endl;
		return false;
	}
	else
		return true;
}


// add a vertival modified barrier constraint
bool RectangleReasoning::addModifiedVerticalBarrierConstraint(int agent, const MDD* mdd, int y,
	int Ri_x, int Rg_x, int Rg_t, list<Constraint>& constraints)
{
	int sign = Ri_x < Rg_x ? 1 : -1;
	int Ri_t = Rg_t - abs(Ri_x - Rg_x);
	int t1 = -1;
	int t_min = max(Ri_t, 0);
	int t_max = min(Rg_t, (int)mdd->levels.size() - 1);
	for (int t2 = t_min; t2 <= t_max; t2++)
	{
		int loc = instance.linearizeCoordinate((Ri_x + (t2 - Ri_t) * sign), y);
		MDDNode* it = nullptr;
		for (MDDNode* n : mdd->levels[t2])
		{
			if (n->location == loc)
			{
				it = n;
				break;
			}
		}
		if (it == nullptr && t1 >= 0) // add constraints [t1, t2)
		{
			int loc1 = instance.linearizeCoordinate((Ri_x + (t1 - Ri_t) * sign), y);
			int loc2 = instance.linearizeCoordinate((Ri_x + (t2 - 1 - Ri_t) * sign), y);
			constraints.emplace_back(agent, loc1, loc2, t2 - 1, constraint_type::BARRIER);
			t1 = -1;
			continue;
		}
		else if (it != nullptr && t1 < 0)
		{
			t1 = t2;
		}
		if (it != nullptr && t2 == t_max)
		{
			int loc1 = instance.linearizeCoordinate((Ri_x + (t1 - Ri_t) * sign), y);
			constraints.emplace_back(agent, loc1, loc, t2, constraint_type::BARRIER); // add constraints [t1, t2]
		}
	}
	if (constraints.empty())
	{
		// std::cout << "Fail to add modified barrier constraints!" << std::endl;
		return false;
	}
	else
		return true;
}

bool RectangleReasoning::blocked(const Path& path, const list<Constraint>& constraints)
{
	for (auto constraint : constraints)
	{
		int a, x, y, t;
		constraint_type type;
		tie(a, x, y, t, type) = constraint;
		assert(type == constraint_type::BARRIER);
		int x1 = instance.getRowCoordinate(x), y1 = instance.getColCoordinate(x);
		int x2 = instance.getRowCoordinate(y), y2 = instance.getColCoordinate(y);
		if (x1 == x2)
		{
			if (y1 < y2)
			{
				for (int i = 0; i <= min(y2 - y1, t); i++)
				{
					if (traverse(path, instance.linearizeCoordinate(x1, y2 - i), t - i))
						return true;
				}
			}
			else
			{
				for (int i = 0; i <= min(y1 - y2, t); i++)
				{
					if (traverse(path, instance.linearizeCoordinate(x1, y2 + i), t - i))
						return true;
				}
			}
		}
		else // y1== y2
		{
			if (x1 < x2)
			{
				for (int i = 0; i <= min(x2 - x1, t); i++)
				{
					if (traverse(path, instance.linearizeCoordinate(x2 - i, y1), t - i))
						return true;
				}
			}
			else
			{
				for (int i = 0; i <= min(x1 - x2, t); i++)
				{
					if (traverse(path, instance.linearizeCoordinate(x2 + i, y1), t - i))
						return true;
				}
			}
		}
	}
	return false;
}

bool RectangleReasoning::traverse(const Path& path, int loc, int t)
{
	if (t >= (int)path.size())
		return loc == path.back().location;
	else return t >= 0 && path[t].location == loc;
}
