#include "SIPP.h"

void SIPP::updatePath(const LLNode* goal, vector<PathEntry> &path)
{
    //num_collisions = goal->num_of_conflicts;
    path.resize(goal->timestep + 1);
    // num_of_conflicts = goal->num_of_conflicts;

    const auto* curr = goal;
    while (curr->parent != nullptr) // non-root node
    {
        const auto* prev = curr->parent;
        int t = prev->timestep + 1;
        while (t < curr->timestep)
        {
            path[t].location = prev->location; // wait at prev location
            t++;
        }
        path[curr->timestep].location = curr->location; // move to curr location
        curr = prev;
    }
    assert(curr->timestep == 0);
    path[0].location = curr->location;
}


// find path by A*
// Returns a path that minimizes the collisions with the paths in the path table, breaking ties by the length
Path SIPP::findPath(const ConstraintTable& constraint_table)
{
    reset();
    //Path path = findNoCollisionPath(constraint_table);
    //if (!path.empty())
    //    return path;
    ReservationTable reservation_table(constraint_table, goal_location);
    Path path;
    Interval interval = reservation_table.get_first_safe_interval(start_location);
    if (get<0>(interval) > 0)
        return path;
    auto holding_time = constraint_table.getHoldingTime(goal_location, constraint_table.length_min);
    auto last_target_collision_time = constraint_table.getLastCollisionTimestep(goal_location);
    // generate start and add it to the OPEN & FOCAL list
    auto h = max(max(my_heuristic[start_location], holding_time), last_target_collision_time + 1);
    auto start = new SIPPNode(start_location, 0, h, nullptr, 0, get<1>(interval), get<1>(interval),
                              get<2>(interval), get<2>(interval));
    pushNodeToFocal(start);

    while (!focal_list.empty())
    {
        auto* curr = focal_list.top();
        focal_list.pop();
        curr->in_openlist = false;
        num_expanded++;
        assert(curr->location >= 0);
        // check if the popped node is a goal
        if (curr->is_goal)
        {
            updatePath(curr, path);
            break;
        }
        else if (curr->location == goal_location && // arrive at the goal location
                 !curr->wait_at_goal && // not wait at the goal location
                 curr->timestep >= holding_time) // the agent can hold the goal location afterward
        {
            int future_collisions = constraint_table.getFutureNumOfCollisions(curr->location, curr->timestep);
            if (future_collisions == 0)
            {
                updatePath(curr, path);
                break;
            }
            // generate a goal node
            auto goal = new SIPPNode(*curr);
            goal->is_goal = true;
            goal->h_val = 0;
            goal->num_of_conflicts += future_collisions;
            // try to retrieve it from the hash table
            if (dominanceCheck(goal))
                pushNodeToFocal(goal);
            else
                delete goal;
        }

        for (int next_location : instance.getNeighbors(curr->location)) // move to neighboring locations
        {
            for (auto & i : reservation_table.get_safe_intervals(
                    curr->location, next_location, curr->timestep + 1, curr->high_expansion + 1))
            {
                int next_high_generation, next_timestep, next_high_expansion;
                bool next_v_collision, next_e_collision;
                tie(next_high_generation, next_timestep, next_high_expansion, next_v_collision, next_e_collision) = i;
                if (next_timestep + my_heuristic[next_location] > constraint_table.length_max)
                    break;
                auto next_collisions = curr->num_of_conflicts +
                                       (int)curr->collision_v * max(next_timestep - curr->timestep - 1, 0) // wait time
                                       + (int)next_v_collision + (int)next_e_collision;
                auto next_h_val = max(my_heuristic[next_location], (next_collisions > 0?
                                                                    holding_time : curr->getFVal()) - next_timestep); // path max
                // generate (maybe temporary) node
                auto next = new SIPPNode(next_location, next_timestep, next_h_val, curr, next_timestep,
                                         next_high_generation, next_high_expansion, next_v_collision, next_collisions);
                // try to retrieve it from the hash table
                if (dominanceCheck(next))
                    pushNodeToFocal(next);
                else
                    delete next;
            }
        }  // end for loop that generates successors
        // wait at the current location
        if (curr->high_expansion == curr->high_generation and
            reservation_table.find_safe_interval(interval, curr->location, curr->high_expansion) and
            get<0>(interval) + curr->h_val <= reservation_table.constraint_table.length_max)
        {
            auto next_timestep = get<0>(interval);
            auto next_h_val = max(curr->h_val, (get<2>(interval) ? holding_time : curr->getFVal()) - next_timestep); // path max
            auto next_collisions = curr->num_of_conflicts +
                                   (int)curr->collision_v * max(next_timestep - curr->timestep - 1, 0) + (int)get<2>(interval);
            auto next = new SIPPNode(curr->location, next_timestep, next_h_val, curr, next_timestep,
                                     get<1>(interval), get<1>(interval), get<2>(interval),
                                     next_collisions);
            next->wait_at_goal = (curr->location == goal_location);
            if (dominanceCheck(next))
                pushNodeToFocal(next);
            else
                delete next;
        }
    }  // end while loop

    //if (path.empty())
    //{
    //    printSearchTree();
    //}
    releaseNodes();
    return path;
}
Path SIPP::findOptimalPath(const HLNode& node, const ConstraintTable& initial_constraints,
                           const vector<Path*>& paths, int agent, int lowerbound)
{
    return findSuboptimalPath(node, initial_constraints, paths, agent, lowerbound, 1).first;
}
// find path by SIPP
// Returns a shortest path that satisfies the constraints of the give node  while
// minimizing the number of internal conflicts (that is conflicts with known_paths for other agents found so far).
// lowerbound is an underestimation of the length of the path in order to speed up the search.
pair<Path, int> SIPP::findSuboptimalPath(const HLNode& node, const ConstraintTable& initial_constraints,
                                         const vector<Path*>& paths, int agent, int lowerbound, double w)
{
    reset();
    this->w = w;

    // build constraint table
    auto t = clock();
    ConstraintTable constraint_table(initial_constraints);
    constraint_table.insert2CT(node, agent);
    runtime_build_CT = (double)(clock() - t) / CLOCKS_PER_SEC;
    int holding_time = constraint_table.getHoldingTime(goal_location, constraint_table.length_min);
    t = clock();
    constraint_table.insert2CAT(agent, paths);
    runtime_build_CAT = (double)(clock() - t) / CLOCKS_PER_SEC;

    // build reservation table
    ReservationTable reservation_table(constraint_table, goal_location);

    Path path;
    num_expanded = 0;
    num_generated = 0;
    Interval interval = reservation_table.get_first_safe_interval(start_location);
    if (get<0>(interval) > 0)
        return {path, 0};

    // generate start and add it to the OPEN list
    auto start = new SIPPNode(start_location, 0, max(my_heuristic[start_location], holding_time), nullptr, 0,
                              get<1>(interval), get<1>(interval), get<2>(interval), get<2>(interval));
    min_f_val = max(holding_time, max((int)start->getFVal(), lowerbound));
    pushNodeToOpenAndFocal(start);

    while (!open_list.empty())
    {
        updateFocalList(); // update FOCAL if min f-val increased
        SIPPNode* curr = focal_list.top(); focal_list.pop();
        open_list.erase(curr->open_handle);
        curr->in_openlist = false;
        num_expanded++;

        // check if the popped node is a goal node
        if (curr->location == goal_location && // arrive at the goal location
            !curr->wait_at_goal && // not wait at the goal location
            curr->timestep >= holding_time) // the agent can hold the goal location afterward
        {
            updatePath(curr, path);
            break;
        }

        for (int next_location : instance.getNeighbors(curr->location)) // move to neighboring locations
        {
            for (auto & i : reservation_table.get_safe_intervals(
                    curr->location, next_location, curr->timestep + 1, curr->high_expansion + 1))
            {
                int next_high_generation, next_timestep, next_high_expansion;
                bool next_v_collision, next_e_collision;
                tie(next_high_generation, next_timestep, next_high_expansion, next_v_collision, next_e_collision) = i;
                // compute cost to next_id via curr node
                int next_g_val = next_timestep;
                int next_h_val = max(my_heuristic[next_location], curr->getFVal() - next_g_val);  // path max
                if (next_g_val + next_h_val > reservation_table.constraint_table.length_max)
                    continue;
                int next_conflicts = curr->num_of_conflicts +
                                     (int)curr->collision_v * max(next_timestep - curr->timestep - 1, 0) +
                                     + (int)next_v_collision + (int)next_e_collision;
                auto next = new SIPPNode(next_location, next_g_val, next_h_val, curr, next_timestep,
                                         next_high_generation, next_high_expansion, next_v_collision, next_conflicts);
                if (dominanceCheck(next))
                    pushNodeToOpenAndFocal(next);
                else
                    delete next;
            }
        }  // end for loop that generates successors

        // wait at the current location
        if (curr->high_expansion == curr->high_generation and
            reservation_table.find_safe_interval(interval, curr->location, curr->high_expansion) and
            get<0>(interval) + curr->h_val <= reservation_table.constraint_table.length_max)
        {
            auto next_timestep = get<0>(interval);
            int next_h_val = max(my_heuristic[curr->location], curr->getFVal() - next_timestep);  // path max
            auto next_collisions = curr->num_of_conflicts +
                                   (int)curr->collision_v * max(next_timestep - curr->timestep - 1, 0) // wait time
                                   + (int)get<2>(interval);
            auto next = new SIPPNode(curr->location, next_timestep, next_h_val, curr, next_timestep,
                                     get<1>(interval), get<1>(interval), get<2>(interval), next_collisions);
            if (curr->location == goal_location)
                next->wait_at_goal = true;
            if (dominanceCheck(next))
                pushNodeToOpenAndFocal(next);
            else
                delete next;
        }
    }  // end while loop

    // no path found
    releaseNodes();
    return {path, min_f_val};
}
/*Path SIPP::findNoCollisionPath(const ConstraintTable& constraint_table)
{
    reset();
    ReservationTable reservation_table(constraint_table, goal_location);
    Path path;
    Interval interval = reservation_table.get_first_safe_interval(start_location);
    if (get<0>(interval) > 0 or get<2>(interval))
        return path;
    auto holding_time = max(constraint_table.getHoldingTime(goal_location, constraint_table.length_min),
                            constraint_table.getLastCollisionTimestep(goal_location) + 1);
    // generate start and add it to the OPEN & FOCAL list

    auto start = new SIPPNode(start_location, 0, max(my_heuristic[start_location], holding_time),
                              nullptr, 0, interval, 0);
    pushNodeToFocal(start);
    while (!focal_list.empty())
    {
        auto* curr = focal_list.top();
        focal_list.pop();
        curr->in_openlist = false;
        num_expanded++;
        assert(curr->location >= 0);
        // check if the popped node is a goal
        if (curr->location == goal_location && // arrive at the goal location
            !curr->wait_at_goal && // not wait at the goal location
            curr->timestep >= holding_time && // the agent can hold the goal location afterward
            constraint_table.getFutureNumOfCollisions(curr->location, curr->timestep) == 0) // no future collisions
        {
            updatePath(curr, path);
            break;
        }
        for (int next_location : instance.getNeighbors(curr->location)) // move to neighboring locations
        {
            int next_h_val = my_heuristic[next_location];
            for (auto& interval : reservation_table.get_safe_intervals(
                    curr->location, next_location, curr->timestep + 1, get<1>(curr->interval) + 1))
            {
                if (get<2>(interval.first))
                    continue;
                if (interval.second + next_h_val > constraint_table.length_max)
                    break;
                generateChildToFocal(interval.first, curr, next_location, interval.second, next_h_val);
            }
        }  // end for loop that generates successors
        // wait at the current location
        bool found = reservation_table.find_safe_interval(interval, curr->location, get<1>(curr->interval));
        if (found and !get<2>(interval))
        {
            generateChildToFocal(interval, curr, curr->location, get<0>(interval), curr->h_val);
        }
    }  // end while loop

    //if (path.empty())
    //    printSearchTree();
    releaseNodes();
    return path;
}*/
// TODO:: currently this is implemented in SIPP inefficiently
int SIPP::getTravelTime(int start, int end, const ConstraintTable& constraint_table, int upper_bound)
{
    reset();
    min_f_val = -1; // this disables focal list
    int length = MAX_TIMESTEP;
    auto root = new SIPPNode(start, 0, compute_heuristic(start, end), nullptr, 0, 1, 1, 0, 0);
    pushNodeToOpenAndFocal(root);
    auto static_timestep = constraint_table.getMaxTimestep(); // everything is static after this timestep
    while (!open_list.empty())
    {
        auto curr = open_list.top(); open_list.pop();
        if (curr->location == end)
        {
            length = curr->g_val;
            break;
        }
        list<int> next_locations = instance.getNeighbors(curr->location);
        next_locations.emplace_back(curr->location);
        for (int next_location : next_locations)
        {
            int next_timestep = curr->timestep + 1;
            int next_g_val = curr->g_val + 1;
            if (static_timestep <= curr->timestep)
            {
                if (curr->location == next_location)
                {
                    continue;
                }
                next_timestep--;
            }
            if (!constraint_table.constrained(next_location, next_timestep) &&
                !constraint_table.constrained(curr->location, next_location, next_timestep))
            {  // if that grid is not blocked
                int next_h_val = compute_heuristic(next_location, end);
                if (next_g_val + next_h_val >= upper_bound) // the cost of the path is larger than the upper bound
                    continue;
                auto next = new SIPPNode(next_location, next_g_val, next_h_val, nullptr, next_timestep,
                                         next_timestep + 1, next_timestep + 1, 0, 0);
                if (dominanceCheck(next))
                    pushNodeToOpenAndFocal(next);
                else
                    delete next;
            }
        }
    }
    releaseNodes();
    return length;
    /*int length = INT_MAX;
    // generate a heap that can save nodes (and a open_handle)
    pairing_heap< SIPPNode*, compare<SIPPNode::compare_node> > open_list;
    // boost::heap::pairing_heap< AStarNode*, boost::heap::compare<LLNode::compare_node> >::handle_type open_handle;
    unordered_set<SIPPNode*, SIPPNode::NodeHasher, SIPPNode::eqnode> nodes;

    Interval interval = reservation_table.get_first_safe_interval(start);
    assert(get<0>(interval) == 0);
    auto root = new SIPPNode(start, 0, instance.getManhattanDistance(start, end), nullptr, 0, interval);
    root->open_handle = open_list.push(root);  // add root to heap
    nodes.insert(root);       // add root to hash_table (nodes)

    while (!open_list.empty())
    {
        auto curr = open_list.top(); open_list.pop();
        if (curr->location == end)
        {
            length = curr->g_val;
            break;
        }
        for (int next_location : instance.getNeighbors(curr->location))
        {
            if ((curr->location == blocked.first && next_location == blocked.second) ||
                (curr->location == blocked.second && next_location == blocked.first)) // use the prohibited edge
            {
                continue;
            }

            for (auto interval : reservation_table.get_safe_intervals(
                curr->location, next_location, curr->timestep + 1, get<1>(curr->interval) + 1))
            {
                int next_timestep = max(curr->timestep + 1, (int)get<0>(interval));
                int next_g_val = next_timestep;
                int next_h_val = instance.getManhattanDistance(next_location, end);
                if (next_g_val + next_h_val >= upper_bound) // the cost of the path is larger than the upper bound
                    continue;
                auto next = new SIPPNode(next_location, next_g_val, next_h_val, nullptr, next_timestep, interval);
                auto it = nodes.find(next);
                if (it == nodes.end())
                {  // add the newly generated node to heap and hash table
                    next->open_handle = open_list.push(next);
                    nodes.insert(next);
                }
                else {  // update existing node's g_val if needed (only in the heap)
                    delete(next);  // not needed anymore -- we already generated it before
                    auto existing_next = *it;
                    if (existing_next->g_val > next_g_val)
                    {
                        existing_next->g_val = next_g_val;
                        existing_next->timestep = next_timestep;
                        open_list.update(existing_next->open_handle);
                    }
                }
            }
        }
    }
    open_list.clear();
    for (auto node : nodes)
    {
        delete node;
    }
    nodes.clear();
    return length;*/
}

void SIPP::updateFocalList()
{
    auto open_head = open_list.top();
    if (open_head->getFVal() > min_f_val)
    {
        int new_min_f_val = (int)open_head->getFVal();
        for (auto n : open_list)
        {
            if (n->getFVal() > w * min_f_val && n->getFVal() <= w * new_min_f_val)
                n->focal_handle = focal_list.push(n);
        }
        min_f_val = new_min_f_val;
    }
}

inline void SIPP::pushNodeToOpenAndFocal(SIPPNode* node)
{
    num_generated++;
    node->open_handle = open_list.push(node);
    node->in_openlist = true;
    if (node->getFVal() <= w * min_f_val)
        node->focal_handle = focal_list.push(node);
    allNodes_table[node].push_back(node);
}
inline void SIPP::pushNodeToFocal(SIPPNode* node)
{
    num_generated++;
    allNodes_table[node].push_back(node);
    node->in_openlist = true;
    node->focal_handle = focal_list.push(node); // we only use focal list; no open list is used
}
inline void SIPP::eraseNodeFromLists(SIPPNode* node)
{
    if (open_list.empty())
    { // we only have focal list
        focal_list.erase(node->focal_handle);
    }
    else if (focal_list.empty())
    {  // we only have open list
        open_list.erase(node->open_handle);
    }
    else
    { // we have both open and focal
        open_list.erase(node->open_handle);
        if (node->getFVal() <= w * min_f_val)
            focal_list.erase(node->focal_handle);
    }
}
void SIPP::releaseNodes()
{
    open_list.clear();
    focal_list.clear();
    for (auto & node_list : allNodes_table)
        for (auto n : node_list.second)
            delete n;
    allNodes_table.clear();
    for (auto n : useless_nodes)
        delete n;
    useless_nodes.clear();
}

// return true iff the new node is not dominated by any old node
bool SIPP::dominanceCheck(SIPPNode* new_node)
{
    auto ptr = allNodes_table.find(new_node);
    if (ptr == allNodes_table.end())
        return true;
    for (auto & old_node : ptr->second)
    {
        if (old_node->timestep <= new_node->timestep and
            old_node->num_of_conflicts <= new_node->num_of_conflicts)
        { // the new node is dominated by the old node
            return false;
        }
        else if (old_node->timestep >= new_node->timestep and
                 old_node->num_of_conflicts >= new_node->num_of_conflicts) // the old node is dominated by the new node
        { // delete the old node
            if (old_node->in_openlist) // the old node has not been expanded yet
                eraseNodeFromLists(old_node); // delete it from open and/or focal lists
            useless_nodes.push_back(old_node);
            ptr->second.remove(old_node);
            num_generated--; // this is because we later will increase num_generated when we insert the new node into lists.
            return true;
        }
        else if(old_node->timestep < new_node->high_expansion and new_node->timestep < old_node->high_expansion)
        { // intervals overlap --> we need to split the node to make them disjoint
            if (old_node->timestep <= new_node->timestep)
            {
                assert(old_node->num_of_conflicts > new_node->num_of_conflicts);
                old_node->high_expansion = new_node->timestep;
            }
            else // i.e., old_node->timestep > new_node->timestep
            {
                assert(old_node->num_of_conflicts <= new_node->num_of_conflicts);
                new_node->high_expansion = old_node->timestep;
            }
        }
    }
    return true;
}