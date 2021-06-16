#pragma once
#include "common.h"
#include "CBSNode.h"
class ConstraintTable
{
public:
    int length_min = 0;
    int length_max = MAX_TIMESTEP;
    size_t num_col;
    size_t map_size;

    int getHoldingTime(int location, int earliest_timestep) const; // the earliest timestep that the agent can hold the location after earliest_timestep
    int getMaxTimestep() const; // everything is static after the max timestep
    int getLastCollisionTimestep(int location) const;
    // void clear(){ct.clear(); cat_small.clear(); cat_large.clear(); landmarks.clear(); length_min = 0, length_max = INT_MAX; latest_timestep = 0;}

    bool constrained(size_t loc, int t) const;
    bool constrained(size_t curr_loc, size_t next_loc, int next_t) const;
    int getNumOfConflictsForStep(size_t curr_id, size_t next_id, int next_timestep) const;
    bool hasConflictForStep(size_t curr_id, size_t next_id, int next_timestep) const;
    bool hasEdgeConflict(size_t curr_id, size_t next_id, int next_timestep) const;
    int getFutureNumOfCollisions(int loc, int t) const;

    ConstraintTable(size_t num_col, size_t map_size) : num_col(num_col), map_size(map_size) {}
    ConstraintTable(const ConstraintTable& other) { copy(other); }
    ~ConstraintTable() = default;

    void copy(const ConstraintTable& other);
    void init(const ConstraintTable& other) { copy(other); }
    void clear()
    {
        ct.clear();
        landmarks.clear();
        cat.clear();
    }
    void insert2CT(const HLNode& node, int agent); // build the constraint table for the given agent at the give node
    void insert2CT(const list<Constraint>& constraints, int agent); // insert constraints for the given agent to the constraint table
    void insert2CT(const Path& path); // insert a path to the constraint table
    void insert2CT(size_t loc, int t_min, int t_max); // insert a vertex constraint to the constraint table
    void insert2CT(size_t from, size_t to, int t_min, int t_max); // insert an edge constraint to the constraint table
    void insert2CAT(int agent, const vector<Path*>& paths); // build the conflict avoidance table using a set of paths
    void insert2CAT(const Path& path); // insert a path to the collision avoidance table
    //int getCATMaxTimestep() const {return cat_max_timestep;}

protected:
    friend class ReservationTable;
    typedef unordered_map<size_t, list< pair<int, int> > > CT; // constraint table
    CT ct; // location -> time range, or edge -> time range
    int ct_max_timestep = 0;
    // typedef unordered_map<size_t, set< pair<int, int> > > CAT; // conflict avoidance table // location -> time range, or edge -> time range
    typedef vector< vector<bool> > CAT;
    CAT cat;
    int cat_max_timestep = 0;
    vector<int> cat_goals;
    map<int, size_t> landmarks; // <timestep, location>: the agent must be at the given location at the given timestep

    void insertLandmark(size_t loc, int t); // insert a landmark, i.e., the agent has to be at the given location at the given timestep
    list<pair<int, int> > decodeBarrier(int B1, int B2, int t) const;
    inline size_t getEdgeIndex(size_t from, size_t to) const { return (1 + from) * map_size + to; }
};
