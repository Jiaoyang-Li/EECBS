// This is used by SIPP
#pragma once
#include "ConstraintTable.h"

typedef tuple<int, int, bool> Interval; // [t_min, t_max), num_of_collisions

class ReservationTable
{
public:
    const ConstraintTable& constraint_table;
    double runtime;

	ReservationTable(const ConstraintTable& constraint_table, int goal_location):
            constraint_table(constraint_table), goal_location(goal_location), sit(constraint_table.map_size) {}

    list<tuple<int, int, int, bool, bool> > get_safe_intervals(int from, int to, int lower_bound, int upper_bound);
    Interval get_first_safe_interval(size_t location);
    bool find_safe_interval(Interval& interval, size_t location, int t_min);

private:
    int goal_location;
    // Safe Interval Table (SIT)
    typedef vector< list<Interval> > SIT;
    SIT sit; // location -> [t_min, t_max), num_of_collisions
    void insert2SIT(int location, int t_min, int t_max);
    void insertSoftConstraint2SIT(int location, int t_min, int t_max);
    // void mergeIntervals(list<Interval >& intervals) const;
    void updateSIT(int location); // update SIT at the given location
    int get_earliest_arrival_time(int from, int to, int lower_bound, int upper_bound) const;
    int get_earliest_no_collision_arrival_time(int from, int to, const Interval& interval,
                                               int lower_bound, int upper_bound) const;
};