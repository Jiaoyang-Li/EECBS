#include "IncrementalPairwiseMutexPropagation.hpp"
#include "MDD.h"

class MutexReasoning{
public:
	double accumulated_runtime = 0;
	MutexReasoning(const Instance& instance, const vector<ConstraintTable>& initial_constraints) : 
		instance(instance), initial_constraints(initial_constraints) {}
	shared_ptr<Conflict> run(int a1, int a2, CBSNode& node, MDD* mdd_1, MDD* mdd_2);

	vector < SingleAgentSolver* > search_engines;  // used to find (single) agents' paths and mdd

private:
  const Instance& instance;
  const vector<ConstraintTable>& initial_constraints;
  // TODO using MDDs from cache
  // A problem can be whether the modified MDD still being safe for other modules..

  // (cons_hasher_0, cons_hasher_1) -> Constraint
  // Invariant: cons_hasher_0.a < cons_hasher_1.a
  unordered_map<ConstraintsHasher,
                unordered_map<ConstraintsHasher, shared_ptr<Conflict>, ConstraintsHasher::Hasher, ConstraintsHasher::EqNode>,
                ConstraintsHasher::Hasher, ConstraintsHasher::EqNode
                > lookupTable;

  shared_ptr<Conflict> findMutexConflict(int a1, int a2, CBSNode& node, MDD* mdd_1, MDD* mdd_2);
};

// other TODOs
// TODO duplicated cardinal test in classify conflicts
