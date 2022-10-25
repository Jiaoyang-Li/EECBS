#include "MutexReasoning.h"
#include "ConstraintPropagation.h"


shared_ptr<Conflict> MutexReasoning::run(int a1, int a2, CBSNode& node, MDD* mdd_1, MDD* mdd_2)
{
	clock_t t = clock();
	auto conflict = findMutexConflict(a1, a2, node, mdd_1, mdd_2);
	accumulated_runtime += (double)(clock() - t) / CLOCKS_PER_SEC;
	return conflict;
}


shared_ptr<Conflict> MutexReasoning::findMutexConflict(int a1, int a2, CBSNode& node, MDD* mdd_1, MDD* mdd_2){
  ConstraintPropagation cp(mdd_1, mdd_2);
  cp.init_mutex();
  cp.fwd_mutex_prop();

  if (cp._feasible(mdd_1->levels.size() - 1, mdd_2->levels.size() - 1) >= 0){
    return nullptr;
  }

  bool swapped = false;
  if (a1 > a2){
    std::swap(a1, a2);
    std::swap(mdd_1, mdd_2);
    swapped = true;
  }

	ConstraintsHasher c_1(a1, &node);
	ConstraintsHasher c_2(a2, &node);

  shared_ptr<Conflict> mutex_conflict = nullptr;
  if (lookupTable.find(c_1) != lookupTable.end()){
    if (lookupTable[c_1].find(c_2) != lookupTable[c_1].end()){
      mutex_conflict = lookupTable[c_1][c_2];
    }
  }

  if (mutex_conflict == nullptr){
    // generate constraint;
    mutex_conflict = make_shared<Conflict>();
    mutex_conflict->mutexConflict(a1, a2);

    MDD mdd_1_cpy(*mdd_1);
    MDD mdd_2_cpy(*mdd_2);

    ConstraintTable ct1(initial_constraints[a1]);
    ConstraintTable ct2(initial_constraints[a2]);

      ct1.insert2CT(node, a1);
      ct2.insert2CT(node, a2);
    auto ip = IPMutexPropagation(&mdd_1_cpy, &mdd_2_cpy, search_engines[a1], search_engines[a2],
                                 ct1, ct2);
    con_vec a;
    con_vec b;
    std::tie(a, b) = ip.gen_constraints();

    for (auto con:a){
      get<0>(con) = a1;
      mutex_conflict->constraint1.push_back(con);
    }

    for (auto con:b){
      get<0>(con) = a2;
      mutex_conflict->constraint2.push_back(con);
    }
    // mutex_conflict->constraint1 = list<Constraint>(a.begin(), a.end());
    // mutex_conflict->constraint2 = list<Constraint>(b.begin(), b.end());

    lookupTable[c_1][c_2] = mutex_conflict;
  }

  // prepare for return
  shared_ptr<Conflict> conflict_to_return = make_shared<Conflict>(*mutex_conflict);

  if (swapped){
    std::swap(conflict_to_return->a1, conflict_to_return->a2);
    std::swap(conflict_to_return->constraint1, conflict_to_return->constraint2);
  }

  return conflict_to_return;

}
