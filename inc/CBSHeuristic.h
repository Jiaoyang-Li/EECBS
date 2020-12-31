#pragma once
#include "MDD.h"
#include "RectangleReasoning.h"
#include "CorridorReasoning.h"


enum heuristics_type { ZERO, CG, DG, WDG, GLOBAL, PATH, LOCAL, CONFLICT, STRATEGY_COUNT }; //  GREEDY,

struct HTableEntry // look-up table entry 
{
	int a1{};
	int a2{};
	HLNode* n{};

	HTableEntry() = default;
	HTableEntry(int a1, int a2, HLNode* n) : a1(a1), a2(a2), n(n) {};

	struct EqNode
	{
		bool operator() (const HTableEntry& h1, const HTableEntry& h2) const
		{
			std::set<Constraint> cons1[2], cons2[2];
			auto curr = h1.n;
			while (curr->parent != nullptr)
			{
				if (get<4>(curr->constraints.front()) == constraint_type::LEQLENGTH ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_VERTEX ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_EDGE) {
					for (auto con : curr->constraints)
					{
						cons1[0].insert(con);
						cons2[0].insert(con);
					}
				}
				else {
					if (get<0>(curr->constraints.front()) == h1.a1)
						for (auto con : curr->constraints)
							cons1[0].insert(con);
					else if (get<0>(curr->constraints.front()) == h1.a2)
						for (auto con : curr->constraints)
							cons2[0].insert(con);
				}

				curr = curr->parent;
			}
			curr = h2.n;
			while (curr->parent != nullptr)
			{
				if (get<4>(curr->constraints.front()) == constraint_type::LEQLENGTH ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_VERTEX ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_EDGE) {
					for (auto con : curr->constraints)
					{
						cons1[1].insert(con);
						cons2[1].insert(con);
					}
				}
				else {
					if (get<0>(curr->constraints.front()) == h2.a1)
						for (auto con : curr->constraints)
							cons1[1].insert(con);
					else if (get<0>(curr->constraints.front()) == h2.a2)
						for (auto con : curr->constraints)
							cons2[1].insert(con);
				}

				curr = curr->parent;
			}
			if (cons1[0].size() != cons1[1].size() || cons2[0].size() != cons2[1].size())
				return false;

			if (!equal(cons1[0].begin(), cons1[0].end(), cons1[1].begin()))
				return false;
			return equal(cons2[0].begin(), cons2[0].end(), cons2[1].begin());
		}
	};


	struct Hasher
	{
		size_t operator()(const HTableEntry& entry) const
		{
			auto curr = entry.n;
			size_t cons1_hash = 0, cons2_hash = 0;
			while (curr->parent != nullptr)
			{
				if (get<0>(curr->constraints.front()) == entry.a1 ||
					get<4>(curr->constraints.front()) == constraint_type::LEQLENGTH ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_VERTEX ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_EDGE)
				{
					for (auto con : curr->constraints)
					{
						cons1_hash += 3 * std::hash<int>()(std::get<0>(con)) +
							5 * std::hash<int>()(std::get<1>(con)) +
							7 * std::hash<int>()(std::get<2>(con)) +
							11 * std::hash<int>()(std::get<3>(con));
					}
				}
				else if (get<0>(curr->constraints.front()) == entry.a2 ||
					get<4>(curr->constraints.front()) == constraint_type::LEQLENGTH ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_VERTEX ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_EDGE)
				{
					for (auto con : curr->constraints)
					{
						cons2_hash += 3 * std::hash<int>()(std::get<0>(con)) +
							5 * std::hash<int>()(std::get<1>(con)) +
							7 * std::hash<int>()(std::get<2>(con)) +
							11 * std::hash<int>()(std::get<3>(con));
					}
				}
				curr = curr->parent;
			}
			return cons1_hash ^ (cons2_hash << 1);
		}
	};
};

// <h value, num of CT nodes, 0> for CBS
// <h value, a1 f at root, a2 f at root> for ECBS
typedef unordered_map<HTableEntry, tuple<int, int, int>, HTableEntry::Hasher, HTableEntry::EqNode> HTable;


class CBSHeuristic
{
public:
	heuristics_type type;
	bool rectangle_reasoning; // using rectangle reasoning
	bool corridor_reasoning; // using corridor reasoning
	bool target_reasoning; // using target reasoning
	bool mutex_reasoning; // using mutex reasoning
	bool disjoint_splitting; // disjoint splitting
	bool PC; // prioritize conflicts

	bool save_stats;
	conflict_selection conflict_seletion_rule;
	node_selection node_selection_rule;

	double runtime_build_dependency_graph = 0;
	double runtime_solve_MVC = 0;
	uint64_t num_solve_MVC = 0;
	uint64_t num_merge_MDDs = 0;
	uint64_t num_solve_2agent_problems = 0;
	uint64_t num_memoization = 0; // number of times when memeorization helps

	 //stats
	list<tuple<int, int, const HLNode*, uint64_t, int> > sub_instances; 	// <agent 1, agent 2, node, number of expanded CT nodes, h value> 


	CBSHeuristic(int num_of_agents,
							const vector<Path*>& paths,
							vector<SingleAgentSolver*>& search_engines,
							const vector<ConstraintTable>& initial_constraints,
							MDDTable& mdd_helper) : num_of_agents(num_of_agents),
		paths(paths), search_engines(search_engines), initial_constraints(initial_constraints), mdd_helper(mdd_helper) {}
	
	void init()
	{
		if (type == heuristics_type::DG || type == heuristics_type::WDG)
		{
			lookupTable.resize(num_of_agents);
			for (int i = 0; i < num_of_agents; i++)
			{
				lookupTable[i].resize(num_of_agents);
			}
		}
	}

	void setInadmissibleHeuristics(heuristics_type h)
    {
        inadmissible_heuristic = h;
        if (h == heuristics_type::CONFLICT)
        {
            sum_distance_errors.assign(int(conflict_type::TYPE_COUNT), 0);  //(int(conflict_priority::PRIORITY_COUNT) * int(conflict_type::TYPE_COUNT), 0);
            sum_cost_errors.assign(int(conflict_type::TYPE_COUNT), 0);
            num_of_errors.assign(int(conflict_type::TYPE_COUNT), 1);
        }
        else
        {
            sum_distance_errors.assign(1, 0);
            sum_cost_errors.assign(1, 0);
            num_of_errors.assign(1, 0);
        }
    }
	bool computeInformedHeuristics(CBSNode& curr, double time_limit);
	bool computeInformedHeuristics(ECBSNode& curr, const vector<int>& min_f_vals, double time_limit);
	void computeQuickHeuristics(HLNode& curr);
	void updateOnlineHeuristicErrors(CBSNode& curr);
	void updateOnlineHeuristicErrors(ECBSNode& curr);
    void updateInadmissibleHeuristics(HLNode& curr);

	// EES heuristics
	// int MVConAllConflicts(HLNode& curr);
	// int greedyWDG(CBSNode& curr, double time_limit);
	double getCostError(int i = 0) const { return (num_of_errors[i] == 0)? 0 : sum_cost_errors[i] / num_of_errors[i]; }
	double getDistanceError(int i = 0) const { return (num_of_errors[i] == 0)? 0 : sum_distance_errors[i]  / num_of_errors[i]; }

	// void copyConflictGraph(HLNode& child, const HLNode& parent);
	void clear() { lookupTable.clear(); }

private:
    heuristics_type inadmissible_heuristic;

	int screen = 0;
	int num_of_agents;
	vector<vector<HTable> > lookupTable;

	// double sum_distance_error = 0;
	// double sum_cost_error = 0;
	// int num_of_errors = 0;
	// int initialize_online_learning = 0;
    vector<double> sum_distance_errors;
    vector<double> sum_cost_errors;
    vector<int> num_of_errors;

	double time_limit;
	int node_limit = 4;  // terminate the sub CBS solver if the number of its expanded nodes exceeds the node limit.
	double start_time;
	int ILP_node_threshold = 5; // when #nodes >= ILP_node_threshold, use ILP solver; otherwise, use DP solver
	int ILP_edge_threshold = 10; // when #edges >= ILP_edge_threshold, use ILP solver; otherwise, use DP solver
	int ILP_value_threshold = 32; // when value >= ILP_value_threshold, use ILP solver; otherwise, use DP solver
	// TODO: run some experiments to pick a good ILP_node_threshold
	const vector<Path*>& paths;
	const vector<SingleAgentSolver*>& search_engines;
	const vector<ConstraintTable>& initial_constraints;
	MDDTable& mdd_helper;

	void buildConflictGraph(vector<bool>& HG, const HLNode& curr);
	void buildCardinalConflictGraph(CBSNode& curr, vector<int>& CG, int& num_of_CGedges);
	bool buildDependenceGraph(CBSNode& node, vector<int>& CG, int& num_of_CGedges);
	bool buildWeightedDependencyGraph(CBSNode& curr, vector<int>& CG);
	bool buildWeightedDependencyGraph(ECBSNode& node, const vector<int>& min_f_vals, vector<int>& CG, int& delta_g);
	bool dependent(int a1, int a2, HLNode& node); // return true if the two agents are dependent
	pair<int, int> solve2Agents(int a1, int a2, const CBSNode& node, bool cardinal); // return h value and num of CT nodes
    tuple<int, int, int> solve2Agents(int a1, int a2, const ECBSNode& node); // return h value and num of CT nodes
	static bool SyncMDDs(const MDD &mdd1, const MDD& mdd2); 	// Match and prune MDD according to another MDD.
	// void setUpSubSolver(CBS& cbs) const;
	int minimumVertexCover(const vector<int>& CG); // mvc on disjoint components
	int minimumVertexCover(const vector<int>& CG, int old_mvc, int cols, int num_of_edges); // incremental mvc
	bool KVertexCover(const vector<int>& CG, int num_of_CGnodes, int num_of_CGedges, int k, int cols);
	int greedyMatching(const vector<bool>& CG, int cols);
    static int greedyMatching(const std::vector<int>& CG,  int cols);
    static int greedyWeightedMatching(const vector<int>& CG, int cols);
	int minimumWeightedVertexCover(const vector<int>& CG);
	// int minimumConstrainedWeightedVertexCover(const vector<int>& CG);
	int weightedVertexCover(const vector<int>& CG);
	int DPForWMVC(vector<int>& x, int i, int sum, const vector<int>& CG, const vector<int>& range, int& best_so_far); // dynamic programming
	// int ILPForWMVC(const vector<int>& CG, const vector<int>& range) const; // Integer linear programming
	int ILPForConstrainedWMVC(const std::vector<int>& CG, const std::vector<int>& range);
	int DPForConstrainedWMVC(vector<bool>& x, int i, int sum, const vector<int>& CG, const vector<int>& range, int& best_so_far);
};




