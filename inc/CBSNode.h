#pragma once
#include "common.h"
#include "Conflict.h"

enum node_selection { NODE_RANDOM, NODE_H, NODE_DEPTH, NODE_CONFLICTS, NODE_CONFLICTPAIRS, NODE_MVC };


class HLNode // a virtual base class for high-level node
{
public:
	list<Constraint> constraints; // new constraints

	int g_val = 0; // sum of costs for CBS, and sum of min f for ECBS
	int h_val = 0; // admissible h
	int cost_to_go = 0; // informed but inadmissible h
	int distance_to_go = 0; // distance to the goal state
	size_t depth = 0; // depath of this CT node
	size_t makespan = 0; // makespan over all paths
	bool h_computed = false;

	uint64_t time_expanded = 0;
	uint64_t time_generated = 0;

	// For debug
	string chosen_from = "none"; // chosen from the open/focal/cleanup least
	int f_of_best_in_cleanup = 0;
	int f_hat_of_best_in_cleanup = 0;
	int d_of_best_in_cleanup = 0;
	int f_of_best_in_open = 0;
	int f_hat_of_best_in_open = 0;
	int d_of_best_in_open = 0;
	int f_of_best_in_focal = 0;
	int f_hat_of_best_in_focal = 0;
	int d_of_best_in_focal = 0;

	// conflicts in the current paths
	list<shared_ptr<Conflict> > conflicts;
	list<shared_ptr<Conflict> > unknownConf;

	// The chosen conflict
	shared_ptr<Conflict> conflict;
	// unordered_map<int, pair<int, int> > conflictGraph; //<edge index, <weight, num of CT nodes> >

	// online learning
	int distance_error = 0;
	int cost_error = 0;
	bool fully_expanded = false;

	HLNode* parent = nullptr;
	list<HLNode*> children;

	inline int getFVal() const { return g_val + h_val; }
	virtual inline int  getFHatVal() const = 0;
	virtual inline int getNumNewPaths() const = 0;
	virtual list<int> getReplannedAgents() const = 0;
	virtual inline string getName() const = 0;
	void clear();
	// void printConflictGraph(int num_of_agents) const;
	void updateDistanceToGo();
	void printConstraints(int id) const;

    virtual ~HLNode(){}
};

std::ostream& operator<<(std::ostream& os, const HLNode& node);


class CBSNode: public HLNode
{
public:
	// the following is used to comapre nodes in the CLEANUP list
	struct compare_node_by_f 
	{
		bool operator()(const CBSNode* n1, const CBSNode* n2) const 
		{
			if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
			{
				if (n1->distance_to_go == n2->distance_to_go)
				{
					if (n1->g_val + n1->cost_to_go == n2->g_val + n2->cost_to_go)
					{
						return n1->h_val >= n2->h_val;
					}
					return n1->g_val + n1->cost_to_go >= n2->g_val + n2->cost_to_go;
				}
				return n1->distance_to_go >= n2->distance_to_go;
			}
			return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
			/*if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
			{
				if (n1->h_val == n2->h_val)
				{
					if (n1->g_val + n1->cost_to_go == n2->g_val + n2->cost_to_go)
					{
						return n1->distance_to_go >= n2->distance_to_go;
					}
					return n1->g_val + n1->cost_to_go >= n2->g_val + n2->cost_to_go;	
				}
				return n1->h_val >= n2->h_val;
			}
			return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;*/
		}
	};  // used by CLEANUP to compare nodes by f_val (top of the heap has min f_val)

	// the following is used to comapre nodes in the FOCAL list
	struct compare_node_by_d 
	{
		bool operator()(const CBSNode* n1, const CBSNode* n2) const 
		{
			if (n1->distance_to_go == n2->distance_to_go)
			{
				if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
				{
					if (n1->g_val + n1->cost_to_go == n2->g_val + n2->cost_to_go)
					{
						return n1->h_val >= n2->h_val;
					}
					return n1->g_val + n1->cost_to_go >= n2->g_val + n2->cost_to_go;
				}
				return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
			}
			return n1->distance_to_go >= n2->distance_to_go;
			/*if (n1->distance_to_go == n2->distance_to_go)
			{
				if (n1->g_val + n1->cost_to_go == n2->g_val + n2->cost_to_go)
				{
					if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
					{
						return n1->h_val >= n2->h_val;
					}
					return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
				}
				return n1->g_val + n1->cost_to_go >= n2->g_val + n2->cost_to_go;
			}
			return n1->distance_to_go >= n2->distance_to_go;*/
		}
	};  // used by FOCAL to compare nodes by distance_to_go (top of the heap has min distance_to_go)

	// the following is used to compare nodes in the OPEN list
	struct compare_node_by_inadmissible_f
	{
		bool operator()(const CBSNode* n1, const CBSNode* n2) const
		{
			if (n1->g_val + n1->cost_to_go == n2->g_val + n2->cost_to_go)
			{
				if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
				{
					if (n1->distance_to_go == n2->distance_to_go)
					{
						return n1->h_val >= n2->h_val;
					}
					return n1->distance_to_go >= n2->distance_to_go;
				}
				return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
			}
			return n1->g_val + n1->cost_to_go >= n2->g_val + n2->cost_to_go;
		}
	};  // used by FOCAL to compare nodes by num_of_collisions (top of the heap has min h-val)

	pairing_heap< CBSNode*, compare<CBSNode::compare_node_by_f> >::handle_type cleanup_handle;
	pairing_heap< CBSNode*, compare<CBSNode::compare_node_by_inadmissible_f> >::handle_type open_handle;
	pairing_heap< CBSNode*, compare<CBSNode::compare_node_by_d> >::handle_type focal_handle;

	CBSNode* parent = nullptr;
	list< pair< int, Path> > paths; // new paths
	inline int getFHatVal() const override { return g_val + cost_to_go; }
	inline int getNumNewPaths() const override { return (int) paths.size(); }
	inline string getName() const override { return "CBS Node"; }
	list<int> getReplannedAgents() const override
	{
		list<int> rst;
		for (const auto& path : paths)
			rst.push_back(path.first);
		return rst;
	}
};


struct ConstraintsHasher // Hash a CT node by constraints on one agent
{
	int a{};
	const HLNode* n{};
	ConstraintsHasher(int a, HLNode* n) : a(a), n(n) {};

	struct EqNode
	{
		bool operator() (const ConstraintsHasher& c1, const ConstraintsHasher& c2) const
		{
			if(c1.a != c2.a)
				return false;
				
			std::set<Constraint> cons1, cons2;
			auto curr = c1.n;
			while (curr->parent != nullptr)
			{
				if (get<4>(curr->constraints.front()) == constraint_type::LEQLENGTH ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_VERTEX ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_EDGE ||
					get<0>(curr->constraints.front()) == c1.a) {
					for (auto con : curr->constraints)
						cons1.insert(con);
				}
				curr = curr->parent;
			}
			curr = c2.n;
			while (curr->parent != nullptr)
			{
				if (get<4>(curr->constraints.front()) == constraint_type::LEQLENGTH ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_VERTEX ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_EDGE ||
					get<0>(curr->constraints.front()) == c2.a) {
					for (auto con : curr->constraints)
						cons2.insert(con);
				}

				curr = curr->parent;
			}

			return equal(cons1.begin(), cons1.end(), cons2.begin(), cons2.end());
		}
	};

	struct Hasher
	{
		std::size_t operator()(const ConstraintsHasher& entry) const
		{
			auto curr = entry.n;
			size_t cons_hash = 0;
			while (curr->parent != nullptr)
			{
				if (get<4>(curr->constraints.front()) == constraint_type::LEQLENGTH ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_VERTEX ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_EDGE ||
					get<0>(curr->constraints.front()) == entry.a)
				{
					for (auto con : curr->constraints)
					{
						cons_hash += 3 * std::hash<int>()(std::get<0>(con)) +
							5 * std::hash<int>()(std::get<1>(con)) +
							7 * std::hash<int>()(std::get<2>(con)) +
							11 * std::hash<int>()(std::get<3>(con));
					}
				}
				curr = curr->parent;
			}
			return cons_hash;
		}
	};
};
