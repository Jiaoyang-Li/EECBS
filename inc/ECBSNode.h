#pragma once
#include "CBSNode.h"


class ECBSNode : public HLNode
{
public:
	// the following is used to comapre nodes in the CLEANUP list
	struct compare_node_by_f
	{
		bool operator()(const ECBSNode* n1, const ECBSNode* n2) const
		{
			if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
			{
				if (n1->distance_to_go == n2->distance_to_go)
				{
					if (n1->sum_of_costs + n1->cost_to_go == n2->sum_of_costs + n2->cost_to_go)
					{
						return n1->h_val >= n2->h_val;
					}
					return n1->sum_of_costs + n1->cost_to_go >= n2->sum_of_costs + n2->cost_to_go;
				}
				return n1->distance_to_go >= n2->distance_to_go;
			}
			return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
		}
	};  // used by CLEANUP to compare nodes by f_val (top of the heap has min f_val)

	// the following is used to comapre nodes in the FOCAL list
	struct compare_node_by_d
	{
		bool operator()(const ECBSNode* n1, const ECBSNode* n2) const
		{
			if (n1->distance_to_go == n2->distance_to_go)
			{
				if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
				{
					if (n1->sum_of_costs + n1->cost_to_go == n2->sum_of_costs + n2->cost_to_go)
					{
						return n1->h_val >= n2->h_val;
					}
					return n1->sum_of_costs + n1->cost_to_go >= n2->sum_of_costs + n2->cost_to_go;
				}
				return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
			}
			return n1->distance_to_go >= n2->distance_to_go;
		}
	};  // used by FOCAL to compare nodes by distance_to_go (top of the heap has min distance_to_go)

		// the following is used to compare nodes in the OPEN list
	struct compare_node_by_inadmissible_f
	{
		bool operator()(const ECBSNode* n1, const ECBSNode* n2) const
		{
			if (n1->sum_of_costs + n1->cost_to_go == n2->sum_of_costs + n2->cost_to_go)
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
			return n1->sum_of_costs + n1->cost_to_go >= n2->sum_of_costs + n2->cost_to_go;
		}
	};  // used by FOCAL to compare nodes by f^-val (top of the heap has min f^-val)

	pairing_heap< ECBSNode*, compare<ECBSNode::compare_node_by_f> >::handle_type cleanup_handle;
	pairing_heap< ECBSNode*, compare<ECBSNode::compare_node_by_inadmissible_f> >::handle_type open_handle;
	pairing_heap< ECBSNode*, compare<ECBSNode::compare_node_by_d> >::handle_type focal_handle;

	int sum_of_costs = 0;  // sum of costs of the paths
	ECBSNode* parent;
	list< pair< int, pair<Path, int> > > paths; // new paths <agent id, <path, min f>>	
	inline int getFHatVal() const { return sum_of_costs + cost_to_go; }
	inline int getNumNewPaths() const { return (int) paths.size(); }
	inline string getName() const { return "ECBS Node"; }
	list<int> getReplannedAgents() const
	{
		list<int> rst;
		for (const auto& path : paths)
			rst.push_back(path.first);
		return rst;
	}
};