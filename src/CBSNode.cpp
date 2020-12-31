#include "CBSNode.h"


void HLNode::clear()
{
	conflicts.clear();
	unknownConf.clear();
	// conflictGraph.clear();
}

/*void HLNode::printConflictGraph(int num_of_agents) const
{
	if (conflictGraph.empty())
		return;
	cout << "	Build conflict graph in " << *this << ": ";
	for (auto e : conflictGraph)
	{
		if (e.second.first == 0)
			continue;
		int i = e.first / num_of_agents;
		int j = e.first % num_of_agents;
		std::cout << "(" << i << "," << j << ")=" << e.second.first << ",";
	}
	cout << endl;
}*/

void HLNode::updateDistanceToGo()
{
	set<pair<int, int>> conflicting_agents;
	for (const auto& conflict : unknownConf) // unknownConf store all conflicts
	{
		auto agents = make_pair(min(conflict->a1, conflict->a2), max(conflict->a1, conflict->a2));
		if (conflicting_agents.find(agents) == conflicting_agents.end())
		{
			conflicting_agents.insert(agents);
		}
	}
	distance_to_go  = (int)(conflicts.size() + conflicting_agents.size()); // while conflicts only store one conflict per pair of agents
}

void HLNode::printConstraints(int id) const
{
    auto curr = this;
    while (curr->parent != nullptr)
    {
        int a, x, y, t;
        constraint_type type;
        for (auto constraint : curr->constraints)
        {
            tie(a, x, y, t, type) = curr->constraints.front();
            switch (type)
            {
                case constraint_type::LEQLENGTH:
                case constraint_type::POSITIVE_VERTEX:
                case constraint_type::POSITIVE_EDGE:
                    cout << constraint << ",";
                    break;
                case constraint_type::GLENGTH:
                case constraint_type::VERTEX:
                case  constraint_type::EDGE:
                case constraint_type::BARRIER:
                case constraint_type::RANGE:
                    if (a == id)
                        cout << constraint << ",";
                    break;
            }
        }
        curr = curr->parent;
    }
}

std::ostream& operator<<(std::ostream& os, const HLNode& node)
{
	os << "Node " << node.time_generated << " from " << node.chosen_from << " ( f = "<< node.g_val << " + " <<
		node.h_val << ", f hat = " << node.getFHatVal() - node.cost_to_go << " + " << node.cost_to_go <<
		", d = " << node.distance_to_go << " ) with " <<
		node.getNumNewPaths() << " new paths ";
	return os;
}