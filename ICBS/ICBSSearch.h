#pragma once

#include "ICBSNode.h"
#include "SingleAgentICBS.h"
#include "compute_heuristic.h"
#include "Agent.h"

class ICBSSearch
{
public:
	constraint_strategy cons_strategy;
	double runtime = 0;
	double runtime_lowlevel;
	double runtime_conflictdetection;
	double runtime_computeh;
	double runtime_listoperation;
	double runtime_updatepaths;
	double runtime_updatecons;
	list<tuple<int, int, int, int, int, int, int>> node_stat;
	//double upper_bound;
	typedef boost::heap::fibonacci_heap< ICBSNode*, boost::heap::compare<ICBSNode::compare_node> > heap_open_t;
	typedef boost::heap::fibonacci_heap< ICBSNode*, boost::heap::compare<ICBSNode::secondary_compare_node> > heap_focal_t;
	typedef boost::heap::fibonacci_heap< MDDNode*, boost::heap::compare<MDDNode::compare_node> > mdd_open_t;
	//typedef dense_hash_map<ICBSNode*, ICBSNode*, ICBSNode::ICBSNodeHasher, ICBSNode::ecbs_eqnode> hashtable_t;

	heap_open_t open_list;
	heap_focal_t focal_list;
	//hashtable_t allNodes_table;
	list<ICBSNode*> allNodes_table;

	bool solution_found;
    int solution;

	vector<bool> goal_optimal;
	int solution_cost;

	double focal_w = 1.0;
	double min_f_val;
	double focal_list_threshold;

	const bool* my_map;
	int map_size;
	int num_of_agents;
	const int* actions_offset;
	const int* moves_offset;
	int num_col;
	vector<bool> only_dummy;
	vector<vector<int> > costs;
	//AgentsLoader al;

	

	uint64_t HL_num_expanded = 0;
	uint64_t HL_num_generated = 0;
	uint64_t LL_num_expanded = 0;
	uint64_t LL_num_generated = 0;

	ICBSNode* dummy_start;

	vector<vector<PathEntry>*> paths;
	vector<int> goal_length;
	vector<vector<PathEntry>> paths_found_initially;  // contain initial paths found
	vector<int> goal_length_initially;

	vector < SingleAgentICBS* > search_engines;  // used to find (single) agents' paths and mdd
	int runICBSSearch();
	bool findPathForSingleAgent(ICBSNode*  node, int ag, double lowerbound = 0);
	bool generateChild(ICBSNode* child, ICBSNode* curr);

	inline void updatePaths(ICBSNode* curr);

	vector < list< tuple<int, int, bool> > >* collectConstraints(ICBSNode* curr, int agent_id);
	void findConflicts(ICBSNode& curr);
	std::shared_ptr<tuple<int, int, int, int, int>> chooseConflict(ICBSNode &parent);
	std::shared_ptr<tuple<int, int, int, int, int>> classifyConflicts(ICBSNode &parent);
	tuple<int, int, int> countRectangleConflicts(ICBSNode &root);
	int identifyRectangleConflict(int s1_x, int s1_y, int s2_x, int s2_y, int g1_x, int g1_y, int g2_x, int g2_y, int t1, int t2);
	int computeHeuristics(const ICBSNode& curr);
	bool KVertexCover(const vector<vector<bool>>& CG, int num_of_CGnodes, int num_of_CGedges, int k);
	//inline bool updateICBSNode(ICBSNode* leaf_node, ICBSNode* root_node);
	//inline void updatePaths(ICBSNode* curr, ICBSNode* root_node);
	inline bool isManhattanOptimal(int loc1, int loc2, int dt);
	inline bool EqualRectConf(const tuple<int, int, int, int, int>& conf1, const tuple<int, int, int, int, int>& conf2);
	//void generateChildwithCurrentCost(ICBSNode* n1, const ICBSNode* curr);
	void buildMDD(ICBSNode& curr, int id, int lookahead); 
	inline int compute_g_val();

	inline int getAgentLocation(const ICBSNode & node, int agent_id, size_t timestep);

	void updateFocalList(double old_lower_bound, double new_lower_bound, double f_weight);
	void updateReservationTable(bool* res_table, int exclude_agent, const ICBSNode &node);
	inline void releaseClosedListNodes();
	inline void releaseOpenListNodes();
	void printPaths() const;
	void printConflicts(const ICBSNode &n) const;
	void printConstraints(const ICBSNode* n) const;
	ICBSSearch(const vector<bool> &my_map, vector<Agent*> &agents, double f_w, const EgraphReader& egr, constraint_strategy c, 
	int cols, const vector<vector<int> > &cons_paths, int curr_time);
	~ICBSSearch();
};

