#include "LLNode.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <limits>


LLNode::LLNode() : loc(0), g_val(0), h_val(0), parent(NULL), timestep(0), num_internal_conf(0), in_openlist(false) {
	vis_goal = false;
	goal_length = 0;
}

LLNode::LLNode(int loc, int g_val, int h_val, LLNode* parent, int timestep, int num_internal_conf, bool in_openlist) :
	loc(loc), g_val(g_val), h_val(h_val), parent(parent), timestep(timestep),
	num_internal_conf(num_internal_conf), in_openlist(in_openlist) {
	vis_goal = parent == NULL ? false : parent->vis_goal;
	goal_length = parent == NULL ? 0 : parent->goal_length;
}

LLNode::LLNode(const LLNode& other) {
	loc = other.loc;
	g_val = other.g_val;
	h_val = other.h_val;
	parent = other.parent;
	timestep = other.timestep;
	in_openlist = other.in_openlist;
	open_handle = other.open_handle;
	focal_handle = other.focal_handle;
	num_internal_conf = other.num_internal_conf;
	vis_goal = other.vis_goal;
	goal_length = other.goal_length;
}

LLNode::~LLNode() {
}

std::ostream& operator<<(std::ostream& os, const LLNode& n) {
	if (n.parent != NULL)
		os << "LOC=" << n.loc << " ; TIMESTEP=" << n.timestep << " ; GVAL=" << n.g_val << " ; HVAL=" << std::setprecision(4) << n.h_val
		<< " ; #CONF=" << n.num_internal_conf << " ; PARENT=" << (n.parent)->loc
		<< " ; IN_OPEN?" << std::boolalpha << n.in_openlist;
	else
		os << "LOC=" << n.loc << " ; TIMESTEP=" << n.timestep << " ; GVAL=" << n.g_val << " ; HVAL=" << std::setprecision(4) << n.h_val
		<< " ; #CONF=" << n.num_internal_conf << " ; ROOT (NO PARENT)";
	return os;
}

