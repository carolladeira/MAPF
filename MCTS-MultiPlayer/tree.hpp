
#ifndef tree_hpp
#define tree_hpp

#include <cstdlib>
#include <vector>
#include <fstream>
#include <iostream>
#include <assert.h>
#include "agent.hpp"


using namespace std;

class node{
public:
    double x; //X-Coordinate
    double y; //Y-Coordinate
    int a_number; //Agent Number
    int n_number; //Node Number
    int p_number; //Parent Number
    double UCB1; //UCB1 value of the node
    double q_node; //MCTS action value of the node
    int action; //Designates which action is taken from the parent to access this state
    double visit_count; //Number of times a particular action has been taken from a parent node
    vector<point>pos_agentes;
};

class level{
    friend class node;
public:
    vector <node> level_vec;
    
};

class tree{
    friend class level;
public:
    vector <level> tree_vec;

};

class multi_tree{
    friend class tree;
public:
    vector <tree> ag_tree;
    void create_tree();
    void print_tree(int agn);
    void create_level(int agn);
    void create_node(int lev, int pos, double agx, double agy, int ag_num, int node_num, int p_num, int a, int agente);
    void create_root_node(int lev, int pos, double agx, double agy, int ag_num, int node_num, int p_num, int a, int agente, int n_agentes);

};

#endif /* tree_hpp */
