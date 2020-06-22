
#ifndef mcts_hpp
#define mcts_hpp

#include <cstdlib>
#include <iostream>
#include <vector>
#include <cmath>
#include <assert.h>
#include <algorithm>
#include "tree.hpp"
#include "agent.hpp"


using namespace std;

class monte_carlo{
public:
    monte_carlo(){}

    void set_mc_parameters(multi_tree *tp, std::vector<std::vector<bool>> my_map);
    void create_root_nodes(multi_tree *tp, vector<agent> agentes);

    void update_tree(multi_tree *tp, int level, int n);
    void set_agent_goal(int id_agente);

    void mc_search(multi_tree *tp);

    void print_path(multi_tree *tp,int a);
    void check_goalAndColision(multi_tree *tp,int a, multi_agent *map);

    //SELECTION
    void select(multi_tree *tp); //Choose an action
    int select_move(multi_tree *tp, int agn, int l); //Choose move which the point will take after MCTS simulation
    void reset_q_roll(multi_tree *tp);
    
    //EXPANSION
    void expand(multi_tree *tp);
    void move_left(multi_tree *tp);
    void move_right(multi_tree *tp);
    void move_up(multi_tree *tp);
    void move_down(multi_tree *tp);
    void no_move(multi_tree *tp);
    void update_node_numbers(multi_tree *tp);
    void pruning(multi_tree *tp);
    void check_boundaries(double xx, double yy); //Agent cannot move outside Gridworld
    void prune(multi_tree *tp, int l); //Agent cannot re-visit a state when expanding the tree
    void reset_coordinates();
    
    //SIMULATION
    int select_node(multi_tree *tp); //Select an expanded node for rollout
    void rollout(multi_tree *tp, int n);
    void calculate_node_value(multi_tree *tp, int n);
    
    //BACK-PROPAGATION
    void back_propagate(multi_tree *tp);
    void back_propagate_evals(multi_agent *map, multi_tree *tp, double reward, int agn, int l, int nn);
    
    //Parameters
    uint64_t no_expanded = 0;
    uint64_t no_generated = 0;

    vector <double> reward_vec;
    vector <int> n_num_vec;
    bool action_check; //Flags possible actions as valid or invalid
    int node_chosen;
    double parent_visit;
    double node_visit;
    int max_lev; //Maximum level tree can expand out to
    int lev; //Current level of the tree
    int p_lev; //Current parent level
    int n_nodes; //Number of nodes currently in a level in the tree
    int node_number;
    int parent_number;
    int current_node;
    int a_num; //Designates which point is currently simulating
    int n_agents; //Number of agents and goals
    int max_agents; //Number of agents and goals
    int action;

    int agente;
    
    //Experimental Parameters
    int rollout_steps; //Number of rollout iterations
    int rollout_iterations; //Number of rollout iterations
    double epsilon; //Exploration vs Exploitation parameter for UCB1
    double rollout_reward;
    double num;
    
    //Coordinates
    double ax; //Current point x coordinate
    double ay; //Current point y coordinate
    double previous_x; //Previous point x coordinate
    double previous_y; //Previous point y coordinate
    int x_lim; //Maximum x dimension
    int y_lim; //Maximum y dimension

    int node = 0;

    vector<point> pos_agentes;
    vector<point> start_agentes;
    vector<point> goal_agentes;

    vector<bool> at_goal;
    std::vector<vector<point>> list_agents;

    std::vector<std::vector<bool>> my_map;

};

#endif /* mcts_hpp */
