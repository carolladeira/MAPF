//
//  sim.hpp
//  MAMCTS
//
//  Created by Nicholas Zerbel on 5/25/17.
//  Copyright © 2017 Nicholas Zerbel. All rights reserved.
//

#ifndef sim_hpp
#define sim_hpp

#include <cstdlib>
#include <iostream>
#include <vector>
#include <assert.h>
#include <algorithm>
#include <time.h>
#include <fstream>
#include "agent.hpp"
#include "tree.hpp"
#include "mcts.hpp"

using namespace std;

class gridworld{
public:
    void initialize_parameters(multi_agent *map, monte_carlo *mcp);
    void cred_evals(multi_agent *map, multi_tree* tp, monte_carlo *mcp);
    void system_rollout(multi_agent *map, multi_tree* tp, monte_carlo *mcp);
    
    //Credit Evaluation
    vector <int> node_vec; //Keeps track of current nodes during credit evaluations
    vector <int> dif_node_vec; //Keeps Track of where each agent's rollout ends in cred eval
    vector <double> agent_rewards; //Tracks the individual rewards of agents in the system
    vector <bool> ag_in_play; //Tracks if an agent is still in play or not
    vector <int> end_lev;
    void reset_all_agents(multi_agent *map, multi_tree *tp);
    void calculate_local(multi_agent *map, monte_carlo *mcp, multi_tree *tp);
    void clear_all_vectors(multi_agent *map, monte_carlo *mcp, multi_tree *tp); //Clear all vectors for next stat run

    void agente_move(multi_tree *tp, monte_carlo *mcp, int agent);
    void all_agents_move(multi_tree *tp, monte_carlo *mcp);
    bool checkCollision(int agent, int act);



    //Paramaters
    int credit_type; //Used to determine which type of credit eval to run
    int x_dim; //Max x-dimension of gridworld
    int y_dim; //max y-dimension of gridworld
    int n_agents; //Number of agents and goals
    int max_lev;

    int timestep=0;
    std::vector<vector<point>> path_agents;
    vector<bool> at_goal;


    bool all_goals_captured;
    bool goal_check;
    bool agents_at_goals;
    
    //Rewards and Penalties
    double g_reward; //Global reward
    double sys_reward; //G_reward but used for data collection and not evals
    double goal_reward; //Reward for reaching a goal
    double step_penalty; //Reward for taking a step without reaching a goal
    double num;
};

#endif /* sim_hpp */
