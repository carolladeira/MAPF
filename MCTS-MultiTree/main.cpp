//
//  main.cpp
//  MAMCTS
//
//  Created by Nicholas Zerbel on 11/16/17.
//  Copyright © 2017 Nicholas Zerbel. All rights reserved.
//

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <assert.h>
#include "agent.hpp"
#include "sim.hpp"
#include "tree.hpp"
#include "sim.hpp"

using namespace std;


int main() {
    //srand( time(NULL) ); //Seed random generator
    srand(8);
    gridworld g; monte_carlo mcts; multi_tree t; multi_agent m; //Declare instance of classes
    gridworld *gp = &g; monte_carlo *mcp = &mcts; multi_tree *tp = &t; multi_agent *map = &m; //Pointers
    
    //Create Txt Files For Data Output
    ofstream D_sysr, G_sysr, L_sysr, D_succ, G_succ, L_succ;
    D_sysr.open("D_SysRewards.txt"); D_succ.open("D_Runs.txt");
    G_sysr.open("G_SysRewards.txt"); G_succ.open("G_Runs.txt");
    L_sysr.open("L_SysRewards.txt"); L_succ.open("L_Runs.txt");
    
    //Testing Parameters
    int stat_runs = 1; //Number of statistical runs : 30
    int max_run = 10000;//Cuts off the simulation if the number of iterations exceed this amount : 10000
    int agent_increment = 0; //Increases the number of agents in a simulation by this amount
    int starting_agents = 2; //Initial number of agents being tested
    int max_agents = 2; //Maximum number of agents to be tested
    g.x_dim = 8; //Maximum X Dimension
    g.y_dim = 8; //Maximum Y Dimension
    mcp->epsilon = 10; //UCB1 exploration constant (0 = greedy action selection)
    mcp->rollout_steps = 15;//Number of rollout moves //15
    mcp->rollout_iterations = 3;
    gp->max_lev = g.x_dim + g.y_dim;
    mcp->max_lev = g.x_dim + g.y_dim; //Cutoff point in tree where tree cannot expand any further
    int success_count; //Counts the number of successful runs (all goals captured)
    
    //Rewards and Penalties
    g.goal_reward = 100; //Reward for reaching an unclaimed goal
    mcts.rollout_reward = 0.5; //Reward received during MCTS rollout for discovering a goal
    g.step_penalty = -1; //Cost of each step taken by an agent in Gridworld

    map->create_config_list(max_agents, g.x_dim, g.y_dim, stat_runs);
    for(int c = 1; c < 2; c++){ //1 = local, 2 = global, 3 = difference
        cout << "Credit Eval: " << c << endl;
        g.credit_type = c;
        for(g.n_agents = starting_agents; g.n_agents <= max_agents;){
            for(int s = 0; s < stat_runs; s++){ //Run tests for specific number of stat runs
                m.create_start_vecs(s, g.n_agents, max_agents);
                g.initialize_parameters(map, mcp);
                mcts.create_root_nodes(tp, map);
                for(int its = 0; its < max_run; its++){
                    for(int anum = 0; anum < g.n_agents; anum++){ //anum = agent number
                        cout << " ============================= Agente "<< anum<<" ===========================" << endl;
                        mcts.set_mc_parameters(tp, anum);
                        mcts.mc_search(tp, map); //Runs MCTS for defined number of expansions
                        mcts.n_num_vec.at(anum) = mcts.node_number; //Used to track what the current node number is in each tree
                    }
                    g.cred_evals(map, tp, mcp);

                    //Check to see if agents have arrived at goals
                    g.system_rollout(map, tp, mcp);
                    g.reset_all_agents(map, tp);
                }
                for(int anum = 0; anum < g.n_agents; anum++){
                    cout<<endl<<"Agente: "<<anum<<" ---> ";
                   cout<<m.agent_start_pos.at(anum).agent_x<<","<<m.agent_start_pos.at(anum).agent_y<<" -> "<<m.goal_start_pos.at(anum).goal_x<<","<<m.goal_start_pos.at(anum).goal_y<<endl;
                    mcts.print_path(tp,anum);
                }
                mcts.check_goalAndColision(tp,0, map);
            }
        }
    }


    return 0;
}
