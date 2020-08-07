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
    gridworld g;
    monte_carlo mcts;
    multi_tree t;
    multi_agent m; //Declare instance of classes
    gridworld *gp = &g;
    monte_carlo *mcp = &mcts;
    multi_tree *tp = &t;
    multi_agent *map = &m; //Pointers

    string scen = "empty-48-48";
    int i = 1;

    string a = "/home/carol/Desktop/Path Planning/mapf-scen-random/scen-random/" + scen + "-random-" + to_string(i) +
               ".scen";
    string b = "/home/carol/Desktop/Path Planning/mapf-map/" + scen + ".map";
    const char *scen_file = a.c_str();
    const char *map_file = b.c_str();

    //Testing Parameters
    int stat_runs = 10; //Number of statistical runs : 30
    int max_run = 15;//Cuts off the simulation if the number of iterations exceed this amount : 10000
    int max_agents = 7; //Maximum number of agents to be tested
    g.x_dim = 8; //Maximum X Dimension
    g.y_dim = 8; //Maximum Y Dimension

    ///UCB
    mcp->epsilon = 10; //UCB1 exploration constant (0 = greedy action selection)

    ///Rollout
    mcp->rollout_steps = 15;//Number of rollout moves //15
    mcp->rollout_iterations = 3;
    mcts.rollout_reward = 0.3; //Reward received during MCTS rollout for discovering a goal
    mcts.rollout_c_goal = 5;

    gp->max_lev = g.x_dim + g.y_dim;
    mcp->max_lev = g.x_dim + g.y_dim; //Cutoff agent in tree where tree cannot expand any further

    //Rewards and Penalties
    g.goal_reward = 100; //Reward for reaching an unclaimed goal
    g.step_penalty = -1; //Cost of each step taken by an agent in Gridworld
    g.n_agents = max_agents;
    g.at_goal.resize(max_agents);

    map->create_config_list(max_agents, g.x_dim, g.y_dim, stat_runs);
    int timestep = 1;
    m.create_start_vecs(0, max_agents, max_agents);
    g.initialize_parameters(map, mcp);
    mcts.create_root_nodes(tp, map);
    for (int anum = 0; anum < g.n_agents; anum++) {
        cout  << "Agente: " << anum << " ---> ";
        cout << m.agent_start_pos.at(anum).agent_x << "," << m.agent_start_pos.at(anum).agent_y << " -> "
             << m.goal_start_pos.at(anum).agent_x << "," << m.goal_start_pos.at(anum).agent_y << endl;
    }
    while (timestep < 2) {
        g.credit_type = 1;
        for (int s = 0; s < stat_runs; s++) {
            cout << " ============================= " << s << " ===========================" << endl;
            for (int its = 0; its < max_run; its++) {
                for (int anum = 0; anum < max_agents; anum++) { //anum = agent number
                    if(g.at_goal[anum])continue;
                    mcts.set_mc_parameters(tp, anum);
                    mcts.mc_search(tp, map); //Runs MCTS for defined number of expansions
                    mcts.n_num_vec.at(
                            anum) = mcts.node_number; //Used to track what the current node number is in each tree
                }
            }
            // g.cred_evals(map, tp, mcp);
            g.all_agents_move(tp, mcp);
            //Check to see if agents have arrived at goals
            // g.system_rollout(map, tp, mcp);
            // g.agente_move(tp, mcp, 0);

            // g.reset_all_agents(map, tp);
            for (int anum = 0; anum < g.n_agents; anum++) {
                int x = g.path_agents[anum][g.timestep].agent_x;
                int y = g.path_agents[anum][g.timestep].agent_y;
                if(m.goal_start_pos.at(anum).agent_x == x && m.goal_start_pos.at(anum).agent_y == y){
                    cout  << "Agente: " << anum << " chegou no destino "<<endl;
                    g.at_goal[anum] = true;
                }

               // mcts.print_path(tp, anum);
            }
            // mcts.check_goalAndColision(tp, 0, map);
        }
    }


    return 0;
}
