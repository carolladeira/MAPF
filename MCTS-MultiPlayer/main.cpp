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
#include <time.h>

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

    //Testing Parameters
    int stat_runs = 30; //Number of statistical runs : 30
    int max_run = 20;//Cuts off the simulation if the number of iterations exceed this amount : 10000
    int agent_increment = 0; //Increases the number of agents in a simulation by this amount
    int starting_agents = 3; //Initial number of agents being tested
    int max_agents = 3; //Maximum number of agents to be tested
    g.x_dim = 8; //Maximum X Dimension
    g.y_dim = 8; //Maximum Y Dimension
    mcp->epsilon = 10; //UCB1 exploration constant (0 = greedy action selection)
    mcp->rollout_steps = 5;//Number of rollout moves //15
    gp->max_lev = g.x_dim + g.y_dim;
    mcp->max_lev = g.x_dim + g.y_dim; //Cutoff point in tree where tree cannot expand any further
    int success_count; //Counts the number of successful runs (all goals captured)
    //Rewards and Penalties
    g.goal_reward = 100; //Reward for reaching an unclaimed goal
    mcts.rollout_reward = 0.0; //Reward received during MCTS rollout for discovering a goal
    g.step_penalty = -1; //Cost of each step taken by an point in Gridworld
    map->create_config_list(max_agents, g.x_dim, g.y_dim, stat_runs);
    m.create_start_vecs(0, starting_agents, max_agents);
    g.initialize_parameters(map, mcp);
    mcp->at_goal.resize(map->n_agents);
    mcts.create_root_nodes(tp, map);
    m.printMap();
    clock_t c_start_geral = clock();
    double tempo;
    double ti_;
    while (ti_ < 7200) {
        //cout<<" "<<i<<endl;
        clock_t c_start = clock();
        for (int its = 0; its < max_run; its++) {
            mcts.set_mc_parameters(tp, 0);
            mcts.mc_search(tp, map); //Runs MCTS for defined number of expansions
        }
        g.agente_move(map, tp, mcp);
        g.print_path(mcp);
        int count =0;
        for(int i =0; i < map->n_agents; i++){
            if(mcts.at_goal[i]){
                count++;
            }
        }
        if(count == 16){
            cout<<"16";

        }
        clock_t c_end = clock();
        double tim = (c_end - c_start);
        double time = (tim / (CLOCKS_PER_SEC));
        double ti = (c_end - c_start_geral);
         ti_ =  (ti / (CLOCKS_PER_SEC));
        //std::cout << "CPU time used: " << time << endl;
        if(count == map->n_agents){
            clock_t c_end_geral = clock();
            double t = (c_end_geral - c_start_geral) ;

            tempo = (t/ (CLOCKS_PER_SEC));
            cout<<"Todos os agentes chegaram na posicao de destino";
            break;
        }
    }
    cout<<endl;
    int count_ag1 = 0;
    int soc =0;int mkspn = MAXFLOAT; int nodes;
    int soc_i =0;
    nodes = mcts.node_number;
    int count =0;
    for(int i =0; i < map->n_agents; i++){
        if(mcts.at_goal[i]){
            count++;
        }
    }
    for(int i =0; i < map->n_agents; i++){
        soc+=map->agents[i].final_timestep;
        soc_i+=map->agents[i].dist;

        if(map->agents[i].final_timestep < mkspn){
            mkspn = map->agents[i].final_timestep;
        }

        cout << "Agente " << i << " Min dist: " << map->agents[i].dist << " MCTS dist: " << map->agents[i].final_timestep << endl;
    }


    cout <<"At goal: "<<count<<" - SOC ideal: "<<soc_i<< " - SOC " << soc << " - No gerados: " << nodes << " - mkspn: " << mkspn << " - tempo: "<< tempo << endl;

    //g.print_path();

    return 0;
}
