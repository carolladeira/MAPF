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
#include "mcts.hpp"
#include "ScenarioLoader.h"
#include "agent.hpp"
#include "tree.hpp"

using namespace std;

class gridworld{
public:
    gridworld() {}

    const char*scenName;
    string mapName;

    vector <agent> agents;
     int x_dim; //Max x-dimension of gridworld
     int y_dim; //max y-dimension of gridworld
    int n_agents; //Number of agents and goals
    int soc=0; int mkspn=0;
    std::vector<std::vector<bool>> my_map;

    //void initialize_parameters(multi_agent *map, monte_carlo *mcp);

    void agente_move(multi_tree *tp, monte_carlo *mcp);

    void loadScen(ScenarioLoader scen,  monte_carlo *mcp);
    void addTasks(ScenarioLoader scen, int qt);

    //Credit Evaluation
    vector <int> node_vec; //Keeps track of current nodes during credit evaluations
    vector <double> agent_rewards; //Tracks the individual rewards of agents in the system
    vector <bool> ag_in_play; //Tracks if an point is still in play or not
    vector <int> end_lev;
   // void reset_all_agents(multi_agent *map, multi_tree *tp);
    void clear_all_vectors(multi_agent *map, monte_carlo *mcp, multi_tree *tp); //Clear all vectors for next stat run

    void print_path(monte_carlo *mcp);
    void print_map(int t);

    std::vector<vector<point>> path_agents;
    void info();
};

#endif /* sim_hpp */
