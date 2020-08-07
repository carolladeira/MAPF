
#include <fstream>
#include <assert.h>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <cmath>
#include "sim.hpp"
#include "ScenarioLoader.h"
#include "agent.hpp"
#include "tree.hpp"
#include <time.h>

using namespace std;


int main() {
    //srand( time(NULL) ); //Seed random generator
    srand(8);
    monte_carlo mcts;
    multi_tree t;
    multi_agent m; //Declare instance of classes
    monte_carlo *mcp = &mcts;
    multi_tree *tp = &t;


    string scen = "empty-48-48";
    int i = 1;

    string a = "/home/carol/Desktop/Path Planning/mapf-scen-random/scen-random/"+scen+"-random-" +to_string(i)+".scen";
    string b = "/home/carol/Desktop/Path Planning/mapf-map/"+scen+".map";
    const char *scen_file = a.c_str();
    const char *map_file =  b.c_str();


    //Testing Parameters
    ///sempre numero mulyiplo fr 5
    int max_run = 25;//Cuts off the simulation if the number of iterations exceed this amount : 10000
    mcp->epsilon = 10; //UCB1 exploration constant (0 = greedy action selection)
    mcp->rollout_steps = 48;//Number of rollout moves //15
    mcp->rollout_iterations = 5;
    mcts.rollout_reward = 0.0; //Reward received during MCTS rollout for discovering a goal

    ScenarioLoader *scenLoad;
    scenLoad = new ScenarioLoader(scen_file);
    scenLoad->MapLoader(map_file);
    int num_experimentos = scenLoad->GetNumExperiments();
    gridworld *grid = new gridworld();
    grid->loadScen(*scenLoad, mcp);

    mcts.set_mc_parameters(tp, grid->my_map);

    grid->addTasks(*scenLoad,2);


//    for(int i = 2; i <num_experimentos; i++){
//    }
    mcp->at_goal.resize(grid->n_agents);
    mcts.create_root_nodes(tp, grid->agents);

    grid->print_map(0);

    clock_t start;// = clock();
    double duration, tempoTotalPathPlan = 0.0;
    int timestep =1;
    while (timestep < 500000000) {
        start = clock();
        for (int its = 0; its <= max_run; its++) {
            mcts.mc_search(tp);
        }
        duration = (clock() - start) / (double) CLOCKS_PER_SEC;
        tempoTotalPathPlan = duration + tempoTotalPathPlan;
        timestep++;
        grid->agente_move(tp, mcp);
        grid->print_path(mcp);

        int a=0;
        for(int i =0; i < mcp->at_goal.size(); i++){
            if(mcp->at_goal[i])a++;
        }
        if(a == mcp->at_goal.size()) break;

        //grid->print_map(0);
    }
    grid->info();
    cout<<endl<<"qtd: "<<grid->agents.size()<<" |map name: "<<grid->mapName<<" |scen name: "<<grid->scenName<<" |mkspn: "<<grid->mkspn<<" |soc: "<<grid->soc<<" |no gerados: "<<mcp->no_generated<<" |no expandido: "<<mcp->no_expanded<<" |TempoPathPlan: "<<tempoTotalPathPlan ;

    return 0;
}
