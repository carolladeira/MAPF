//
// Created by carol on 12/17/18.
//

#ifndef MAPD_SIMULATION_H
#define MAPD_SIMULATION_H

#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <cstring>
#include <cassert>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <climits>
#include "dlib/optimization/max_cost_assignment.h"
#include "ICBSSearch.h"
#include "egraph_reader.h"

#include "map_loader.h"
#include "ScenarioLoader.h"

using namespace std;

class Simulation {
public:
    int time;
    float computation_time;

    int row, col;
    int maxtime;
    int num_agents;
    int num_endpoint;

    vector<bool> my_map;
    int timestep;

    double tempoTotalGA;
    double tempoTotalPathPlan;

    vector<vector<int> > Dis;
    vector<Agent> agents;

    vector<Task*> list_taskset;

    int height;
    int width;
    int num_experimentos;

    Simulation();

    void runSearch(ScenarioLoader scen);
    void BFS();

    void run_CBS_TA(bool bestInd);

    bool PathFinding(vector<Agent*> &ags, const vector<vector<int> > &cons_paths);

    vector<int> CalcCost(Agent* ag);

    int getShortTime();

    int mkspn();
    double tempoGA() { return this->tempoTotalGA; }
    double tempoPathPlan()  { return this->tempoTotalPathPlan; }

    void deleteTask_id(int id);

    bool TestConstraints();


};


#endif //MAPD_SIMULATION_H
