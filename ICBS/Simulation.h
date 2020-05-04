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

    int row, col;
    int maxtime;

    vector<bool> my_map;
    int timestep;

    double tempoTotalGA;
    double tempoTotalPathPlan;

    vector<vector<int> > Dis;
    vector<Agent> agents;

    uint64_t no_expanded;
    uint64_t no_generated;
    int height;
    int width;
    int num_experimentos;

    const char*scenName;
    string mapName;



    Simulation(ScenarioLoader scen);

    void runSearch(ScenarioLoader scen);
    void addTasks(ScenarioLoader scen, int qt);

    void BFS();

    bool PathFinding(vector<Agent*> &ags, const vector<vector<int> > &cons_paths);

    vector<int> CalcCost(Agent* ag);

    double tempoPathPlan()  { return this->tempoTotalPathPlan; }

    void printMap(ScenarioLoader scen);
    void printPath();
    void info();


    bool TestConstraints();


};


#endif //MAPD_SIMULATION_H
