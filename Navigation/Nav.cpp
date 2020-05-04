//
// Created by carol on 11/17/18.
//
  /* time */

#include "Nav.h"

#include <iostream>

using namespace std;
//#define DEBUG


//-------------------------Agente------------------------------------
Agente::Agente() {

}
Nav::Nav(ScenarioLoader scen) {

   // n_agents = scen.GetNumExperiments();
   n_agents = 5;
    this->agentes.resize(n_agents);

    for(int i=0; i < n_agents; i++){
        Experiment exp = scen.GetNthExperiment(i);
        int start_x = exp.GetStartX();
        int start_y = exp.GetStartY();
        int goal_x = exp.GetGoalX();
        int goal_y = exp.GetGoalY();
        agentes[i].start.x = start_x;
        agentes[i].start.y = start_y;
        agentes[i].goal.x = goal_x;
        agentes[i].goal.y = goal_y;
    }
}


