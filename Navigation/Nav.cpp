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
   n_agents = 1;
    this->agentes.resize(n_agents);

    for(int i=0; i < n_agents; i++){
        Experiment exp = scen.GetNthExperiment(6);
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

void Nav::read_file(string arq, int width){
    string line;
    ifstream myfile(arq);
    //  cout << arq << endl;
    if (!myfile.is_open()) {
        cout << "Map file not found." << endl;
        system("PAUSE");
        return;
    }
    getline(myfile, line);
    int c;
    sscanf(line.c_str(), "%d", &c);
    int y = c%width;
    int x = c/width;
    Point a;
    a.x = x;
    a.y = y;
    agentes[0].path.push_back(a);
    int i=0;
    while(i < 5000){
        i++;
       // cout<<"   "<<i<<"    ";
        getline(myfile, line);
      //  cout<<line<<endl;
        sscanf(line.c_str(), "%d", &c);
         y = c%width;
         x = c/width;
        a.x = x;
        a.y = y;
        agentes[0].path.push_back(a);
    }
}


