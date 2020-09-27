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

//   // n_agents = scen.GetNumExperiments();
//   n_agents = 1;
//    this->agentes.resize(n_agents);
//
//    for(int i=0; i < n_agents; i++){
//        Experiment exp = scen.GetNthExperiment(6);
//        int start_x = exp.GetStartX();
//        int start_y = exp.GetStartY();
//        int goal_x = exp.GetGoalX();
//        int goal_y = exp.GetGoalY();
//        agentes[i].start.x = start_x;
//        agentes[i].start.y = start_y;
//        agentes[i].goal.x = goal_x;
//        agentes[i].goal.y = goal_y;
//    }
}

void Nav::read_file(string arq, int width){
    string line;
    ifstream myfile(arq);
    //  cout << arq << endl;
    if (!myfile.is_open()) {
        cout << "Result file not found." << endl;
        system("PAUSE");
        return;
    }
//    getline(myfile, line);
//    int c;
//    sscanf(line.c_str(), "%d", &c);
    int y;
    int x;
    int x_g, y_g;
    int qtd = 10;
    this->agentes.resize(qtd);
    Point a,b;
    a.x = x;
    a.y = y;
    int t;
  //  agentes[0].path.push_back(a);
    int i=0;
    while (i<999){

      //  cout<<i<<" "<<x<<","<<y<<endl;
        for(int i =0; i < qtd ; i++){
            getline(myfile, line);
            sscanf(line.c_str(), "%d, %d,%d,%d,%d", &t, &x, &y, &x_g, &y_g);
            a.x = x;
            a.y = y;
            b.x = x_g;
            b.y = y_g;
            agentes[i].path.push_back(a);
            agentes[i].path_goal.push_back(b);
        }

        i++;


    }



}


