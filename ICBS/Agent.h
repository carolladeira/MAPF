//
// Created by carol on 12/17/18.
//

#ifndef MAPD_AGENT_H
#define MAPD_AGENT_H

#include <iostream>
#include <vector>
#include <list>
#include <string>
#include <functional>  // for std::hash (c++11 and above)
#include <map>
#include <algorithm>
#include <queue>
#include <limits.h>
#include <iostream>

//#include "Node.h"
#include "Endpoint.h"
class Task;
class Token;


using namespace std;


class Agent {
public:
    int loc, park_loc, goal_loc;
    int id;
    int row, col;

    int finish_time;
    vector<int> path;
    vector<int> cost;
    bool only_dummy;
    int start_time;
    vector<int> non_dummy_path;
    int debug;
    Endpoint *ep_park_loc;
    Endpoint* next_ep;
    bool delivering;
    Task *task;

    double timepathPlan;


    int AStar_sCol(int start_loc, Endpoint *goal, int begin_time, Token token, bool evaluate, bool coleta);

    void setAgent(int loc, int col, int row, int id);
    bool TP( Token &token);
    Task* bestTask(Token token);

    int AStar(int start_loc, Endpoint *goal, int begin_time, Token token, bool evaluate, bool coleta);
    bool isConstrained(int curr_id, int next_id, int next_timestep, Token token, int ag_hide);
    void updatePath(Node *goal);
    void releaseClosedListNodes(map<unsigned int, Node*> &allNodes_table);
    bool Move2EP(Token &token, bool constraint);
    int EvaluateTP(Token &token, int finish_time);
    int fastEvaluateTP(Token &token);
    int planPath(int start_loc, Endpoint *goal, int begin_time, Token &token);


};
class Task{
public:

    int id;
  //  int state; //0: livre | 1:com agente;
    Agent *agent;

    int seq_id; // TSP sequences id

    int agent_id;
    int release_time;//tempo que a tarefa chegou no sistema;
    int start_time;
    int goal_time;

    int ag_arrive_start;
    int ag_arrive_goal;

    Endpoint *start;
    Endpoint *goal;

    bool delivering;


    Task() {};
    Task(int id, int t, Endpoint *s, Endpoint *g, int s_t, int g_t):id(id), release_time(t), start(s), goal(g), start_time(s_t), goal_time(g_t), ag_arrive_goal(0), ag_arrive_start(0), delivering(false) {}
};

class Token{
public:
    int timestep;

    vector<bool> map;
    vector<bool> endpoints;
    vector<Agent*> agents;

    vector<vector<int>> path; //path[agent][time] = loc (id)

    list<Task*> taskset;

    Token(){timestep = 0;};


};

#endif //MAPD_AGENT_H
