//
//  point.hpp
//  MAMCTS
//
//  Created by Nicholas Zerbel on 5/25/17.
//  Copyright © 2017 Nicholas Zerbel. All rights reserved.
//

#ifndef agent_hpp
#define agent_hpp

#include <cstdlib>
#include <iostream>
#include <vector>
#include <assert.h>
#include <fstream>

using namespace std;

class point{
public:
    int agent_x;
    int agent_y;

};

class agent : public point{
    friend class point;

public:
    point start;
    point goal;
    int dist;
    int final_timestep = 0;
    vector<point> path_agent;

    agent();
};

class multi_agent{
    friend class point;
public:
    vector <point> agent_start_pos;
    vector <point> agent_list;
    vector <point> goal_start_pos;
    vector <point> goal_list;

    vector <agent> agents;
    
    void create_config_list(int max_a, int xd, int yd, int nstat); //Create new goal and point coordinates
    void create_start_vecs(int stat, int n, int max_a);
    
    //Goal Functions
    void assign_goal_coordinates();
    void check_goal_coordinates(int n, double xc, double yc);

    //Agent Functions
    void assign_agent_coordinates();
    void check_agent_coordinates(int n);
    
    //Parameters
    int n_agents; //Number of agents and goals
    int xdim; //X dimension of Gridworld
    int ydim; //Y dimension of Gridworld
    bool unique_pos; //Detects if an point or a goal has been stacked on another point or goal
};

#endif /* agent_hpp */

