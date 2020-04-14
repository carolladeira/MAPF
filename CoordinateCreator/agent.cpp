//
//  agent.cpp
//  MAMCTS
//
//  Created by Nick Zerbel on 5/25/17.
//  Copyright © 2017 Nick Zerbel. All rights reserved.
//

#include "agent.hpp"

//AGENT FUNCTIONS-----------------------------------------------------------------------------------------------
void multi_agent::create_agent_vec(int n, int xd, int yd){
    double x, y;
    agent a;
    xdim = xd; ydim = yd;
    n_agents = n;
    
    for(int i = 0; i < n_agents; i++){
        agent_vec.push_back(a);
        agent_vec.at(i).agent_x = -1;
        agent_vec.at(i).agent_y = -1;
    }
    
    
    //Create Coordinates
    for(int i = 0; i < n_agents; i++){
        x = (double)(rand() % xdim);
        y = (double)(rand() % ydim);
        check_agent_coordinates(i, x, y);
        while(unique_pos == false){
            x = (double)(rand() % xdim);
            y = (double)(rand() % ydim);
            check_agent_coordinates(i, x, y);
        }
        agent_vec.at(i).agent_x = x;
        agent_vec.at(i).agent_y = y;
    }
    
    //Check to make sure no agents are stacked
    for(int i = 0; i < n_agents; i++){
        x = agent_vec.at(i).agent_x;
        y = agent_vec.at(i).agent_y;
        check_agent_coordinates(i, x ,y);
        assert(unique_pos == true);
    }
}

void multi_agent::check_agent_coordinates(int n, double x, double y){
    unique_pos = true;
    for(int i = 0; i < n_agents; i++){ //Check agent coordinates against other agent coordinates
        if(i != n){
            if(x == agent_vec.at(i).agent_x && y == agent_vec.at(i).agent_y){
                unique_pos = false;
                break;
            }
        }
    }
}

//GOAL FUNCTIONS----------------------------------------------------------------------------------------------------
void multi_agent::create_goal_vec(){
    goal g;
    double x, y;
    
    for(int i = 0; i < n_agents; i++){
        goal_vec.push_back(g);
        goal_vec.at(i).goal_x = -1;
        goal_vec.at(i).goal_y = -1;
    }
    
    //Create Coordinates
    for(int i = 0; i < n_agents; i++){
        x = (double)(rand() % xdim);
        y = (double)(rand() % ydim);
        check_goal_coordinates(i, x, y);
        while(unique_pos == false){
            x = (double)(rand() % xdim);
            y = (double)(rand() % ydim);
            check_goal_coordinates(i, x ,y);
        }
        goal_vec.at(i).goal_x = x;
        goal_vec.at(i).goal_y = y;
    }
    
    //Check to make sure no goals are stacked
    for(int i = 0; i < n_agents; i++){
        x = goal_vec.at(i).goal_x;
        y = goal_vec.at(i).goal_y;
        check_goal_coordinates(i, x, y);
        assert(unique_pos == true);
    }
}

void multi_agent::check_goal_coordinates(int n, double xc, double yc){ //Goal number, goal_x, goal_y
    unique_pos = true;
    for(int i = 0; i < n_agents; i++){ //Check goal coordinates against other goal coordinates and agent coordinates
        if(i != n){
            if(xc == goal_vec.at(i).goal_x && yc == goal_vec.at(i).goal_y){
                unique_pos = false;
                break;
            }
        }
        if(xc == agent_vec.at(i).agent_x && yc == agent_vec.at(i).agent_y){ //Goal cannot start out at same position as agent
            unique_pos = false;
            break;
        }
    }
}

//Universal Function -----------------------------------------------------------------------------------------------------
void multi_agent::create_config_list(int max_a){
    ofstream ac_x, ac_y, gc_x, gc_y;
    ac_x.open("agent_x_coords.txt"); ac_y.open("agent_y_coords.txt");
    gc_x.open("goal_x_coords.txt"); gc_y.open("goal_y_coords.txt");
    
    for(int j = 0; j < n_configs; j++){ //Maximum number of configurations
        create_agent_vec(max_a, x_dim, y_dim);
        create_goal_vec();
        for(int k = 0; k < max_a; k++){ //Maximum number of agents
            ac_x << agent_vec.at(k).agent_x << "\n"; ac_y << agent_vec.at(k).agent_y << "\n";
            gc_x << goal_vec.at(k).goal_x << "\n"; gc_y << goal_vec.at(k).goal_y << "\n";
        }
        agent_vec.clear();
        goal_vec.clear();
    }
    ac_x.close(); ac_y.close(); gc_x.close(); gc_y.close();
}
