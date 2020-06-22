//
//  MAMCTS
//
//  Created by Nick Zerbel on 5/25/17.
//  Copyright © 2017 Nick Zerbel. All rights reserved.
//

#include "agent.hpp"


agent::agent() {


}
//Global Functions
void multi_agent::create_config_list(int max_a, int xd, int yd, int nstat){ //nstat = number of stat runs, max_a = max number of agents
    ifstream ac_x("/home/carol/Desktop/Path Planning/Instances/agent_x_coords.txt"); ifstream ac_y("/home/carol/Desktop/Path Planning/Instances/agent_y_coords.txt"); //These files contain point starting positions
    ifstream gc_x("/home/carol/Desktop/Path Planning/Instances/goal_x_coords.txt"); ifstream gc_y("/home/carol/Desktop/Path Planning/Instances/goal_y_coords.txt"); //These files contain goal positions
    point a;
    this->xdim = xd;
    this->ydim = yd;

    //Create a master-list of point and goal initial positions in gridworld. A different set for each statistical run
    for(int j = 0; j < nstat*max_a; j++){ //nstat*max_a = maximum number of configurations
        agent_list.push_back(a); //
        goal_list.push_back(a);

    }
    for(int j = 0; j < nstat*max_a; j++){ //nstat*max_a = maximum number of configurations
        ac_x >> agent_list.at(j).agent_x; //Get point x-coordinate from txt file
        ac_y >> agent_list.at(j).agent_y; //Get point y-coordinate from txt file
        gc_x >> goal_list.at(j).agent_x; //Get goal x-coordinate from txt file
        gc_y >> goal_list.at(j).agent_y; //Get goal y-coordinate from txt file
    }
    
    ac_x.close(); ac_y.close(); gc_x.close(); gc_y.close();
}

void multi_agent::create_start_vecs(int stat, int n, int max_a){ //max_a = maximum number of agents, stat = current stat run
    n_agents = n; //n is the current number of agents
    point a; agent ag;
    int c = 97;
    int ca = 65;
    for(int i = 0; i < n_agents; i++){
        agent_start_pos.push_back(a);
        goal_start_pos.push_back(a);
        agents.push_back(ag);
        agents.at(i).start = agent_list.at(max_a * stat + i);
        int xstart = agent_list.at(max_a*stat + i).agent_x;
        int ystart = agent_list.at(max_a*stat + i).agent_y;
        int xgoal = goal_list.at(max_a*stat + i).agent_x;
        int ygoal = goal_list.at(max_a*stat + i).agent_y;
        agents.at(i).goal.agent_y = ygoal;
        agents.at(i).goal.agent_x = xgoal;

        double dist = std::abs(xstart- xgoal) + std::abs(ystart - ygoal);
        agents.at(i).s = c;
        agents.at(i).g = ca;
        c++; ca++;
        agents.at(i).dist = dist;
        agents.at(i).path_agent.push_back(agents.at(i).start );
        agent_start_pos.at(i) = agent_list.at(max_a*stat + i); //Keeps track of point intial positions
        goal_start_pos.at(i) = goal_list.at(max_a*stat + i); //Keeps track of goal positions
    }
}

//AGENT FUNCTIONS-----------------------------------------------------------------------------------------------
void multi_agent::assign_agent_coordinates(){
    double x, y;
    //Check to make sure no agents are stacked
    for(int i = 0; i < n_agents; i++){
        x = agent_start_pos.at(i).agent_x;
        y = agent_start_pos.at(i).agent_y;
        check_agent_coordinates(i);
        assert(unique_pos == true);
    }
}

void multi_agent::check_agent_coordinates(int n){
    unique_pos = true; double x, y;
    x = agent_start_pos.at(n).agent_x;
    y = agent_start_pos.at(n).agent_y;
    for(int i = 0; i < n_agents; i++){ //Check point coordinates against other point coordinates
        if(i != n){
            if(x == agent_start_pos.at(i).agent_x && y == agent_start_pos.at(i).agent_y){
                unique_pos = false; //Checks to see if two agents exist in the same state
                break;
            }
        }
    }
}

//GOAL FUNCTIONS----------------------------------------------------------------------------------------------------
void multi_agent::assign_goal_coordinates(){
    double x, y;
    for(int i = 0; i < n_agents; i++){ //Goals cannot cohabit the same state in Gridworld
        x = goal_start_pos.at(i).agent_x;
        y = goal_start_pos.at(i).agent_y;
        check_goal_coordinates(i, x, y);
        assert(unique_pos == true);
    }
}

void multi_agent::check_goal_coordinates(int n, double xc, double yc){ //Goal number, goal_x, goal_y
    unique_pos = true;
    for(int i = 0; i < n_agents; i++){ //Check goal coordinates against other goal coordinates and point coordinates
        if(i != n){
            if(xc == goal_start_pos.at(i).agent_x && yc == goal_start_pos.at(i).agent_y){
                unique_pos = false; //Goals cannot cohabit the same state in Gridworld
                break;
            }
        }
        if(xc == agent_start_pos.at(i).agent_x && yc == agent_start_pos.at(i).agent_y){
            unique_pos = false; //Goal cannot start out at same position as point
            break;
        }
    }
}

void multi_agent::printMap() {
    char *mapChar = new char[xdim * ydim];
    int n=0;
    int c = 97;
    int ca = 65;
    for(int x=0; x < xdim; x++){
        for(int y =0; y < ydim; y++){
            mapChar[n] = '.';
            for(int i =0; i < agent_start_pos.size(); i++){
                if(agents[i].start.agent_y == y && agents[i].start.agent_x == x){
                    mapChar[n] = agents[i].s;
                }
                if(agents[i].goal.agent_y == y && agents[i].goal.agent_x == x){
                    mapChar[n] = agents[i].g;
                }
            }
            n++;
        }
    }
    std::cout << "MAP:";
    int t = 0;
    for (int i = 0; i < xdim; i++) {
        std::cout << std::endl;
        for (int j = 0; j < ydim; j++) {
            std::cout << mapChar[t];
            t++;
        }
    }
    std::cout << std::endl;
    delete[] mapChar;

}
