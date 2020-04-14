//
//  sim.cpp
//  MAMCTS
//
//  Created by Nick Zerbel on 5/25/17.
//  Copyright © 2017 Nick Zerbel. All rights reserved.
//

#include "sim.hpp"

//GRIDWORLD PROBLEM---------------------------------------------------------------------------------------------------------
void gridworld::initialize_parameters(multi_agent *map, monte_carlo *mcp) {
    for (int i = 0; i < n_agents; i++) {
        mcp->n_num_vec.push_back(0);
        node_vec.push_back(0);
        ag_in_play.push_back(true);
        agent_rewards.push_back(0);
        end_lev.push_back(0);
    }
    map->assign_agent_coordinates();
    map->assign_goal_coordinates();
    mcp->node_number = 0;
    mcp->x_lim = x_dim;
    mcp->y_lim = y_dim;

    path_agents.resize(map->n_agents);

    for (int i = 0; i < map->n_agents; i++) {
        path_agents[i].push_back(map->agent_start_pos[i]);
        int xstart = map->agent_start_pos[i].agent_x;
        int ystart = map->agent_start_pos[i].agent_y;
        int xgoal = map->goal_start_pos[i].agent_x;
        int ygoal = map->goal_start_pos[i].agent_y;
        double dist = std::abs(xstart- xgoal) + std::abs(ystart - ygoal);
        cout<<"Agente "<<i<<" "<<xstart<<","<<ystart<<" | "<<xgoal<<","<<ygoal<<" | "<<dist<<endl;

    }

}


void gridworld::reset_all_agents(multi_agent *map, multi_tree *tp) { //Resets all agents to initial positions
    for (int i = 0; i < n_agents; i++) { //Agent Number
      //  map->agent_vec.at(i).agent_x = tp->ag_tree.at(i).tree_vec.at(0).level_vec.at(0).x; //Initial X
     //   map->agent_vec.at(i).agent_y = tp->ag_tree.at(i).tree_vec.at(0).level_vec.at(0).y; //Initial Y
        node_vec.at(i) = 0;
        ag_in_play.at(i) = true;
    }
}


void gridworld::agente_move(multi_agent *map, multi_tree *tp, monte_carlo *mcp) {

    int a = 0;
    std::vector<point> agents;
    int parent_number = 0;
    point ag;

    mcp->parent_number = tp->ag_tree.at(0).tree_vec.at(0).level_vec.at(0).n_number;
    parent_number = mcp->parent_number;
    cout << " ==========================================" << endl;
    tp->print_tree(0);
    cout << " ==========================================" << endl;
    cout << " ************** MC SEARCH ****************" << endl;
    for (int lev = 1; lev < tp->ag_tree.at(a).tree_vec.size() - 1; lev++) {
        int act = mcp->select_move(tp, a, lev);

        for (int en = 0; en < tp->ag_tree.at(a).tree_vec.at(lev).level_vec.size(); en++) {
            if (act == tp->ag_tree.at(a).tree_vec.at(lev).level_vec.at(en).action &&
                parent_number == tp->ag_tree.at(a).tree_vec.at(lev).level_vec.at(en).p_number) {
                int nn = tp->ag_tree.at(a).tree_vec.at(lev).level_vec.at(en).n_number;
                int id_agente = tp->ag_tree.at(a).tree_vec.at(lev).level_vec.at(en).a_number;
                parent_number = nn;
                int ucb = tp->ag_tree.at(a).tree_vec.at(lev).level_vec.at(en).UCB1;
                int n_visit = tp->ag_tree.at(a).tree_vec.at(lev).level_vec.at(en).visit_count;
                int agx = tp->ag_tree.at(a).tree_vec.at(lev).level_vec.at(en).x;
                int agy = tp->ag_tree.at(a).tree_vec.at(lev).level_vec.at(en).y;
                int p_num = tp->ag_tree.at(a).tree_vec.at(lev).level_vec.at(en).p_number;
                ag.agent_x = agx;
                ag.agent_y = agy;
                if (!mcp->at_goal[id_agente]) {
                    path_agents[id_agente].push_back(ag);
                    if(agx == map->goal_start_pos[id_agente].agent_x && agy == map->goal_start_pos[id_agente].agent_y){
                        mcp->set_agent_goal(id_agente);
                        //cout<<"Agente"<<id_agente<<endl;
                        map->agents[id_agente].final_timestep = path_agents[id_agente].size() - 1;
                       // print_path();
                    }

                }

                //cout<<"Agente "<<id_agente<<" -> "<< agx << "," << agy<<endl;
                //std::cout << "Node " << nn << " pos " << lev << " UCB1 " << ucb << " N-visit " << n_visit << " | " << agx << "," << agy << " parent  " << p_num << endl;
                mcp->update_tree(tp, map, lev, en);
                return;
            }
        }
    }
}


void gridworld::print_path(monte_carlo *mcp) {

    for (int i = 0; i < path_agents.size(); i++) {
        cout << endl << "Agente " << i<<" ";
        for (int j = 0; j < path_agents[i].size(); j++) {
            cout << " -> ["<<j<<"]" << path_agents[i][j].agent_x << "," << path_agents[i][j].agent_y;
        }
    }


    for (int i = 0; i < path_agents.size(); i++) {
        if (!mcp->at_goal[i]){
            cout << endl << "Agente " << i<<" ";
            for (int j = 0; j < path_agents[i].size(); j++) {
                cout << " -> " << path_agents[i][j].agent_x << "," << path_agents[i][j].agent_y;
            }
        }

    }
}


void gridworld::clear_all_vectors(multi_agent *map, monte_carlo *mcp, multi_tree *tp) {
    tp->ag_tree.clear();
    mcp->reward_vec.clear();
   // map->agent_vec.clear();
   // map->goal_vec.clear();
    map->agent_start_pos.clear();
    map->goal_start_pos.clear();
    mcp->n_num_vec.clear();
    node_vec.clear();
    ag_in_play.clear();
    agent_rewards.clear();
    end_lev.clear();
}
