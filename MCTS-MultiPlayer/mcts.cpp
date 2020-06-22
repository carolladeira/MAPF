//
//  mcts.cpp
//  MAMCTS
//
//  Created by Nick Zerbel on 5/25/17.
//  Copyright © 2017 Nick Zerbel. All rights reserved.
//

#include "mcts.hpp"
#include <numeric>

#define EXPAND
#define ROLLOUT
#define ROLLOUT2
#define SELECT
#define MCSEARCH
#define BACK_PROPAGATE


void monte_carlo::set_mc_parameters(multi_tree *tp, std::vector<std::vector<bool>> my_map) {
    a_num = 0;
    lev = 0;
    this->my_map = my_map;
}

void monte_carlo::set_agent_goal(int id_agente) {
    at_goal[id_agente] = true;
}

void monte_carlo::print_path(multi_tree *tp, int a) {
    tp->print_tree(a);
    // cout<<"============== Print Path =================="<<endl;
    //cout<<"**** Agente: "<<a<<" ****"<<endl;
    std::vector<point> agents;
    int score = 0, id;
    bool teste = false;
    int p_number = 0;
    point ag;
    int agx = tp->ag_tree.at(a).tree_vec.at(0).level_vec.at(0).x;
    int agy = tp->ag_tree.at(a).tree_vec.at(0).level_vec.at(0).y;
    ag.agent_y = agy;
    ag.agent_x = agx;
    agents.push_back(ag);

    for (int lev = 1; lev < tp->ag_tree.at(a).tree_vec.size() - 1; lev++) {
        int act = select_move(tp, a, lev);

        for (int en = 0; en < tp->ag_tree.at(a).tree_vec.at(lev).level_vec.size(); en++) {
            if (act == tp->ag_tree.at(a).tree_vec.at(lev).level_vec.at(en).action &&
                p_number == tp->ag_tree.at(a).tree_vec.at(lev).level_vec.at(en).p_number) {
                int nn = tp->ag_tree.at(a).tree_vec.at(lev).level_vec.at(en).n_number;
                p_number = nn;
                int ucb = tp->ag_tree.at(a).tree_vec.at(lev).level_vec.at(en).UCB1;
                int n_visit = tp->ag_tree.at(a).tree_vec.at(lev).level_vec.at(en).visit_count;
                int agx = tp->ag_tree.at(a).tree_vec.at(lev).level_vec.at(en).x;
                int agy = tp->ag_tree.at(a).tree_vec.at(lev).level_vec.at(en).y;
                int p_num = tp->ag_tree.at(a).tree_vec.at(lev).level_vec.at(en).p_number;
                ag.agent_x = agx;
                ag.agent_y = agy;
                agents.push_back(ag);
                // cout<<lev<<":"<<"("<<agx<<","<<agy<<")"<<" ";
                std::cout << "Node " << nn << " pos " << lev << " UCB1 " << ucb << " N-visit " << n_visit << " q-value "
                          << score << " | " << agx << "," << agy << " parent  " << p_num << endl;
                break;
            }
        }
    }
    list_agents.push_back(agents);
}

void monte_carlo::check_goalAndColision(multi_tree *tp, int a, multi_agent *map) {

    cout << endl << "============================================" << endl;
    vector<bool> goal_found;
    goal_found.resize(list_agents.size());
    bool goal = false;
    for (int i = 0; i < list_agents.size(); i++) {
        for (int j = 0; j < list_agents[i].size(); j++) {
            int x = list_agents[i][j].agent_x;
            if (x == -1)break;
            int y = list_agents[i][j].agent_y;
            if (x == map->goal_start_pos.at(i).agent_x && y == map->goal_start_pos.at(i).agent_y) {
                goal_found[i] = true;
                for (int t = j + 1; t < list_agents[i].size(); t++) {
                    list_agents[i][t].agent_x = -1;
                    list_agents[i][t].agent_y = -1;
                }

            }
        }
    }
    for (int i = 0; i < goal_found.size(); i++) {
        if (!goal_found[i]) {
            cout << " Agente " << i << " nao chegou no destino" << endl;
        }
    }
    for (int i = 0; i < list_agents.size(); i++) {
        cout << endl << i << ":";
        for (int j = 0; j < list_agents[i].size(); j++) {
            cout << " -> " << list_agents[i][j].agent_x << "," << list_agents[i][j].agent_y;
        }
    }
    cout << endl;

    int y_B, x_B;
    for (int i = 0; i < list_agents.size(); i++) {
        for (int j = 0; j < list_agents[i].size(); j++) {

            for (int j = 0; j < list_agents[i + 1].size(); j++) {
                int x_A = list_agents[i][j].agent_x;
                int y_A = list_agents[i][j].agent_y;
                if (i + 1 < list_agents.size()) {
                    x_B = list_agents[i + 1][j].agent_x;
                    y_B = list_agents[i + 1][j].agent_y;
                }
                if (x_A == x_B && y_A == y_B) { // colisao de vertex
                    //cout<<"Colisao -  Agente: "<<i<<" e "<< i+1<<" no tempo "<<j<<" posicao "<<x_A<<","<<y_A<<endl;
                    break;
                }
            }
        }
    }


}

int monte_carlo::select_move(multi_tree *tp, int agn, int l) { //(point number, level)
    int count;
    double best; //Tracks best Q-value for action selection
    action_check = false;
    assert(tp->ag_tree.at(agn).tree_vec.size() > l);

    reward_vec.clear();
    for (int i = 0; i < tp->ag_tree.at(agn).tree_vec.at(l).level_vec.size(); i++) {
        if (parent_number == tp->ag_tree.at(agn).tree_vec.at(l).level_vec.at(i).p_number) {
            reward_vec.push_back(tp->ag_tree.at(agn).tree_vec.at(l).level_vec.at(i).q_node);
            action_check = true;
        }
    }

    if (action_check == false) {
        current_node = parent_number;
        action = 4; //Stay in position (just in case)
        goto skip;
    }

    best = *max_element(reward_vec.begin(), reward_vec.end()); //Best Q-value from among child nodes in level
    reward_vec.clear();
    count = 0;
    for (int i = 0; i < tp->ag_tree.at(agn).tree_vec.at(l).level_vec.size(); i++) {
        if (parent_number == tp->ag_tree.at(agn).tree_vec.at(l).level_vec.at(i).p_number) {
            if (best == tp->ag_tree.at(agn).tree_vec.at(l).level_vec.at(i).q_node) {
                action = tp->ag_tree.at(agn).tree_vec.at(l).level_vec.at(i).action;
                current_node = tp->ag_tree.at(agn).tree_vec.at(l).level_vec.at(i).n_number; //Selected node
                count++;
                i = tp->ag_tree.at(agn).tree_vec.at(l).level_vec.size();
                break;
            }
        }
    }
    assert(count == 1);
    skip:
    assert(action >= 0);
    assert(action <= 4);
    return action;
}

void monte_carlo::create_root_nodes(multi_tree *tp, vector<agent> agentes) {
    // cout << " ************** CREATE ROOT NODES ****************" << endl;

    agente = 0;
    max_agents = agentes.size();
    pos_agentes.resize(max_agents);
    start_agentes.resize(max_agents);
    goal_agentes.resize(max_agents);


    for (int ag = 0; ag < max_agents; ag++) { //Create Root Nodes
        pos_agentes[ag].agent_x = agentes.at(ag).start.agent_x;
        pos_agentes[ag].agent_y = agentes.at(ag).start.agent_y;

        start_agentes[ag].agent_x = agentes.at(ag).start.agent_x;
        start_agentes[ag].agent_y = agentes.at(ag).start.agent_y;

        goal_agentes[ag].agent_x = agentes.at(ag).goal.agent_x;
        goal_agentes[ag].agent_y = agentes.at(ag).goal.agent_y;
    }

    tp->create_tree(); //Create a tree for each point
    tp->create_level(agente); //Create Root Level
    n_nodes = tp->ag_tree.at(agente).tree_vec.at(0).level_vec.size();
    tp->create_root_node(0, n_nodes, -1, -1, 0, node_number, 0, -1, -1, max_agents); //Root Node

}

//update tree, update root node n = posicao
void monte_carlo::update_tree(multi_tree *tp, int level, int n) {
    int a = 0;
    //update no raiz (level 0), o no do level 1 vira raiz (level 0)
    tp->ag_tree.at(0).tree_vec.at(0).level_vec.at(0) = tp->ag_tree.at(0).tree_vec.at(level).level_vec.at(n);
    vector<int> parents;

    for (int en = 0; en < tp->ag_tree.at(a).tree_vec.at(1).level_vec.size(); en++) {
        if (en != n) {
            parents.push_back(tp->ag_tree.at(a).tree_vec.at(1).level_vec.at(en).n_number);
        }
    }
    //deleta nivel  1 da arvore
    tp->ag_tree.at(0).tree_vec.erase(tp->ag_tree.at(0).tree_vec.begin() + 1);


    for (int lev = 1; lev < tp->ag_tree.at(a).tree_vec.size(); lev++) {
        for (int en = 0; en < tp->ag_tree.at(a).tree_vec.at(lev).level_vec.size(); en++) {
            int p_num = tp->ag_tree.at(a).tree_vec.at(lev).level_vec.at(en).p_number;
            if (std::find(parents.begin(), parents.end(), p_num) != parents.end()){
                int no = tp->ag_tree.at(a).tree_vec.at(lev).level_vec.at(en).n_number;
                parents.push_back(no);
                tp->ag_tree.at(0).tree_vec.at(lev).level_vec.erase(tp->ag_tree.at(0).tree_vec.at(lev).level_vec.begin() + en);
                en --;
            }

        }
    }

}

void monte_carlo::mc_search(multi_tree *tp) { //4-step MCTS process
#ifndef MCSEARCH
    cout << " ==========================================" << endl;
tp->print_tree(a_num);
cout << " ==========================================" << endl;
cout << " ************** MC SEARCH ****************" << endl;

#endif
    select(tp);
    expand(tp);
    for (int en = 0;
         en < tp->ag_tree.at(a_num).tree_vec.at(lev).level_vec.size(); en++) { //Rollout all newly expanded nodes
        if (parent_number == tp->ag_tree.at(a_num).tree_vec.at(lev).level_vec.at(en).p_number) {
            rollout(tp, en);
        }
    }
    back_propagate(tp);
}

//SELECTION------------------------------------------------------------------------------------------------------------
void monte_carlo::select(multi_tree *tp) {
#ifndef SELECT

    cout << " ************** SELECT ****************" << endl;
#endif

    double best, q_val, q_ucb1;
    bool no_nodes;
    previous_x = tp->ag_tree.at(a_num).tree_vec.at(0).level_vec.at(0).x; //Set Coordinates to Root Position
    previous_y = tp->ag_tree.at(a_num).tree_vec.at(0).level_vec.at(0).y;
    parent_number = tp->ag_tree.at(a_num).tree_vec.at(0).level_vec.at(0).n_number;
    parent_visit = tp->ag_tree.at(a_num).tree_vec.at(0).level_vec.at(0).visit_count;

    if (tp->ag_tree.at(a_num).tree_vec.size() == 1) { //If there is only the root level, go immediately to expansion
        p_lev = 0;
        lev = 1;
#ifndef SELECT
        cout << "(Arvore na raiz, pula select e vai pra fase de expand)" << endl;
#endif

        previous_x = start_agentes[agente].agent_x;
        previous_y = start_agentes[agente].agent_y;
        goto expansion_phase; //If the tree has not been expanded, skip first selection
    }

    reward_vec.clear();
    for (int i = 1;
         i < tp->ag_tree.at(a_num).tree_vec.size(); i++) { //Starting at level 1 record all q values of child nodes
        no_nodes = true; //Boolean used to find when we reach an unexpanded node
        for (int j = 0;
             j < tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.size(); j++) { //For all nodes in the current level
            if (parent_number ==
                tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(j).p_number) { //If it is a child node
                node_visit = tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(j).visit_count;
                q_val = tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(j).q_node; //Current Q-value of node
                q_ucb1 = q_val + (epsilon * sqrt(log(parent_visit) / node_visit)); //Q_UCB1 value for selection
                tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(j).UCB1 = q_ucb1;
                reward_vec.push_back(tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(j).UCB1);
                no_nodes = false; //If a child node was found, the current parent node is expanded
                node = j;

            }
        }

        if (no_nodes == true) {
            lev = i; //If there are no children of the current parent in this level, then the parent needs to be expanded
#ifndef SELECT

            cout << "(Pai nao tem filho, pai: " << parent_number << " precisa ser expandido)" << endl;
#endif

            goto expansion_phase;
        }

        best = *max_element(reward_vec.begin(),
                            reward_vec.end()); //Pick the highest Q-value from among child nodes found
        reward_vec.clear();
        for (int k = 0; k < tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.size(); k++) {
            if (best == tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(k).UCB1 &&
                parent_number == tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(k).p_number) {
                parent_visit = tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(k).visit_count;
                parent_number = tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(k).n_number; //New parent node
                pos_agentes = tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(k).pos_agentes;

                if (tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(k).pos_agentes[agente].agent_x == -1) {
                    previous_x = start_agentes[agente].agent_x;
                    previous_y = start_agentes[agente].agent_y;
                    pos_agentes = tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(k).pos_agentes;
                }
//                else{
//                    previous_x = tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(k).pos_agentes[agente].agent_x;
//                    previous_y = tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(k).pos_agentes[agente].agent_y;
//                    //pos_agentes = tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(k).pos_agentes;
//                }
                lev = i + 1; //Needed in case selection reaches the bottom most level of the tree
                p_lev = i;
                node = k;
                break;
            }
        }
    }
    expansion_phase:
    assert(p_lev == lev - 1);
    assert(lev <= tp->ag_tree.at(a_num).tree_vec.size());
}

//EXPANSION---------------------------------------------------------------------------------------------------------------
void monte_carlo::expand(multi_tree *tp) {

    int ag_expand = tp->ag_tree.at(a_num).tree_vec.at(p_lev).level_vec.at(node).a_number;
    agente = ag_expand + 1;
    if (agente >= max_agents) {
        agente = 0;
    }
    while (at_goal[agente]) {
        agente = agente + 1;
        if (agente >= max_agents) {
            agente = 0;
        }
    }

#ifndef SELECT
    std::cout << "Node to expand " << parent_number << " lev " << lev <<" Agente "<<agente<<" - N "<<node<<endl;
#endif
    if (tp->ag_tree.at(a_num).tree_vec.at(p_lev).level_vec.at(node).pos_agentes[agente].agent_x == -1) {
        previous_x = start_agentes[agente].agent_x;
        previous_y = start_agentes[agente].agent_y;
        pos_agentes = tp->ag_tree.at(a_num).tree_vec.at(p_lev).level_vec.at(node).pos_agentes;
    } else {
        previous_x = pos_agentes[agente].agent_x;
        previous_y = pos_agentes[agente].agent_y;
    }
    /// se os agentes desaparecem quando chegam no final.
    for(int i =0; i < max_agents; i++){
        if(at_goal[i]){
            pos_agentes[i].agent_x = -1;
            pos_agentes[i].agent_y = -1;

        }
    }
#ifndef SELECT
    cout << " AGENTE " << agente << " - " << previous_x << "," << previous_y<< endl;

    cout << " ************** EXPAND ****************" << endl;
#endif
    no_expanded++;

    if (lev == tp->ag_tree.at(a_num).tree_vec.size()) {
        tp->create_level(a_num);
    }
    move_left(tp); //Explores Left Action (x--)
    move_up(tp); //Explores Up Action (y++)
    move_down(tp); //Explores Down Action (y--)
    move_right(tp); //Explores Right Action (x++)
    no_move(tp);
}

void monte_carlo::move_left(multi_tree *tp) {
    reset_coordinates();
    ax--;
#ifndef EXPAND
    cout<<"----- move left -------> "<<ax<<","<<ay;
#endif
    pruning(tp);
    if (action_check == true) { //If action is legal, add node to tree
        update_node_numbers(tp);
        tp->create_node(lev, n_nodes, ax, ay, a_num, node_number, parent_number, 0, agente);
    }
#ifndef EXPAND
    else{
        cout<<"----- action not legal-------"<<endl;
    }
#endif

}

void monte_carlo::move_up(multi_tree *tp) {
    reset_coordinates();
    ay++;
#ifndef EXPAND
    cout<<"----- move up ---------> "<<ax<<","<<ay;
#endif
    pruning(tp);
    if (action_check == true) { //If action is legal, add node to tree
        update_node_numbers(tp);
        tp->create_node(lev, n_nodes, ax, ay, a_num, node_number, parent_number, 1, agente);
    }
#ifndef EXPAND
    else{
        cout<<"----- action not legal-------"<<endl;
    }
#endif
}

void monte_carlo::move_down(multi_tree *tp) {
    reset_coordinates();
    ay--;
#ifndef EXPAND
    cout<<"----- move down -------> "<<ax<<","<<ay;
#endif
    pruning(tp);
    if (action_check == true) { //If action is legal, add node to tree
        update_node_numbers(tp);
        tp->create_node(lev, n_nodes, ax, ay, a_num, node_number, parent_number, 2, agente);
    }
#ifndef EXPAND
    else{
        cout<<"----- action not legal-------"<<endl;
    }
#endif
}

void monte_carlo::move_right(multi_tree *tp) {
    reset_coordinates();
    ax++;
#ifndef EXPAND
    cout<<"----- move right-------> "<<ax<<","<<ay;
#endif
    pruning(tp);
    if (action_check == true) { //If action is legal, add node to tree
        update_node_numbers(tp);
        tp->create_node(lev, n_nodes, ax, ay, a_num, node_number, parent_number, 3, agente);
    }
#ifndef EXPAND
    else{
        cout<<"----- action not legal-------"<<endl;
    }
#endif
}

void monte_carlo::no_move(multi_tree *tp) {
    reset_coordinates();
#ifndef EXPAND
    cout<<"----- no move ---------> "<<ax<<","<<ay;
#endif
    check_boundaries(ax, ay);
    prune(tp, lev - (max_lev));
    if (action_check == true) { //If action is legal, add node to tree
        update_node_numbers(tp);
        tp->create_node(lev, n_nodes, ax, ay, a_num, node_number, parent_number, 4, agente);
    }
#ifndef EXPAND
    else{
        cout<<"----- action not legal-------"<<endl;
    }
#endif
}

void monte_carlo::pruning(multi_tree *tp) {
    check_boundaries(ax, ay); //Agent cannot go out of bounds
//    if (action_check == true) {
//        prune(tp, lev); //Agent cannot revisit a previous state during a MCTS simulation
//    }
}

void monte_carlo::check_boundaries(double xx, double yy) { //Illegal actions are not added to the tree
    action_check = true;
    if (xx >= x_lim) {
        action_check = false;
    }
    if (xx < 0) {
        action_check = false;
    }
    if (yy >= y_lim) {
        action_check = false;
    }
    if (yy < 0) {
        action_check = false;
    }
    if(action_check){
        if(!my_map[xx][yy]){
            action_check = false;
        }
    }

    //checar colisao com outros agentes de acordo com a posicao dos agentes nos n'os pais
    for (int i = 0; i < pos_agentes.size(); i++) {
        if (agente == i) continue;
        if (pos_agentes[i].agent_x == xx && pos_agentes[i].agent_y == yy) {
            action_check = false;
        }
    }

}

void monte_carlo::prune(multi_tree *tp, int l) { //A state is not visited more than once along a decision path
//    double prune_x, prune_y;
//    for (int i = l - 1;
//         i >= 0; i--) { //Starting from the parent level, make sure no duplicate states are already in the tree
//        for (int j = 0; j < tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.size(); j++) {
//            prune_x = tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(j).x;
//            prune_y = tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(j).y;
//            if (ax == prune_x && ay == prune_y) { //If a state has been visited before, prune the tree
//                action_check = false;
//                break;
//            }
//        }
//        if (action_check == false) {
//            break;
//        }
//    }
}

void monte_carlo::update_node_numbers(multi_tree *tp) {
    no_generated ++;
    node_number++;
    n_nodes = tp->ag_tree.at(a_num).tree_vec.at(lev).level_vec.size();
}

void monte_carlo::reset_coordinates() { //Reset point coordinates to parent node state
    ax = previous_x;
    ay = previous_y;
}

//Rollout-----------------------------------------------------------------------------------------------------------------
void monte_carlo::rollout(multi_tree *tp, int n) { //n = node position in level of tree


    int a = 0;
    double q_val;
    q_val = 0;
    int act;
    double x, y;

    x = tp->ag_tree.at(a_num).tree_vec.at(lev).level_vec.at(n).x;
    y = tp->ag_tree.at(a_num).tree_vec.at(lev).level_vec.at(n).y;
#ifndef ROLLOUT

    cout << " ************** ROLLOUT ****************" << endl;
    cout  << "(" << x << "," << y << ")" << " -> ";
#endif

    //manhattan distance
    int xgoal = goal_agentes[agente].agent_x;
    int ygoal = goal_agentes[agente].agent_y;
    double dist = std::abs(x - xgoal) + std::abs(y - ygoal);
    if (dist == 0) dist = 1;
    dist = 1 / dist;

#ifndef ROLLOUT2
    cout << "dist " << dist;
#endif

    while (a < rollout_iterations) {
        x = tp->ag_tree.at(a_num).tree_vec.at(lev).level_vec.at(n).x;
        y = tp->ag_tree.at(a_num).tree_vec.at(lev).level_vec.at(n).y;

#ifndef ROLLOUT2
        cout << endl << "(" << x << "," << y << ")" << " -> ";
#endif
        for (int i = 0; i < rollout_steps; i++) {
            act = rand() % 4; //Rollout policy is to take a random exploratory action
            if (act == 0) {
                x--;
                check_boundaries(x, y);
                if (action_check == false) {
                    x++;
                }

#ifndef ROLLOUT2
                else {

                    cout << "(" << x << "," << y << ")" << "->";
                }
#endif

            }
            if (act == 1) {
                y++;
                check_boundaries(x, y);
                if (action_check == false) {
                    y--;
                }
#ifndef ROLLOUT2
                else {

                    cout << "(" << x << "," << y << ")" << "->";
                }
#endif

            }
            if (act == 2) {
                y--;
                check_boundaries(x, y);
                if (action_check == false) {
                    y++;
                }
#ifndef ROLLOUT2
                else {

                    cout << "(" << x << "," << y << ")" << "->";
                }
#endif
            }
            if (act == 3) {
                x++;
                check_boundaries(x, y);
                if (action_check == false) {
                    x--;
                }
#ifndef ROLLOUT2
                else {

                    cout << "(" << x << "," << y << ")" << "->";
                }
#endif
            }
#ifndef ROLLOUT2
            if (act == 4) {
                cout << "(" << x << "," << y << ")" << "->";
            }
#endif
            //if act == 4, no move is made (stand still)
            // for(int j = 0; j < map->n_agents; j++){
            if (x == goal_agentes.at(agente).agent_x && y == goal_agentes.at(agente).agent_y) {
                q_val += rollout_reward; //Receive a rollout reward when a goal is reached
                break;
            }
            // }
        }
        a++;
    }
#ifndef ROLLOUT
    cout <<endl<< "Node: " << tp->ag_tree.at(a_num).tree_vec.at(lev).level_vec.at(n).n_number << " q: " << q_val << " dist: "
         << dist << " q_value: " << q_val + dist
         << endl;
#endif

    tp->ag_tree.at(a_num).tree_vec.at(lev).level_vec.at(n).q_node = q_val + dist;
}

//BACK-PROPAGATION-----------------------------------------------------------------------------------------------------------
void monte_carlo::back_propagate(multi_tree *tp) {
    #ifndef BACK_PROPAGATE
    cout << " ************** BACK PROPAGATE ****************" << endl;
#endif

    double n;
    n = 0;
    double q_val, q_prev;

    reward_vec.clear();
    for (int i = 0; i < tp->ag_tree.at(a_num).tree_vec.at(lev).level_vec.size(); i++) {
        if (parent_number == tp->ag_tree.at(a_num).tree_vec.at(lev).level_vec.at(i).p_number) {
            reward_vec.push_back(tp->ag_tree.at(a_num).tree_vec.at(lev).level_vec.at(i).q_node);
            n++;
        }
    }

    if (n > 0) {
        q_val = *max_element(reward_vec.begin(), reward_vec.end());
       // q_val = accumulate( reward_vec.begin(), reward_vec.end(), 0.0)/reward_vec.size();
    }
    if (n == 0) {
        q_val = 0;
    }

    //Back-Propagate
    for (int i = lev - 1; i >= 0; i--) { //i = level number
        for (int j = 0; j < tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.size(); j++) { //j = node
            if (parent_number == tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(j).n_number) {
                q_prev = tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(j).q_node;
                tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(j).visit_count += 1;
                node_visit = tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(j).visit_count;
              //  tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(j).q_node = q_prev + q_val;
                tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(j).q_node = q_prev + ((q_val - q_prev) / node_visit);
                parent_number = tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(j).p_number;
#ifndef BACK_PROPAGATE
                cout << "Node: " << tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(j).n_number << " q-new: "<< tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(j).q_node <<" q-prev: "<<q_prev<<" visit: "<<node_visit<<" m: "<<q_val<< endl;
#endif
                break;
            }
        }

        n = 0;
        reward_vec.clear();
        for (int tn = 0; tn < tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.size(); tn++) {
            if (parent_number == tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(tn).p_number) {
                reward_vec.push_back(tp->ag_tree.at(a_num).tree_vec.at(i).level_vec.at(tn).q_node);
                n++;
            }
        }
        assert(n > 0);
        q_val = *max_element(reward_vec.begin(),
                             reward_vec.end()); //A parent's value is the maximum of its children's action value
        reward_vec.clear();
    }
}

void monte_carlo::back_propagate_evals(multi_agent *map, multi_tree *tp, double reward, int agn, int l, int nn) {
    double q_val, q_prev;
    int count;
    //count = count for number of nodes, nv = node visit count
//        cout << " ==========================================" << endl;
//    tp->print_tree(agn);
//    cout << " ==========================================" << endl;
    cout << " ************** BACK PROPAGATE EVALS ****************" << endl;


    count = 0;
    for (int i = 0; i < tp->ag_tree.at(agn).tree_vec.at(l).level_vec.size(); i++) { //Update current node value
        if (nn == tp->ag_tree.at(agn).tree_vec.at(l).level_vec.at(i).n_number) { //If this is the correct node
            q_val = reward; //Reward gained from evals
            q_prev = tp->ag_tree.at(agn).tree_vec.at(l).level_vec.at(i).q_node; //Previous estimate of Q value
            tp->ag_tree.at(agn).tree_vec.at(l).level_vec.at(i).visit_count += 1;
            node_visit = tp->ag_tree.at(agn).tree_vec.at(l).level_vec.at(i).visit_count;
            tp->ag_tree.at(agn).tree_vec.at(l).level_vec.at(i).q_node =
                    q_prev + ((q_val - q_prev) / node_visit); //Updated Q values
            cout << "Agente " << agn << " No " << nn << " Old Q value: " << q_prev << " Novo q value: "
                 << tp->ag_tree.at(agn).tree_vec.at(l).level_vec.at(i).q_node << endl;
            parent_number = tp->ag_tree.at(agn).tree_vec.at(l).level_vec.at(i).p_number;
            count++;
            break;
        }
    }
    assert(count > 0);

    count = 0;
    reward_vec.clear();
    for (int tn = 0; tn < tp->ag_tree.at(agn).tree_vec.at(l).level_vec.size(); tn++) {
        if (parent_number == tp->ag_tree.at(agn).tree_vec.at(l).level_vec.at(tn).p_number) {
            reward_vec.push_back(tp->ag_tree.at(agn).tree_vec.at(l).level_vec.at(tn).q_node);
            count++;
        }
    }
    assert(count > 0);
    q_val = *max_element(reward_vec.begin(), reward_vec.end());

    for (int i = l - 1; i >= 0; i--) { //i = level
        for (int j = 0; j < tp->ag_tree.at(agn).tree_vec.at(i).level_vec.size(); j++) { //j = node
            if (parent_number == tp->ag_tree.at(agn).tree_vec.at(i).level_vec.at(j).n_number) {
                q_prev = tp->ag_tree.at(agn).tree_vec.at(i).level_vec.at(j).q_node;
                tp->ag_tree.at(agn).tree_vec.at(i).level_vec.at(j).visit_count += 1;
                node_visit = tp->ag_tree.at(agn).tree_vec.at(i).level_vec.at(j).visit_count;
                tp->ag_tree.at(agn).tree_vec.at(i).level_vec.at(j).q_node = q_prev + ((q_val - q_prev) / node_visit);
                parent_number = tp->ag_tree.at(agn).tree_vec.at(i).level_vec.at(j).p_number;
                break;
            }
        }

        count = 0;
        reward_vec.clear();
        for (int tn = 0; tn < tp->ag_tree.at(agn).tree_vec.at(i).level_vec.size(); tn++) {
            if (parent_number == tp->ag_tree.at(agn).tree_vec.at(i).level_vec.at(tn).p_number) {
                reward_vec.push_back(tp->ag_tree.at(agn).tree_vec.at(i).level_vec.at(tn).q_node);
                count++;
            }
        }


//        cout << " ==========================================" << endl;
//        tp->print_tree(agn);
//        cout << " ==========================================" << endl;

        assert(count > 0);
        q_val = *max_element(reward_vec.begin(), reward_vec.end());
    }
}

