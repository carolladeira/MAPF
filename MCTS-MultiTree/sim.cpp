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
    path_agents.resize(n_agents);

    for (int i = 0; i < path_agents.size(); i++) {
        int x = map->agent_start_pos.at(i).agent_x;
        int y = map->agent_start_pos.at(i).agent_y;
        point p;
        p.agent_x = x;
        p.agent_y = y;
        path_agents[i].push_back(p);

    }
}

//Credit Evaluation Functions ---------------------------------------------------------------------------------------
void gridworld::cred_evals(multi_agent *map, multi_tree *tp, monte_carlo *mcp) {
    if (credit_type == 1) { //Local Eval
        reset_all_agents(map, tp); //Resets all agent coordinates to root position
        calculate_local(map, mcp, tp);
        for (int i = 0; i < n_agents; i++) { //Agent
            mcp->back_propagate_evals(map, tp, agent_rewards.at(i), i, end_lev.at(i), dif_node_vec.at(i));
        }
    }

}

void gridworld::reset_all_agents(multi_agent *map, multi_tree *tp) { //Resets all agents to initial positions
    for (int i = 0; i < n_agents; i++) { //Agent Number
        map->agent_vec.at(i).agent_x = tp->ag_tree.at(i).tree_vec.at(0).level_vec.at(0).x; //Initial X
        map->agent_vec.at(i).agent_y = tp->ag_tree.at(i).tree_vec.at(0).level_vec.at(0).y; //Initial Y
        node_vec.at(i) = 0;
        ag_in_play.at(i) = true;
    }
}

void gridworld::calculate_local(multi_agent *map, monte_carlo *mcp, multi_tree *tp) { //s = size of largest tree
    int act, count;
    cout << " ==========================================" << endl;
    tp->print_tree(0);
    cout << " ==========================================" << endl;
    cout << " ************** MC SEARCH ****************" << endl;
    for (int i = 0; i < agent_rewards.size(); i++) { //zero the vector at the beginning
        agent_rewards.at(i) = 0;
    }

    for (int i = 1; i < max_lev; i++) { //Each Agent takes a step if able
        count = 0;
        for (int aa = 0; aa < n_agents; aa++) {
            if (ag_in_play.at(aa) == false) {
                count++;
            }
        }
        if (count == n_agents) { //If all agents have no more moves left to make, finish eval
            i = max_lev;
            break;
        }
        for (int a = 0; a < n_agents; a++) { //Agent Number
            if (i < tp->ag_tree.at(a).tree_vec.size()) { //If level exceeds the maximum size of tree, do not calculate
                mcp->parent_number = node_vec.at(
                        a); //Parent is the node number of the previously selected node initially 0 for root
                act = mcp->select_move(tp, a, i); //(agent_number, level) choose best child node
                if (mcp->action_check == false) {
                    ag_in_play.at(a) = false; //Agent is at terminal node and cannot move
                }
                if (mcp->action_check == true && ag_in_play.at(a) == true) { //If a child node was found
                    end_lev.at(a) = i;
                    node_vec.at(a) = mcp->current_node; //Node selected from select move
                    map->agent_move(a, act);
                    map->check_agent_status(a);
                    map->check_agent_coordinates(a);
                    if (map->agent_at_goal == true) { //Agent is at a goal
                        cout << "agente chegou na posicao de destino" << endl;
                        agent_rewards.at(a) = goal_reward;
                        ag_in_play.at(a) = false;
                    }
                    if (map->agent_at_goal == false) { //Agent is not at a goal
                        cout << "agente nao chegou na posicao de destino" << endl;
                        agent_rewards.at(a) = step_penalty;
                    }
                }
            }
        }
    }
    dif_node_vec = node_vec;
}

void gridworld::agente_move(multi_tree *tp, monte_carlo *mcp, int agent) {

    int a = agent;
    std::vector<point> agents_points;
    int parent_number = 0;
    point ag;

    mcp->parent_number = tp->ag_tree.at(a).tree_vec.at(0).level_vec.at(0).n_number;
    parent_number = mcp->parent_number;
//    cout << " ==========================================" << endl;
//    tp->print_tree(a);
//    cout << " ==========================================" << endl;

    bool can_move = false;
    int lev = 1; //como atualizo a arvore somente ando no primeiro level
    int tam_level = tp->ag_tree.at(a).tree_vec.at(lev).level_vec.size();
    int vz = 0, act;
    while (!can_move) {
        if (vz == tam_level) break;
        act = mcp->select_move(tp, a, lev);
        if (!checkCollision(a, act)) {
            can_move = true;
            break;
        }
        for (int en = 0; en < tp->ag_tree.at(a).tree_vec.at(lev).level_vec.size(); en++) {
            if (act == tp->ag_tree.at(a).tree_vec.at(lev).level_vec.at(en).action &&
                parent_number == tp->ag_tree.at(a).tree_vec.at(lev).level_vec.at(en).p_number) {
                tp->ag_tree.at(a).tree_vec.at(lev).level_vec.at(en).q_node = -10000;
            }
        }
        vz++;
    }
   // assert(vz > tam_level -1);

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
            path_agents[agent].push_back(ag);
//                if (!mcp->at_goal[id_agente]) {
//
//                    this->agents[id_agente].path_agent.push_back(ag);
//                    if (agx == this->agents[id_agente].goal.agent_x && agy == this->agents[id_agente].goal.agent_y) {
//                        mcp->set_agent_goal(id_agente);
//                        //cout<<"Agente"<<id_agente<<endl;
//                        this->agents[id_agente].final_timestep = path_agents[id_agente].size() - 1;
//                    }
//                }
//
////                cout<<"Agente "<<id_agente<<" -> "<< agx << "," << agy<<endl;
//            std::cout << "Node " << nn << " pos " << lev << " UCB1 " << ucb << " N-visit " << n_visit << " | " << agx
//                      << "," << agy << " parent  " << p_num << endl;
            mcp->update_tree(tp, lev, en, agent);
            return;
        }

    }
}

void gridworld::system_rollout(multi_agent *map, multi_tree *tp, monte_carlo *mcp) {
    int act, count;
    sys_reward = 0;
    for (int i = 0; i < agent_rewards.size(); i++) { //zero the vector at the beginning
        agent_rewards.at(i) = 0;
    }
    reset_all_agents(map, tp); //reset agents to starting positions
    for (int i = 1; i < max_lev; i++) { //Each Agent takes a step if able
        count = 0;
        for (int aa = 0; aa < n_agents; aa++) {
            if (ag_in_play.at(aa) == false) {
                count++;
            }
        }
        if (count == n_agents) { //If all agents have no more moves left to make, finish eval
            break;
        }
        for (int a = 0; a < n_agents; a++) { //Agent Number
            if (i < tp->ag_tree.at(a).tree_vec.size()) { //If level exceeds the maximum size of tree, do not calculate
                mcp->parent_number = node_vec.at(a); //Parent is the node number of the previously selected node
                act = mcp->select_move(tp, a, i); //(agent_number, level) choose best child node

                if (mcp->action_check == false) {
                    ag_in_play.at(a) = false; //Agent is at a dead end in the tree
                }
                if (mcp->action_check == true && ag_in_play.at(a) == true) { //If a child node was found
                    end_lev.at(a) = i;
                    node_vec.at(a) = mcp->current_node; //Node selected from select move
                    map->agent_move(a, act);
                    map->check_agent_status(a);
                    map->check_agent_coordinates(a);
                    if (map->agent_at_goal == true && map->unique_pos == true) { //Agent at uncaptured goal
                        agent_rewards.at(a) = goal_reward;
                        ag_in_play.at(a) = false;
                    }
                    if (map->agent_at_goal == true && map->unique_pos == false) { //Agent at captured goal
                        cout << "ver 2 " << endl;
                        agent_rewards.at(a) = 0;
                        ag_in_play.at(a) = false;
                    }
                }
            }
        }
    }

    for (int i = 0; i < agent_rewards.size(); i++) { //zero the vector at the beginning
        sys_reward += agent_rewards.at(i);
    }
}

void gridworld::clear_all_vectors(multi_agent *map, monte_carlo *mcp, multi_tree *tp) {
    tp->ag_tree.clear();
    mcp->reward_vec.clear();
    map->agent_vec.clear();
    map->goal_vec.clear();
    map->agent_start_pos.clear();
    map->goal_start_pos.clear();
    mcp->n_num_vec.clear();
    node_vec.clear();
    ag_in_play.clear();
    agent_rewards.clear();
    end_lev.clear();
}

void gridworld::all_agents_move(multi_tree *tp, monte_carlo *mcp) {


    //TODO fazer a lista de prioridades de agente de acordo com o makespan

    for (int i = 0; i < n_agents; i++) {
        if(at_goal[i])continue;
        agente_move(tp, mcp, i);
    }
    timestep++;


}

bool gridworld::checkCollision(int agent, int act) {

    int ax, ay, ax_curr, ay_curr;
    int next_timestep = timestep +1;
    ax_curr = path_agents[agent].at(timestep).agent_x;
    ay_curr = path_agents[agent].at(timestep).agent_y;
    ax = ax_curr; ay = ay_curr;
    cout<<" Agente: "<<agent<<" | "<<ax<<","<<ay<<" -> ";
    if(act == 0)ax--;//Move left
    else if(act == 1)ay++;//Move "up"
    else if(act == 2)ay--;//Move "down"
    else if(act == 3)ax++; //Move "right"
//    if(act == 4) //Do not move
    for (int i = 0; i < path_agents.size(); i++) {
        if (i == agent) continue;
        if (path_agents[i].size() <= next_timestep) continue;
        int x_curr = path_agents[i].at(timestep).agent_x;
        int y_curr = path_agents[i].at(timestep).agent_y;
        int x_next = path_agents[i].at(next_timestep).agent_x;
        int y_next = path_agents[i].at(next_timestep).agent_y;
        if (ax == x_next && ay == y_next) return true; // vertex collision
        else if (x_curr == ax && y_curr == ay && x_next == ax_curr && y_next == ay_curr) return true;
    }
    cout<<ax<< ","<<ay<<endl;
    return false;

}
