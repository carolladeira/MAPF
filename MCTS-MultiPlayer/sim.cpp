
#include "sim.hpp"
#include <iostream>
#include <string>

void gridworld::agente_move(multi_tree *tp, monte_carlo *mcp) {

    int a = 0;
    std::vector<point> agents_points;
    int parent_number = 0;
    point ag;

    mcp->parent_number = tp->ag_tree.at(0).tree_vec.at(0).level_vec.at(0).n_number;
    parent_number = mcp->parent_number;
    cout << " ==========================================" << endl;
    tp->print_tree(0);
    cout << " ==========================================" << endl;
    cout << " ************** MC SEARCH ****************" << endl;
    ///isso esta errado, arrumei no outro programa
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
                    this->agents[id_agente].path_agent.push_back(ag);
                    if (agx == this->agents[id_agente].goal.agent_x && agy == this->agents[id_agente].goal.agent_y) {
                        mcp->set_agent_goal(id_agente);
                        //cout<<"Agente"<<id_agente<<endl;
                        this->agents[id_agente].final_timestep = path_agents[id_agente].size() - 1;
                    }
                }

//                cout<<"Agente "<<id_agente<<" -> "<< agx << "," << agy<<endl;
//                std::cout << "Node " << nn << " pos " << lev << " UCB1 " << ucb << " N-visit " << n_visit << " | " << agx << "," << agy << " parent  " << p_num << endl;
                mcp->update_tree(tp, lev, en);
                return;
            }
        }
    }
}

void gridworld::print_path(monte_carlo *mcp) {

//    for (int i = 0; i < path_agents.size(); i++) {
//        cout << endl << "Agente " << i << " ";
//        for (int j = 0; j < path_agents[i].size(); j++) {
//            cout << " -> [" << j << "]" << path_agents[i][j].agent_x << "," << path_agents[i][j].agent_y;
//        }
//    }


    for (int i = 0; i < path_agents.size(); i++) {
        if (!mcp->at_goal[i]) {
            cout << endl << "Agente " << i << " ";
            for (int j = 0; j < path_agents[i].size(); j++) {
                cout << " -> [" << path_agents[i][j].agent_x + path_agents[i][j].agent_y * y_dim << "]" << path_agents[i][j].agent_x << "," << path_agents[i][j].agent_y;
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

void gridworld::loadScen(ScenarioLoader scen, monte_carlo *mcp) {

    this->x_dim = scen.width;
    this->y_dim = scen.height;

    mcp->node_number = 0;
    mcp->x_lim = x_dim;
    mcp->y_lim = y_dim;

    this->my_map.resize(y_dim, vector<bool>(x_dim));
    int t = 0;
    for (int i = 0; i < y_dim; i++) {
        for (int j = 0; j < x_dim; j++) {
            //   my_map[j].resize(x_dim);
            if (scen.map[t] == 1 || scen.map[t] == 2 || scen.map[t] == 4) {
                my_map[j][i] = false;
            } else {
                my_map[j][i] = true;
            }
            t++;
        }
    }
}

void gridworld::addTasks(ScenarioLoader scen, int qt) {
    agents.clear();
    agents.resize(qt);
    Experiment exp = scen.GetNthExperiment(1);
    this->mapName = exp.GetMapName();
    this->scenName = scen.GetScenarioName();
    this->n_agents = qt;
    this->path_agents.resize(qt);

    int c = 97;
    int ca = 65;
    for (int i = 0; i < qt; i++) {
        Experiment exp = scen.GetNthExperiment(i);
        agents[i].id = i;
        agents[i].start.agent_x = exp.GetStartX();
        agents[i].start.agent_y = exp.GetStartY();
        agents[i].goal.agent_x = exp.GetGoalX();
        agents[i].goal.agent_y = exp.GetGoalY();
        agents[i].s = c;
        c++;
        agents[i].g = ca;
        ca++;
        path_agents[i].push_back(agents[i].start);

    }
}

void gridworld::print_map(int t) {

    char *mapChar = new char[x_dim * y_dim];
    int n = 0;
    for (int y = 0; y < y_dim; y++) {
        for (int x = 0; x < x_dim; x++) {
            mapChar[n] = '.';
            if (!my_map[x][y]) mapChar[n] = '*';
            for (int i = 0; i < path_agents.size(); i++) {
                if (path_agents[i][path_agents[i].size() - 1].agent_y == y &&
                    path_agents[i][path_agents[i].size() - 1].agent_x == x) {
                    mapChar[n] = agents[i].s;
                }
                if (agents[i].goal.agent_y == y && agents[i].goal.agent_x == x) {
                    mapChar[n] = agents[i].g;
                }
            }
            n++;
        }
    }
    std::cout << "MAP:"<<endl;
    int h = 0;
    for (int i = 0; i < x_dim; i++) {
        std::cout <<i<< " ";
        for (int j = 0; j < y_dim; j++) {
            std::cout << mapChar[h];
            h++;
        }
        cout<<endl;
    }
    std::cout << std::endl;
    delete[] mapChar;

}

void gridworld::info()
{
    for(int i = 0; i < agents.size(); i++){
        soc += agents[i].final_timestep;
        if(mkspn < agents[i].final_timestep){
            mkspn = agents[i].final_timestep;
        }
    }
}
