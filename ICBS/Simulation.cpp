//
// Created by carol on 12/17/18.
//

#include "Simulation.h"

#define INF 1000000000
#define maxtime 10000

void Simulation::run_CBS_TA(bool bestInd) {

//    int sumofcost = 0;
//    vector<bool> last_vis(agents.size(), false);
//    int tot = 0;
//    int aux = 1;
//    BFS();
//    GA_TA ga;
//    bool adc = false;
//    vector<vector<int>> task_agents;
//    list_agents = ga.run_GA(agents, list_taskset, token, tasks_total, Dis, bestInd);
//    this->tempoTotalGA = ga.tempo_GA + this->tempoTotalGA;
//
//    for (timestep = 0; true; timestep++) {
//        // cout << "Timestep " << timestep << " " << tot << endl;
//
//        if (timestep > t_task) {
//            bool finish = true;
//            for (int i = 0; i < agents.size(); i++)
//                if (list_agents[i].size() > 0) finish = false;
//            if (finish) {
//                // printf("%d\n", tasks_total.size());
//                // cout << "MAKESPAN " << timestep - 1 << endl;
//                break;
//            }
//        }
//
//        for (int i = aux; i <= timestep; i++) {
//            if (taskset[i].empty())continue;
//            if (i >= taskset.size()) break;
//            aux = i + 1;
//            for (vector<Task>::iterator it = taskset[i].begin(); it != taskset[i].end(); it++) {
//                list_taskset.push_back(&(*it));
//                adc = true;
//            }
//        }
//        vector<int> need_plan;
//
//
//        for (int i = 0; i < agents.size(); i++) agents[i].loc = agents[i].path[timestep];
//
//
//        if (adc) {
//            if (!list_taskset.empty()) {
//                vector<Agent> copy_agents = agents;
//                for (int i = 0; i < agents.size(); i++) {
//                    if (agents[i].delivering) {
//                        int t = timestep;
//                        while (agents[i].path[t] != agents[i].next_ep->loc) t++;
//                        copy_agents[i].finish_time = t;
//                        copy_agents[i].loc = agents[i].path[t];
//                        continue;
//                    }
//                    copy_agents[i].loc = agents[i].path[timestep];
//                    copy_agents[i].finish_time = timestep;
//                }
//
//                task_agents = ga.run_GA(copy_agents, list_taskset, token, tasks_total, Dis, bestInd);
//                this->tempoTotalGA = ga.tempo_GA + this->tempoTotalGA;
//                adc = false;
//
//
//                for (int i = 0; i < agents.size(); i++) {
//                    if (agents[i].delivering) {
//                        int id_task = list_agents[i].front();
//                        list_agents[agents[i].id].clear();
//                        list_agents[agents[i].id].push_back(id_task);
//                        for (int j = 0; j < task_agents[i].size(); j++)
//                            list_agents[agents[i].id].push_back(task_agents[i][j]);
//                    } else {
//                        //se nao tem tarefas a serem adicionadas para o agente eu limpo a lista dele, e faco ele ficar na posicao que ele esta ate o final;
//                        //para nao repetir tarefas
//                        if (!task_agents[i].size()) {
//                            agents[i].next_ep = NULL;
//                            need_plan.push_back(agents[i].id);
//                            list_agents[agents[i].id].clear();
//                            continue;
//                        }
//                        int id_task = task_agents[i].front();
//                        Task *task = &tasks_total[id_task];
//                        if (agents[i].next_ep == NULL) {
//                            need_plan.push_back(agents[i].id);
//                            for (int j = 0; j < task_agents[i].size(); j++)
//                                list_agents[agents[i].id].push_back(task_agents[i][j]);
//                        } else if (agents[i].next_ep->loc != task->start->loc) {
//                            list_agents[agents[i].id].clear();
//                            need_plan.push_back(agents[i].id);
//                            for (int j = 0; j < task_agents[i].size(); j++)
//                                list_agents[agents[i].id].push_back(task_agents[i][j]);
//
//                        } else if (agents[i].next_ep->loc == task->start->loc) {
//                            list_agents[agents[i].id].clear();
//                            for (int j = 0; j < task_agents[i].size(); j++)
//                                list_agents[agents[i].id].push_back(task_agents[i][j]);
//                        }
//                    }
//                }
//            }
//        }
//        if (timestep == 0) {
//            for (int i = 0; i < agents.size(); i++) need_plan.push_back(i);
//        }
//
//        for (int i = 0; i < agents.size(); i++) {
//            if (!list_agents[i].size()) continue;
//            int id_task = list_agents[i].front();
//            Task *task = &tasks_total[id_task];
//            int id = i;
//            if (agents[id].loc == task->goal->loc && task->delivering == true && timestep == task->ag_arrive_goal) {
//                // cout << " Agente " << id << " termina tarefa " << task->id << " no tempo " << task->ag_arrive_goal << " na loc " << agents[id].loc << endl;
//                tot++;
//                task->agent_id = task->agent->id;
//                task->delivering = false;
//                task->agent->delivering = false;
//                agents[id].finish_time = timestep;
//                task->agent->next_ep = NULL;
//                task->agent->task = NULL;
//                list_agents[i].erase(list_agents[i].begin());
//                need_plan.push_back(i);
//            }
//        }
//
//        vector<Agent *> ag_icbs;
//        vector<bool> is_conflict(agents.size(), true);
//        for (int j = 0; j < need_plan.size(); j++) {
//            int id = need_plan[j];
//            if (!list_agents[id].size()) continue;
//            int id_task = list_agents[id].front();
//            Task *task = &tasks_total[id_task];
//            if (agents[id].loc != task->start->loc || timestep < task->release_time) {
//                list_agents[id] = list_agents[id];
//                // cout << " Agente " << agents[id].id << " comeca tarefa " << task->id << " vai da posicao loc " << agents[id].loc << " para coleta loc " << task->start->loc <<" dis: "<<Dis[agents[id].loc][task->start->loc]<<endl;
//                agents[id].next_ep = task->start;
//                ag_icbs.push_back(&agents[id]);
//                is_conflict[id] = false;
//            }
//        }
//
//
//        for (int i = 0; i < agents.size(); i++) {
//            if (!list_agents[i].size()) continue;
//            int id_task = list_agents[i].front();
//            Task *task = &tasks_total[id_task];
//            int id = i;
//            if (task->delivering == false && timestep >= task->release_time && agents[id].loc == task->start->loc) {
//                //   cout << " Agente " << agents[id].id << " da tarefa " << task->id << " terminou coleta na loc " << agents[id].loc << " no tempo " << timestep << " esta indo para entrega " << task->goal->loc << endl;
//                task->ag_arrive_start = timestep;
//                agents[id].task = task;
//                agents[id].next_ep = task->goal;
//                agents[id].delivering = true;
//                ag_icbs.push_back(&agents[id]);
//                deleteTask_id(task->id);
//                task->agent = &agents[id];
//                task->delivering = true;
//                is_conflict[id] = false;
//            }
//        }
//
//        if (!ag_icbs.empty()) //path finding
//        {
//            for (int i = 0; i < agents.size(); i++) {
//                if (!list_agents[i].size()) continue;
//                int id_task = list_agents[i].front();
//                Task *task = &tasks_total[id_task];
//                int id = i;
//                bool flag = true;
//                for (int j = 0; j < ag_icbs.size(); j++)
//                    if (ag_icbs[j]->id == id)
//                        flag = false;
//                if (flag) {
//                    is_conflict[id] = false;
//                    if (agents[id].next_ep == NULL) {
//                        //  cout<<" Agente "<<agents[id].id<<" com next_ep nulo comeca tarefa "<<task->id<<" e vai para posicao coleta "<<task->start->loc<<endl;
//                        agents[id].next_ep = task->start;
//                    }
//                    ag_icbs.push_back(&agents[id]);
//                }
//            }
//
//            vector<vector<int>> cons_paths;
//            for (int i = 0; i < agents.size(); i++)
//                if (is_conflict[i])
//                    cons_paths.push_back(agents[i].path);
//            if (!PathFinding(ag_icbs, cons_paths))
//                return;
//        }
//        if (!TestConstraints()) //test correctness
//        {
//            printf("TestConstraints ERROR\n");
//            while (1);
//        }
//    }
}


int Simulation::getShortTime() {

    int id = 0;
    int short_time = agents[0].finish_time;
    for (int i = 0; i < agents.size(); i++) {

        if (agents[i].finish_time < short_time) {
            short_time = agents[i].finish_time;
            id = agents[i].id;
        }
    }
    //  cout<<endl<<"Agent "<<id<<" no menor tempo "<<short_time;
    return short_time;
}

void Simulation::BFS() {
    for (int i = 0; i < my_map.size(); i++) {
        vector<int> D;
        D.resize(my_map.size(), INF);
        if (my_map[i]) {
            D[i] = 0;
            queue<int> Q;
            Q.push(i);
            while (!Q.empty()) {
                int u = Q.front();
                Q.pop();
                int offset[4] = {-1, 1, -col, col};
                for (int j = 0; j < 4; j++) {
                    int v = u + offset[j];
                    if (0 <= v && v < my_map.size() && abs(v % col - u % col) < 2 && my_map[v] && D[v] > D[u] + 1) {
                        D[v] = D[u] + 1;
                        Q.push(v);
                    }
                }
            }
        }
        Dis.push_back(D);
    }
    return;
}

int Simulation::mkspn() {

    return timestep - 1;
}

void Simulation::deleteTask_id(int id) {

    for (int i = 0; i < list_taskset.size(); i++) {
        if (list_taskset[i]->id == id) {
            list_taskset.erase(list_taskset.begin() + i);
            return;
        }
    }

}

bool Simulation::PathFinding(vector<Agent *> &ags, const vector<vector<int>> &cons_paths) {
    EgraphReader egr;
    constraint_strategy s;
    s = constraint_strategy::ICBS;

    std::clock_t start;
    double duration = 0;
    start = std::clock();
    for (int i = 0; i < ags.size(); i++) {
        //ags[i]->goal_loc = ags[i]->goal_loc;
        //   cout<<" Agente "<<ags[i]->id<<" vai para posicao "<<ags[i]->next_ep->loc<<endl;
        ags[i]->start_time = timestep;
        ags[i]->only_dummy = false;
        ags[i]->cost = CalcCost(ags[i]);
    }
    ICBSSearch icbs(my_map, ags, 1.0, egr, s, col, cons_paths, timestep);
    int res = icbs.runICBSSearch();
    if (res == 1) {
        //update
        for (unsigned int i = 0; i < ags.size(); i++) {
            //update searching path
            for (unsigned int j = 0; j < icbs.paths[i]->size(); j++) {
                ags[i]->path[timestep + j] = icbs.paths[i]->at(j).location;
            }
            //hold endpoint
            for (unsigned int j = icbs.paths[i]->size() + timestep; j < maxtime; j++) {
                ags[i]->path[j] = ags[i]->park_loc;
            }
            //update task
            if (ags[i]->delivering == true) {
                int t = ags[i]->task->ag_arrive_start;
                while (ags[i]->path[t] != ags[i]->next_ep->loc) t++;
                ags[i]->task->ag_arrive_goal = t; //timestep + icbs.paths[i]->size() - 1;
            }
            //agents[i]->task = NULL;
        }
        duration = (std::clock() - start) / (double) CLOCKS_PER_SEC;
        tempoTotalPathPlan = duration + tempoTotalPathPlan;
        return true;
    } else if (res == 0) {
        cout << res << endl;
        cout << "CBS fails, timestep: " << timestep << endl;
        timestep = 0;
        return false;
    } else if (res == 3) {
        cout << res << endl;
        cout << "AQUELE ERRO, timestep: " << timestep << endl;
        timestep = -1;
        return false;
    }
}

vector<int> Simulation::CalcCost(Agent *ag) {

    vector<int> cost;
    int tt = Dis[ag->park_loc][ag->goal_loc];
    cost.push_back(tt);
    return cost;
}

bool Simulation::TestConstraints() {
    for (int i = 0; i < agents.size(); i++) {
        for (int t = 0; t < maxtime; t++)
            if (my_map[agents[i].path[t]] == false) {
                printf("ILLEGAL POS! %d %d\n", i, agents[i].path[t]);
                while (1);
            }
    }
    for (unsigned int ag = 0; ag < agents.size(); ag++) {
        for (unsigned int i = ag + 1; i < agents.size(); i++) {
            for (unsigned int j = timestep + 1; j < maxtime; j++) {
                if (agents[ag].path[j] == agents[i].path[j]) {
                    cout << "Agent " << ag << " and " << i << " collide at location "
                         << agents[ag].path[j] << " at time " << j << endl;
                    for (int k = timestep; k < timestep + 50; k++)
                        printf("%d ", agents[ag].path[k]);
                    printf("\n");
                    for (int k = timestep; k < timestep + 50; k++)
                        printf("%d ", agents[i].path[k]);
                    printf("\n");
                    return false;
                } else if (agents[ag].path[j] == agents[i].path[j - 1]
                           && agents[ag].path[j - 1] == agents[i].path[j]) {
                    cout << "Agent " << ag << " and " << i << " collide at edge "
                         << agents[ag].path[j - 1] << "-" << agents[ag].path[j] << " at time " << j << endl;
                    for (int k = timestep; k < timestep + 50; k++)
                        printf("%d:%d ", k, agents[ag].path[k]);
                    printf("\n");
                    for (int k = timestep; k < timestep + 50; k++)
                        printf("%d:%d ", k, agents[i].path[k]);
                    printf("\n");
                    return false;
                }
            }
        }
    }
    return true;
}

void Simulation::runSearch(ScenarioLoader scen) {

    this->height = scen.height;
    width = scen.width;
    num_experimentos = scen.GetNumExperiments();
    int valor = 2;
    agents.resize(valor);
    col = width;
    my_map.resize(height*width);
    int t=0;
    for (int i = 0; i < height ; i++) {
        for (int j = 0; j < width; j++) {
            if(scen.map[t] == 1 || scen.map[t] == 2 || scen.map[t] == 4) {
                my_map[t] = true;
                t++;
            }
        }
    }
    BFS();

    for (int i = 0; i < valor; i++) {
        Experiment exp = scen.GetNthExperiment(i);
        agents[i].id = i;
        int start_x = exp.GetStartX();
        int start_y = exp.GetStartY();
        int goal_x = exp.GetGoalX();
        int goal_y = exp.GetGoalY();
        agents[i].park_loc = start_y * width + start_x;
        agents[i].loc = start_y * width + start_x;
        agents[i].goal_loc = goal_y * width + goal_x;
        for (int j = 0; j < 10000; j++) {
            agents[i].path.push_back(start_y * width + start_x);//stay still all the tiem
        }
    }

    vector<Agent *> ag_icbs;
    vector<bool> is_conflict(agents.size(), true);


    for (int i = 0; i < agents.size(); i++) {

        ag_icbs.push_back(&agents[i]);
    }


    vector<vector<int>> cons_paths;
    for (int i = 0; i < agents.size(); i++)
        if (is_conflict[i])
            cons_paths.push_back(agents[i].path);
    if (!PathFinding(ag_icbs, cons_paths))
        return;

    if (!TestConstraints()) //test correctness
    {
        printf("TestConstraints ERROR\n");
        while (1);
    }

}

Simulation::Simulation() {}
