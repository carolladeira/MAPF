//
// Created by carol on 12/17/18.
//

#include "Agent.h"

#define maxtime 10000


bool meujeito(const Node *i, const Node *j) { return i->g_val + i->h_val > j->g_val + j->h_val; }


void Agent::setAgent(int loc, int col, int row, int id) {
    this->park_loc = loc;
    this->loc = loc;
    this->col = col;
    this->row = row;
    this->id = id;
    this->finish_time = 0;
    for (int i = 0; i < maxtime; i++) {
        path.push_back(loc);//stay still all the tiem
    }

}

void Agent::updatePath(Node *goal) {
    for (int i = goal->timestep + 1; i < path.size(); i++) {
        this->path[i] = goal->loc;
    }
    Node *curr = goal;
    while (curr != NULL) {
        this->path[curr->timestep] = curr->loc;
        curr = curr->parent;
    }

}

int Agent::EvaluateTP(Token &token, int finish_time) {

    return -2;
}

int Agent::fastEvaluateTP(Token &token) {

    loc = path[finish_time];
    vector<int> dist_start;
    vector<int> dist_goal;
    Task *task = NULL;
    task = bestTask(token);
    if (task == NULL) {

        bool move = false;
        for (list<Task *>::iterator it = token.taskset.begin(); it != token.taskset.end(); it++) {
            if ((*it)->goal->loc == loc) {
                move = true;
                break;
            }
        }
        if (move) {
            token.timestep = finish_time;
            if (Move2EP(token, false)) {
                int i = 0;
                for (i = token.timestep; i < token.path[id].size(); i++) //agent move with package or waiting
                {
                    token.path[id][i] = path[i];
                }
                return 0;
            } else {
                debug = 1;
                //  cout << endl << endl << " Agent  " << id << "  impedindo tarefa no lugar " <<loc<< " no finish time " <<this->finish_time <<endl;
                bool t = Move2EP(token, true);
                // cout << endl << endl << "chamo com constraint true, agent  " << id << " finish time " <<this->finish_time <<endl;
                int i = 0;
                if (t) {
                    for (i = token.timestep; i < token.path[id].size(); i++) //agent move with package or waiting
                    {
                        token.path[id][i] = path[i];
                    }
                } else {
                    cout << "ERROOOOOOOOOOO na AG: agente nao conseguiu mover pra um endpoint sem restricoes" << endl;
                }
                return 0;
            }

        } else {
            //   cout << "Agent " << id << " nao pegou tarefa: -1" << endl;
            return -1;
        }

    } else {
        dist_start = task->start->h_val;
        int finish_time_pickup = dist_start[loc]; //posicao inicial ate pickup
        dist_goal = task->goal->h_val;
        int finish_time_delivery = dist_goal[task->start->loc]; //posicao de pickup ate goal
        int finish_time_final =
                finish_time + finish_time_pickup + finish_time_delivery; //posicao final do agente + entrega + coleta


        path[finish_time_final] = task->goal->loc;
        path[path.size() - 1] = task->goal->loc;
        token.path[id][finish_time_pickup + finish_time] = task->start->loc;
        token.path[id][token.path[id].size() - 1] = task->goal->loc;
        token.path[id][finish_time_final] = task->goal->loc;

//            std::cout << " Agent " << id << " posicao " << loc << " take task " << task->start->loc << " --> "
//                      << task->goal->loc;
//            std::cout << "	Timestep " << finish_time << "-->" << finish_time + finish_time_pickup << " | "
//                      << finish_time + finish_time_pickup << " -->" << finish_time_final << endl;



        token.timestep = finish_time_final;
        token.taskset.remove(task);
        this->finish_time = finish_time_final;

        return finish_time_final;
    }
    return 0;


}

bool Agent::TP(Token &token) {

    loc = path[token.timestep];

    Task *task = NULL;
    task = bestTask(token);
    if (task == NULL) {

        bool move = false;
        for (list<Task *>::iterator it = token.taskset.begin(); it != token.taskset.end(); it++) {
            if ((*it)->goal->loc == loc) {
                move = true;
                break;
            }
        }
        if (move) {
            if (Move2EP(token, false)) {
                int i = 0;
                for (i = token.timestep; i < token.path[id].size(); i++) //agent move with package or waiting
                {
                    token.path[id][i] = path[i];
                }
                return true;
            } else {
                cout << endl << endl << "ERRO " << id << " nao conseguiu se mover pra nennhum endpoint" << endl;
                debug = 1;
                bool t = Move2EP(token, false);
            }

        } else {
            std::cout << "Agent " << id << " wait at timestep " << token.timestep << endl;
            this->finish_time = token.timestep + 1;
            //  this->path[token.timestep + 1] = loc;
            return true;
        }
    } else {
        std::cout << endl << " Agent " << id << " take task " << task->start->id << " " << task->start->loc << " --> "
                  << task->goal->id << " " << task->goal->loc;

        int arrive_start = AStar(loc, task->start, token.timestep, token, false, true);
        if (arrive_start == -1) {
            cout << "ERRO: nao encontrou caminho para coleta" << endl;
            system("PAUSE");

        }
        std::cout << "	Timestep " << token.timestep << "-->" << arrive_start;

        int arrive_goal = AStar(task->start->loc, task->goal, arrive_start, token, false, false);

        if (arrive_goal == -1) {
            //    cout <<"encontrou caminho para entrego, arrumo chamando coleta e entrega de novo com evaluate true em coleta"<< endl;
            arrive_start = AStar(loc, task->start, token.timestep, token, true, true);
            arrive_goal = AStar(task->start->loc, task->goal, arrive_start, token, false, false);

        }
        std::cout << "-->" << arrive_goal;
        for (int i = token.timestep; i <= arrive_start; i++) {
            std::cout << "	" << path[i];
        }
        std::cout << " | ";

        for (int i = arrive_start + 1; i <= arrive_goal; i++) {
            std::cout << "	" << path[i];
        }

        bool hold = true;
        for (int i = arrive_goal + 1; i < maxtime; i++) {
            for (int j = 0; j < token.agents.size(); j++) {
                if (j != id && task->goal->loc == token.path[j][i]) {
//std::cout << endl << " Agent " << id << " colide no tempo " << i << " com agente " << j;
                    hold = false;
                    break;
                }
            }
        }
        if (!hold) {
            int t = AStar(task->goal->loc, task->goal, arrive_goal, token, true, false);
            // std::cout << endl << " Agent " << id << " arruma posicao no tempo  " << t;
            if (t == -1) {
                //cout << "Nao conseguiu arrumar a posicao, arrumo chamando entrega de novo com evalueate true" << endl;
                arrive_goal = AStar(task->start->loc, task->goal, arrive_start, token, true, false);
                // std::cout << endl << "-->" << arrive_goal;

//                for (int i = arrive_start + 1; i <= arrive_goal; i++) {
//                    std::cout << "	" << path[i];
//                }
            }
//            for (int i = arrive_goal; i <= t; i++) {
//                std::cout << "	" << path[i];
//            }
        }
        for (int i = token.timestep; i < token.path[id].size(); i++) {
            token.path[id][i] = path[i];
        }
        this->finish_time = arrive_goal;
        task->agent = this;
        task->ag_arrive_start = arrive_start;
        task->ag_arrive_goal = arrive_goal;
        token.taskset.remove(task);
        return true;
    }
    return false;

}

int Agent::AStar(int start_loc, Endpoint *goal, int begin_time, Token token, bool evaluate, bool coleta) {

    vector<Node *> open;
    int goal_location = goal->loc;
    map<unsigned int, Node *> allNodes_table;

    Node *start = new Node(start_loc, 0, goal->h_val[start_loc], NULL, begin_time, false);

    open.push_back(start);
    start->in_openList = true;
    allNodes_table.insert(make_pair(start_loc, start));

    while (!open.empty()) {
        sort(open.begin(), open.end(), meujeito);
        Node *curr = open.back();
        open.pop_back();
        curr->in_openList = false;

        if (curr->loc == goal_location) {
            bool hold = true;


            if(coleta){
                if(AStar(goal_location, ep_park_loc, curr->timestep,token, true, false) ==-1){
                    hold = false;
                }
            }

            if (evaluate) {


                for (int i = curr->timestep + 1; i < maxtime; i++) {
                    for (int j = 0; j < token.agents.size(); j++) {
                        if (j != id && curr->loc == token.path[j][i]) {
                            hold = false;
                            break;
                        }
                    }
                }

            }

            if (hold) {
                updatePath(curr);
                int t = curr->timestep;
                releaseClosedListNodes(allNodes_table);
                return t;
            }
        }
        int next_loc;
        int action[5] = {0, 1, -1, col, -col};
        for (int i = 0; i < 5; i++) {
            next_loc = curr->loc + action[i];
            int next_timestep = curr->timestep + 1;
            if (!isConstrained(curr->loc, next_loc, next_timestep, token, 0)) {
                int next_g_val = curr->g_val + 1;
                int next_h_val = goal->h_val[next_loc];

                Node *next = new Node(next_loc, next_g_val, next_h_val, curr, next_timestep, false);

                map<unsigned int, Node *>::iterator it = allNodes_table.find(next->loc + next->g_val * row * col);
                if (it == allNodes_table.end()) {
                    next->in_openList = true;
                    allNodes_table.insert(pair<unsigned int, Node *>(next->loc + next->g_val * row * col, next));
                    open.push_back(next);
                } else {
                    delete (next);
                }
            }
        }
    }
    releaseClosedListNodes(allNodes_table);
    return -1;

}

int Agent::planPath(int start_loc, Endpoint *goal, int begin_time, Token &token){

   // std::cout << endl << " Agent " << id << " vai para a posicao " << goal->loc << " " ;

    clock_t start;
    double duration = 0;
    start = clock();

    int arrive_start = AStar(start_loc, goal, begin_time, token, false, false);
    if (arrive_start == -1) {
        cout << "ERRO: nao encontrou caminho para coleta" << endl;
        system("PAUSE");

    }
  //  std::cout << " no tempo "  << arrive_start;

    int arrive_goal = AStar(goal->loc, this->ep_park_loc, arrive_start, token, true, false);
   // cout<<" Agente "<<id<<" na posicao "<<goal->loc<<" no tempo "<<arrive_start<<" para casa "<<ep_park_loc->loc<<" no tempo "<<arrive_goal<<endl;

    if (arrive_goal == -1) {
       // cout << " Ag nao conseguiu voltar pra casa " << endl;
        //cout<<" Agente "<<id<<" na posicao "<<goal->loc<<" no tempo "<<arrive_start<<" para casa "<<ep_park_loc->loc<<endl;
        arrive_start = AStar(start_loc, goal, begin_time, token, false, true);
        if (arrive_start == -1) {
            cout << "ERRO: nao encontrou caminho para coleta" << endl;
            system("PAUSE");

        }
        arrive_goal = AStar(goal->loc, this->ep_park_loc, arrive_start, token, true, false);
        if (arrive_goal == -1) {
            cout << "ERRO: nao encontrou caminho para coleta" << endl;
            system("PAUSE");

        }

    }
    // std::cout << "-->" << arrive_goal;
    // for (int i = begin_time; i <= arrive_start; i++) {
    //     std::cout << "	" << path[i];
    // }
    // std::cout << " | ";

    // for (int i = arrive_start + 1; i <= arrive_goal; i++) {
    //     std::cout << "	" << path[i];
    // }
    for (int i = begin_time; i < token.path[id].size(); i++) {
        token.path[id][i] = path[i];
    }

    duration = (clock() - start) / (double) CLOCKS_PER_SEC;
    this->timepathPlan = duration + this->timepathPlan;

    return arrive_start;


}


int Agent::AStar_sCol(int start_loc, Endpoint *goal, int begin_time, Token token, bool evaluate, bool coleta) {

    clock_t inicio;
    double duration = 0;
    inicio = clock();

    vector<Node *> open;
    int goal_location = goal->loc;
    map<unsigned int, Node *> allNodes_table;
    

    Node *start = new Node(start_loc, 0, goal->h_val[start_loc], NULL, begin_time, false);

    open.push_back(start);
    start->in_openList = true;
    allNodes_table.insert(make_pair(start_loc, start));

    while (!open.empty()) {
        sort(open.begin(), open.end(), meujeito);
        Node *curr = open.back();
        open.pop_back();
        curr->in_openList = false;

        if (curr->loc == goal_location) {
            updatePath(curr);
            int t = curr->timestep;
            releaseClosedListNodes(allNodes_table);
            duration = (clock() - inicio) / (double) CLOCKS_PER_SEC;
            this->timepathPlan = duration + this->timepathPlan;
            return t;

        }
        int next_loc;
        int action[5] = {0, 1, -1, col, -col};
        for (int i = 0; i < 5; i++) {
            next_loc = curr->loc + action[i];
            int next_timestep = curr->timestep + 1;
            if (token.map[next_loc]) {
                int next_g_val = curr->g_val + 1;
                int next_h_val = goal->h_val[next_loc];

                Node *next = new Node(next_loc, next_g_val, next_h_val, curr, next_timestep, false);

                map<unsigned int, Node *>::iterator it = allNodes_table.find(next->loc + next->g_val * row * col);
                if (it == allNodes_table.end()) {
                    next->in_openList = true;
                    allNodes_table.insert(pair<unsigned int, Node *>(next->loc + next->g_val * row * col, next));
                    open.push_back(next);
                } else {
                    delete (next);
                }
            }
        }
    }
    releaseClosedListNodes(allNodes_table);
    return -1;

}

Task *Agent::bestTask(Token token) {
    vector<bool> hold(col * row, false);
    for (int i = 0; i < token.path.size(); i++) {
        if (i != id) {
            int loc_f = token.path[i][token.path[i].size() - 1];
            hold[loc_f] = true;
        }
    }

    Task *task = NULL;
    list<Task *>::iterator n;
    for (list<Task *>::iterator it = token.taskset.begin(); it != token.taskset.end(); it++) {
        bool p = hold[(*it)->start->loc];
        if (hold[(*it)->start->loc] == true || hold[(*it)->goal->loc] == true) continue;
        if (task == NULL) task = *it;
        unsigned int t = task->start->h_val[loc];
        unsigned int m = (*it)->start->h_val[loc];
        if ((*it)->start->h_val[loc] < task->start->h_val[loc]) {
            task = *it;
        }
    }
    return task;
}

bool Agent::isConstrained(int curr_loc, int next_loc, int next_timestep, Token token, int ag_hide) {

    if (!token.map[next_loc]) return true;

    for (int i = 0; i < token.path.size(); i++) {
        if (i == id) continue;
        else if (token.path[i][next_timestep] == next_loc) {
               if(this->debug)cout << " VERTEX agente "<<i<<" no tempo "<<next_timestep<<" na posicao "<< next_loc << endl;
            return true;
        } else if (token.path[i][next_timestep] == curr_loc && token.path[i][next_timestep - 1] == next_loc) {
             if(this->debug)  cout << " EDGE agente "<<i<<" no tempo "<<next_timestep<<" na posicao "<< curr_loc << endl;
            return true;
        }
    }

    return false;
}

bool Agent::Move2EP(Token &token, bool constraint) {

    //BFS algorithm, choose the first empty endpoint to go to
    queue<Node *> Q;
    map<unsigned int, Node *> allNodes_table; //key = g_val * map_size + loc
    int action[5] = {0, 1, -1, col, -col};
    Node *start = new Node(loc, 0, NULL, token.timestep);
    allNodes_table.insert(make_pair(loc, start)); //g_val = 0 --> key = loc
    Q.push(start);
    while (!Q.empty()) {
        Node *v = Q.front();
        Q.pop();
        if (v->timestep >= maxtime - 1) continue; // time limit
        if (token.endpoints[v->loc]) // if v->loc is an endpoint
        {
            bool occupied = false;
            // check whether v->loc can be held (no collision with other agents)
            for (unsigned int t = v->timestep; t < maxtime && !occupied; t++) {
                for (unsigned int ag = 0; ag < token.agents.size() && !occupied; ag++) {
                    if (ag != id && token.path[ag][t] == v->loc) occupied = true;
                }
            }
            // check whether it is a goal of a task
            for (list<Task *>::iterator it = token.taskset.begin(); it != token.taskset.end() && !occupied; it++) {
                if ((*it)->goal->loc == v->loc) occupied = true;
            }
            if (!occupied) {
                updatePath(v);
                finish_time = v->timestep;
                //   if(this->debug) cout << "Agent " << id << " moves to endpoint " << v->loc << " no tempo " << finish_time << endl;
                releaseClosedListNodes(allNodes_table);
                return true;
            }
        }
        for (int i = 0; i < 5; i++) // search its neighbor
        {
            if (constraint == true) {
                map<unsigned int, Node *>::iterator it = allNodes_table.find(
                        v->loc + action[i] + (v->g_val + 1) * row * col);
                if (it == allNodes_table.end()) //undiscover
                {  // add the newly generated node to hash table
                    Node *u = new Node(v->loc + action[i], v->g_val + 1, v, v->timestep + 1);
                    allNodes_table.insert(pair<unsigned int, Node *>(u->loc + u->g_val * row * col, u));
                    Q.push(u);
                }
            } else {
                if (!isConstrained(v->loc, v->loc + action[i], v->timestep + 1, token, id)) {
                    //try to retrieve it from the hash table
                    map<unsigned int, Node *>::iterator it = allNodes_table.find(
                            v->loc + action[i] + (v->g_val + 1) * row * col);
                    if (it == allNodes_table.end()) //undiscover
                    {  // add the newly generated node to hash table
                        Node *u = new Node(v->loc + action[i], v->g_val + 1, v, v->timestep + 1);
                        allNodes_table.insert(pair<unsigned int, Node *>(u->loc + u->g_val * row * col, u));
                        Q.push(u);
                    }
                }
            }

        }
    }
    return false;
}

void Agent::releaseClosedListNodes(map<unsigned int, Node *> &allNodes_table) {
    map<unsigned int, Node *>::iterator it;
    for (it = allNodes_table.begin(); it != allNodes_table.end(); it++) {
        delete ((*it).second);
    }
    allNodes_table.clear();
}
