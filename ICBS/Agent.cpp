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
    this->arrive_goal = 0;
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

int Agent::planPath(int start_loc, int goal_loc, int begin_time, vector<bool> my_map, int cost, int width){

   // std::cout << endl << " Agent " << id << " vai para a posicao " << goal->loc << " " ;
    this->my_map = my_map;
    this->width = width;
    clock_t start;
    double duration = 0;
    start = clock();

    int arrive_start = AStar(start_loc, goal_loc, begin_time, cost, false, false);
    if (arrive_start == -1) {
        cout << "ERRO: nao encontrou caminho para coleta" << endl;
        system("PAUSE");

    }
  //  std::cout << " no tempo "  << arrive_start;

//    for (int i = begin_time; i < path[id].size(); i++) {
//        this->path[id][i] = path[i];
//    }

    duration = (clock() - start) / (double) CLOCKS_PER_SEC;
    this->timepathPlan = duration + this->timepathPlan;

    return arrive_start;


}

int Agent::AStar(int start_loc, int goal, int begin_time, int cost, bool evaluate, bool coleta) {

    vector<Node *> open;
    int goal_location = goal;
    map<unsigned int, Node *> allNodes_table;

    Node *start = new Node(start_loc, 0, cost, NULL, begin_time, false);

    open.push_back(start);
    start->in_openList = true;
    allNodes_table.insert(make_pair(start_loc, start));

    while (!open.empty()) {
        sort(open.begin(), open.end(), meujeito);
        Node *curr = open.back();
        cout<<curr->loc<<endl;
        open.pop_back();
        curr->in_openList = false;

        if (curr->loc == goal_location) {
            bool hold = true;
            if (hold) {
                updatePath(curr);
                int t = curr->timestep;
                releaseClosedListNodes(allNodes_table);
                return t;
            }
        }
        int next_loc;
        int action[5] = {0, -width, 1, width, -1};
        for (int i = 0; i < 5; i++) {
            next_loc = curr->loc + action[i];
            int next_timestep = curr->timestep + 1;
            if (!isConstrained(curr->loc, next_loc, next_timestep, 0)) {
                int next_g_val = curr->g_val + 1;
                int next_h_val = dist(goal,next_loc);

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

int Agent::dist(int start, int goal){
    int start_y = start%width;
    int start_x = (start - start_y)/width;
    int goal_y = goal%width;
    int goal_x = (goal - goal_y)/width;
    int dist = std::abs(start_x - goal_x) + std::abs(start_y - goal_y);
}

bool Agent::isConstrained(int curr_loc, int next_loc, int next_timestep, int ag_hide) {

    if(next_loc > my_map.size() || next_loc <0) return true;
    if (my_map[next_loc] == false) return true; //colisao

//    for (int i = 0; i < token.path.size(); i++) {
//        if (i == id) continue;
//        else if (token.path[i][next_timestep] == next_loc) {
//               if(this->debug)cout << " VERTEX agente "<<i<<" no tempo "<<next_timestep<<" na posicao "<< next_loc << endl;
//            return true;
//        } else if (token.path[i][next_timestep] == curr_loc && token.path[i][next_timestep - 1] == next_loc) {
//             if(this->debug)  cout << " EDGE agente "<<i<<" no tempo "<<next_timestep<<" na posicao "<< curr_loc << endl;
//            return true;
//        }
//    }

    return false;
}

bool Agent::Move2EP(Token &token, bool constraint) {

//    //BFS algorithm, choose the first empty endpoint to go to
//    queue<Node *> Q;
//    map<unsigned int, Node *> allNodes_table; //key = g_val * map_size + loc
//    int action[5] = {0, 1, -1, col, -col};
//    Node *start = new Node(loc, 0, NULL, token.timestep);
//    allNodes_table.insert(make_pair(loc, start)); //g_val = 0 --> key = loc
//    Q.push(start);
//    while (!Q.empty()) {
//        Node *v = Q.front();
//        Q.pop();
//        if (v->timestep >= maxtime - 1) continue; // time limit
//        if (token.endpoints[v->loc]) // if v->loc is an endpoint
//        {
//            bool occupied = false;
//            // check whether v->loc can be held (no collision with other agents)
//            for (unsigned int t = v->timestep; t < maxtime && !occupied; t++) {
//                for (unsigned int ag = 0; ag < token.agents.size() && !occupied; ag++) {
//                    if (ag != id && token.path[ag][t] == v->loc) occupied = true;
//                }
//            }
//            // check whether it is a goal of a task
//            for (list<Task *>::iterator it = token.taskset.begin(); it != token.taskset.end() && !occupied; it++) {
//                if ((*it)->goal->loc == v->loc) occupied = true;
//            }
//            if (!occupied) {
//                updatePath(v);
//                arrive_goal = v->timestep;
//                //   if(this->debug) cout << "Agent " << id << " moves to endpoint " << v->loc << " no tempo " << finish_time << endl;
//                releaseClosedListNodes(allNodes_table);
//                return true;
//            }
//        }
//        for (int i = 0; i < 5; i++) // search its neighbor
//        {
//            if (constraint == true) {
//                map<unsigned int, Node *>::iterator it = allNodes_table.find(
//                        v->loc + action[i] + (v->g_val + 1) * row * col);
//                if (it == allNodes_table.end()) //undiscover
//                {  // add the newly generated node to hash table
//                    Node *u = new Node(v->loc + action[i], v->g_val + 1, v, v->timestep + 1);
//                    allNodes_table.insert(pair<unsigned int, Node *>(u->loc + u->g_val * row * col, u));
//                    Q.push(u);
//                }
//            } else {
//                if (!isConstrained(v->loc, v->loc + action[i], v->timestep + 1, token, id)) {
//                    //try to retrieve it from the hash table
//                    map<unsigned int, Node *>::iterator it = allNodes_table.find(
//                            v->loc + action[i] + (v->g_val + 1) * row * col);
//                    if (it == allNodes_table.end()) //undiscover
//                    {  // add the newly generated node to hash table
//                        Node *u = new Node(v->loc + action[i], v->g_val + 1, v, v->timestep + 1);
//                        allNodes_table.insert(pair<unsigned int, Node *>(u->loc + u->g_val * row * col, u));
//                        Q.push(u);
//                    }
//                }
//            }
//
//        }
//    }
//    return false;
}

void Agent::releaseClosedListNodes(map<unsigned int, Node *> &allNodes_table) {
    map<unsigned int, Node *>::iterator it;
    for (it = allNodes_table.begin(); it != allNodes_table.end(); it++) {
        delete ((*it).second);
    }
    allNodes_table.clear();
}
