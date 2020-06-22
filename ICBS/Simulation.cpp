#include "Simulation.h"

#define INF 1000000000
#define maxtime 10000

void Simulation::BFS() {
//    for (int i = 0; i < my_map.size(); i++) {
//        vector<int> D;
//        D.resize(my_map.size(), INF);
//        if (my_map[i]) {
//            D[i] = 0;
//            queue<int> Q;
//            Q.push(i);
//            while (!Q.empty()) {
//                int u = Q.front();
//                Q.pop();
//                int offset[4] = {-1, 1, -col, col};
//                for (int j = 0; j < 4; j++) {
//                    int v = u + offset[j];
//                    if (0 <= v && v < my_map.size() && abs(v % col - u % col) < 2 && my_map[v] && D[v] > D[u] + 1) {
//                        D[v] = D[u] + 1;
//                        Q.push(v);
//                    }
//                }
//            }
//        }
//        Dis.push_back(D);
//    }
//    return;
}

bool Simulation::PathFinding(vector<Agent *> &ags, const vector<vector<int>> &cons_paths) {
    EgraphReader egr;
    constraint_strategy s;
    s = constraint_strategy::ICBS;

    std::clock_t start;
    double duration = 0;
    start = std::clock();
    for (int i = 0; i < ags.size(); i++) {
        ags[i]->start_time = timestep;
        ags[i]->only_dummy = false;
        ags[i]->cost = CalcCost(ags[i]);
    }
    ICBSSearch icbs(my_map, ags, 1.0, egr, s, col, cons_paths, timestep);
    int res = icbs.runICBSSearch();
    if (res == 1) {
        this->no_expanded = icbs.HL_num_expanded;
        this->no_generated = icbs.HL_num_generated;

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
            int t = timestep;
            while (ags[i]->path[t] != ags[i]->goal_loc) t++;
            ags[i]->arrive_goal = t; //timestep + icbs.paths[i]->size() - 1;
        }
        duration = (std::clock() - start) / (double) CLOCKS_PER_SEC;
        tempoTotalPathPlan = duration;
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
   // int tt = Dis[ag->park_loc][ag->goal_loc];
    int start = ag->park_loc;
    int goal = ag->goal_loc;
    int start_y = start%width;
    int start_x = (start - start_y)/width;
    int goal_y = goal%width;
    int goal_x = (goal - goal_y)/width;
    int dist = std::abs(start_x - goal_x) + std::abs(start_y - goal_y);

    cost.push_back(dist);
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

Simulation::Simulation(ScenarioLoader scen) {

    timestep = 0;
    this->height = scen.height;
    width = scen.width;
    num_experimentos = scen.GetNumExperiments();
    col = width;
    this->scenName = scen.GetScenarioName();

    my_map.resize(height * width);
    int t = 0;
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            if (scen.map[t] == 1 || scen.map[t] == 2 || scen.map[t] == 4) {
                my_map[t] = false;
            } else {
                my_map[t] = true;
            }
            t++;
        }
    }
  //  BFS();
}


void Simulation::addTasks(ScenarioLoader scen, int qt) {
    agents.clear();
    agents.resize(qt);
    Experiment exp = scen.GetNthExperiment(1);
    this->mapName = exp.GetMapName();
    //this->Dis.resize(height*width, vector<long>(height*width));
    int dist = 0;
    for (int i = 0; i < qt; i++) {
        Experiment exp = scen.GetNthExperiment(i);
        agents[i].id = i;
        int start_x = exp.GetStartX();
        int start_y = exp.GetStartY();
        int goal_x = exp.GetGoalX();
        int goal_y = exp.GetGoalY();
        dist = std::abs(start_x - goal_x) + std::abs(start_y - goal_y);
        agents[i].park_loc = start_x  + start_y * width;
        agents[i].loc = start_x + start_y * width ;
        agents[i].goal_loc = goal_x + goal_y * width ;
      //  bool t = my_map[agents[i].park_loc];
      //  bool ta = my_map[agents[i].goal_loc];

        for (int j = 0; j < 10000; j++) {
            agents[i].path.push_back(-1);//stay still all the tiem
        }
    }

 //   Agent *ag;
 //   ag = &agents[0];
  //  int cost = CalcCost(ag);
 //   planPath(int start_loc, int goal_loc, int begin_time, vector<bool> my_map, int cost, int width)
//    ag->planPath(ag->park_loc, ag->goal_loc, 0, my_map, dist, width);
}

void Simulation::printPath() {

    for(int i = 0; i < agents.size(); i++){
        cout<<endl<<" Agente "<<i<<" : ";
        for(int j = 0; j <= agents[i].arrive_goal; j++){
            cout<< agents[i].path[j]<<" ";
        }
        cout<<"  "<< agents[i].arrive_goal<<" ";
    }

}

void Simulation::info() {

    string res_file = "/home/carol/Desktop/Path Planning/ICBS/Resultados/icbs";
    ofstream fout(res_file, ios::app);
    if (!fout) cout << "ERR0 - salvar arquivo";
    int mkspn = 0, soc =0;
    for(int i = 0; i < agents.size(); i++){
        soc += agents[i].arrive_goal;
        if(mkspn < agents[i].arrive_goal){
            mkspn = agents[i].arrive_goal;
        }
    }
    fout << agents.size() << "," << mapName << "," << scenName << "," << mkspn << "," <<soc<<","<< no_generated << ","<<no_expanded <<","<<this->tempoPathPlan()<< endl;
    fout.close();

    cout<<endl<<"qtd: "<<agents.size()<<" |map name: "<<mapName<<" |scen name: "<<scenName<<" |mkspn: "<<mkspn<<" |soc: "<<soc<<" |no gerados: "<<no_generated<<" |no expandido: "<<no_expanded<<" |TempoPathPlan: "<<this->tempoPathPlan() ;
}
void Simulation::printMap(ScenarioLoader scen) {
    char *mapChar = new char[height * width];
    for (int i = 0; i < height * width; i++) {
        if (scen.map[i] == 1)
            mapChar[i] = '*';
        else if (scen.map[i] == 2)
            mapChar[i] = 'W';
        else if (scen.map[i] == 4)
            mapChar[i] = 'T';
        else if (scen.map[i] == 0)
            mapChar[i] = '.';
    }

    for (int t = 0; t < agents.size(); t++) {
        int start = agents[t].park_loc;
        int goal = agents[t].goal_loc;
        cout << start << " -> " << goal << endl;
        mapChar[start] = 'S';
        mapChar[goal] = 'G';

    }


    std::cout << "MAP:"<<endl;
    int t = 0;
    for (int i = 0; i < height; i++) {
        std::cout <<i<< " ";
        for (int j = 0; j < width; j++) {
            std::cout << mapChar[t];
            t++;
        }
        cout<<endl;
    }
    std::cout << std::endl;
    delete[] mapChar;
}

void Simulation::checkMap(){
    for ( int ag = 0; ag < agents.size(); ag++) {
        if(!my_map[agents[ag].park_loc]) cout<<"A posicao inicial de um agente "<<ag<<" esta em posicao que nao pode (parede)"<<endl;
        if(!my_map[agents[ag].goal_loc]) cout<<"A posicao final de um agente "<<ag<<" esta em posicao que nao pode (parede)"<<endl;
        for (int i = ag + 1; i < agents.size(); i++) {
            if(agents[i].park_loc == agents[ag].park_loc) cout<<"Dois agentes "<<ag<<", "<<i<<" tem a mesma posicao inicial"<<endl;
            if(agents[i].goal_loc == agents[ag].goal_loc) cout<<"Dois agentes "<<ag<<", "<<i<<" tem a mesma posicao final"<<endl; //n tem problema
            if(agents[i].park_loc == agents[ag].goal_loc) cout<<"A posicao inicial de um agente "<<ag<<" eh igual a posicao final de outro agente"<<i<<endl; //n tem problema

        }
    }
}