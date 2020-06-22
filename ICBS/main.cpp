#include <iostream>
#include "ScenarioLoader.h"
#include "Simulation.h"

int main() {
    srand(42);
    string scen = "empty-48-48";
    for(int i =1; i < 10; i++){
        string a = "/home/carol/Desktop/Path Planning/mapf-scen-random/scen-random/"+scen+"-random-" +to_string(i)+".scen";
        string b = "/home/carol/Desktop/Path Planning/mapf-map/"+scen+".map";
        const char *scen_file = a.c_str();
        const char *map_file =  b.c_str();

        cout<<"file : "<<scen_file<<endl;

        ScenarioLoader *scenLoad;
        scenLoad = new ScenarioLoader(scen_file);

        scenLoad->MapLoader(map_file);
        int num_experimentos = scenLoad->GetNumExperiments();

        Simulation *simu = new Simulation(*scenLoad);

      //  simu->addTasks(*scenLoad, num_experimentos);
       // simu->checkMap();
        for(int i = 1; i <num_experimentos; i++){
            cout<<endl<<"======== "<< i <<" ========";
            simu->addTasks(*scenLoad, i);
           // simu->printMap(*scenLoad);
            simu->runSearch(*scenLoad);
            simu->printPath();
            simu->info();
        }

    }
    return 0;
}