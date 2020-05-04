#include <iostream>
#include "ScenarioLoader.h"
#include "Simulation.h"

int main() {
    srand(42);
    const char *scen_file = "/home/carol/Desktop/Path Planning/scenarios/dao/arena.map.scen";
    const char *map_file = "/home/carol/Desktop/Path Planning/maps/dao/arena.map";

    ScenarioLoader *scenLoad;
    scenLoad = new ScenarioLoader(scen_file);
    scenLoad->MapLoader(map_file);
    int num_experimentos = scenLoad->GetNumExperiments();

    Simulation *simu = new Simulation(*scenLoad);
    for(int i = 1; i < 5; i++){
        cout<<endl<<"======== "<< i <<" ========";
        simu->addTasks(*scenLoad, i);
        simu->printMap(*scenLoad);
        simu->runSearch(*scenLoad);
        //simu->printPath();
        simu->info();
    }


    return 0;
}