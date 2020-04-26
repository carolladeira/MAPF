#include <iostream>
#include "ScenarioLoader.h"
#include "Simulation.h"

int main() {
    const char *scen_file = "/home/carol/Desktop/Path Planning/scenarios/dao/arena.map.scen";
    const char *map_file = "/home/carol/Desktop/Path Planning/maps/dao/arena.map";

    ScenarioLoader *scenLoad;
    scenLoad = new ScenarioLoader(scen_file);
    scenLoad->MapLoader(map_file);
    Simulation *simu = new Simulation();
    simu->runSearch(*scenLoad);

    return 0;
}