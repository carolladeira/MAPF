#include <iostream>
#include <string>

#include "ScenarioLoader.h"
#include "MapLoader.h"


int main() {
    std::cout << "Hello, World!" << std::endl;
    //  y*width+x


    const char *map_file = "/home/carol/Desktop/Path Planning/scenarios/dao/arena.map.scen";
    //const char *teste = "/home/carol/Desktop/Path Planning/teste.scen";
    const char *mapa = "/home/carol/Desktop/Path Planning/maps/dao/arena.map";

    ScenarioLoader *simu;
    MapLoader *maps;
    simu = new ScenarioLoader(map_file);
    maps = new MapLoader(mapa);
    //maps->printMap();
    int height = maps->height;
    int width = maps->width;

    char *mapChar = new char[height * width];
    Experiment mm = simu->GetNthExperiment(0);
    for (int i=0; i<height * width; i++) {
         if (maps->map[i] == 1)
            mapChar[i] = '@';
        else if(maps->map[i] == 2)
            mapChar[i] = 'W';
         else if(maps->map[i] == 4)
             mapChar[i] = 'T';
         else if(maps->map[i] == 0)
             mapChar[i] = '.';
    }

    int val = simu->GetNumExperiments();
    for (int t = 0; t < val; t++) {
        Experiment mm = simu->GetNthExperiment(t);
        int x = mm.GetStartX();
        int y = mm.GetStartY();
        mapChar[y*width+x] = 'S';
        int xx = mm.GetGoalX();
        int yy = mm.GetGoalY();
        mapChar[yy*width+xx] = 'G';

    }


    std::cout << "MAP:";
    int t=0;
    for (int i = 0; i < height ; i++) {
        std::cout << std::endl;
        for (int j = 0; j < width; j++) {
            std::cout << mapChar[t];
            t++;
        }
    }
    std::cout << std::endl;
    delete[] mapChar;

    return 0;
}