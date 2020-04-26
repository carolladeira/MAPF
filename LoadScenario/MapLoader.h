//
// Created by carol on 4/25/20.
//

#ifndef LOADSCENARIO_MAPLOADER_H
#define LOADSCENARIO_MAPLOADER_H

#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <vector>

class MapLoader;

class Loc{


public:
    int x;
    int y;
    int loc;
};
class MapLoader{
public:
    int height;
    int width;
    std::vector<int> map;
    std::vector<Loc> mapInfo;
 //  vector
    MapLoader(const char *fname);
    void printMap();
   // char * mapToChar();
};


#endif //LOADSCENARIO_MAPLOADER_H
