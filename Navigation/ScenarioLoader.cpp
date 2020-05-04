//
// Created by carol on 4/26/20.
//

#include <fstream>

using std::ifstream;
using std::ofstream;

#include "ScenarioLoader.h"
#include <assert.h>
#include <iostream>

/**
 * Loads the experiments from the scenario file.
 */
ScenarioLoader::ScenarioLoader(const char *fname) {

    strncpy(scenName, fname, 1024);
    ifstream sfile(fname, std::ios::in);

    float ver;
    string first;
    sfile >> first;

    // Check if a version number is given
    if (first != "version") {
        ver = 0.0;
        sfile.seekg(0, std::ios::beg);
    } else {
        sfile >> ver;
    }

    int sizeX = 0, sizeY = 0;
    int bucket;
    string map;
    int xs, ys, xg, yg;
    double dist;

    // Read in & store experiments
    if (ver == 0.0) {
        while (sfile >> bucket >> map >> xs >> ys >> xg >> yg >> dist) {
            Experiment exp(xs, ys, xg, yg, bucket, dist, map);
            experiments.push_back(exp);
        }
    } else if (ver == 1.0) {
        while (sfile >> bucket >> map >> sizeX >> sizeY >> xs >> ys >> xg >> yg >> dist) {
            Experiment exp(xs, ys, xg, yg, sizeX, sizeY, bucket, dist, map);
            experiments.push_back(exp);
        }
    } else {
        printf("Invalid version number.\n");
        //assert(0);
    }
}

void ScenarioLoader::Save(const char *fname) {
//	strncpy(scenName, fname, 1024);
    ofstream ofile(fname);

    float ver = 1.0;
    ofile << "version " << ver << std::endl;


    for (unsigned int x = 0; x < experiments.size(); x++) {
        ofile << experiments[x].bucket << "\t" << experiments[x].map << "\t" << experiments[x].scaleX << "\t";
        ofile << experiments[x].scaleY << "\t" << experiments[x].startx << "\t" << experiments[x].starty << "\t";
        ofile << experiments[x].goalx << "\t" << experiments[x].goaly << "\t" << experiments[x].distance << std::endl;
    }
}

void ScenarioLoader::AddExperiment(Experiment which) {
    experiments.push_back(which);
}

void ScenarioLoader::MapLoader(const char *fname) {

    FILE *f;
    f = fopen(fname, "r");
    if (!f) {
        std::cout << "Point file not found." << std::endl;
        system("PAUSE");
        return;
    }
    fscanf(f, "type octile\nheight %d\nwidth %d\nmap\n", &height, &width);
    map.resize(height * width);
    mapInfo.resize(height * width);

    for (int y = 0; y < height; y++) {

        for (int x = 0; x < width; x++) {

            char what;
            fscanf(f, "%c", &what);
            switch (toupper(what))
            {

                case '@': //out of bounds
                    map[y*width+x] = 1;
                case 'O': //out of bounds
                    map[y*width+x] = 1;
                    break;
                case 'S': //swamp (passable from regular terrain)
                    map[y*width+x] = 0;
                    break;
                case 'W': //water (traversable, but not passable from terrain)
                    map[y*width+x] = 2;
                    break;
                case 'T': //trees (unpassable)
                    map[y*width+x] = 4;
                    break;
                default: //passable terrain (.,G)
                    map[y*width+x] = 0;
                    break;
            }
            mapInfo[y*width+x].x = x;
            mapInfo[y*width+x].y = y;
            mapInfo[y*width+x].loc = y*width+x;


        }
        fscanf(f, "\n");
    }
    fclose(f);
}
