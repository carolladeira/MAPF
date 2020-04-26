//
// Created by carol on 4/25/20.
//


#include "MapLoader.h"

using namespace std;


MapLoader::MapLoader(const char *fname) {

    FILE *f;
    f = fopen(fname, "r");
    if (!f) {
        cout << "Map file not found." << endl;
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

//void MapLoader::printMap() {
//    for (int y = 0; y < height; y++) {
//        for (int x = 0; x < width; x++) {
//            cout << " " << mapInfo[y*width+x].loc << " [" << (y*width+x) << "] ";
//
//        }
//        cout << endl;
//    }
//}



