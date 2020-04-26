//
// Created by carol on 12/29/18.
//

#include "Endpoint.h"
#include <signal.h>
#include <queue>



void Endpoint:: setH_val(vector<bool> map, int loc){

    h_val = map_h(map, loc);
}

vector<int> Endpoint::map_h(vector<bool> map, int col) {


    vector<int> h (map.size() - 1); //cria uma lista com todos os pontos do mapa para setar o valor h para a localizacao dada
    queue<int>Q;
    int neighbor[4] = {1, -1, col, -col};
    vector<bool> status (map.size(), false);
    status[loc] = true;
    h[loc] = 0;
    Q.push(loc);
    while(!Q.empty()){
        int v =  Q.front();
        Q.pop();
        for (int i = 0; i< 4; i++){
            int u = v + neighbor[i];
            if(u<0){
                continue;
            }
            if(map[u]){
                if(!status[u]){
                    status[u] = true;
                    h[u] = h[v] + 1;
                    Q.push(u);
                }
            }
        }
    }

    return h;
}

Endpoint::~Endpoint() {

}
