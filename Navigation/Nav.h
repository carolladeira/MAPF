//
// Created by carol on 11/17/18.
//

#ifndef UNTITLED_NAVMESH_H
#define UNTITLED_NAVMESH_H
#include <vector>

#include <math.h>
#include <time.h>
#include <stdlib.h>     /* srand, rand */
#include <vector>
#include <bits/stdc++.h>
#include "ScenarioLoader.h"


#define QUANTIDADE_CELULA 50

class Point{
public:
    float x;
    float y;

};

class Agente {
public:
    Point start;
    Point goal;
    Point atual;
    std::vector<Point>path;
    Agente();
};

class Nav {
public:
    int n_agents;
    std::vector<Agente>agentes;
    int tamanho;
    Nav(ScenarioLoader scen);
    virtual ~Nav(){};

};


#endif //UNTITLED_NAVMESH_H
