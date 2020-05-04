//
// Created by carol on 11/17/18.
//

#ifndef UNTITLED_SCENE_H
#define UNTITLED_SCENE_H
#include "Nav.h"
#include "ScenarioLoader.h"


class Scene {
public:
    Scene() {};

    ~Scene() {};


    void drawScene(Nav *navMesh, Agente *agente, int nWall, ScenarioLoader scen);
    void drawObstacles(ScenarioLoader scen);
    void drawAgent(Nav *navMesh);
    void drawMeshes(Nav *navMesh, int nWall);
    void drawGraph(ScenarioLoader scen);
  //  void drawPath(DStar *dStar);
   // void drawPathDStar(DStar *dstar);
    void drawPathAtual(Agente *agente);



};


#endif //UNTITLED_SCENE_H
