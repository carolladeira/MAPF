//
// Created by carol on 11/17/18.
//
#include <GL/glut.h>
#include <iostream>
#include "Scene.h"

#include <fstream>
#define DEBUG

using namespace std;

void Scene::drawScene(Nav *navMesh, Agente agente, int nWall, ScenarioLoader scen) {
    this->drawObstacles(scen);
    this->drawGraph(scen);
    this->drawAgent(std::vector<Agente>(), 0);
//    this->drawMeshes(navMesh, nWall);
    //this->drawPath(dstar);
    // this->drawPathDStar(dstar);
   // this->drawPathAtual(agente);


}

void Scene::drawPathAtual(Agente agente) {
    glPointSize((GLfloat) 15.0f);
    glColor3f(1.0, 0.0, 0.0); //red
    glBegin(GL_POINTS);
    glVertex2f(agente.atual.x, agente.atual.y);
    glEnd();
    glColor3f(0.0, 0.1, 0.6); //red
    glBegin(GL_POINTS);
    glVertex2f(agente.goal.x, agente.goal.y);
    glEnd();
#ifdef DEBUG
    std::cout <<agente.atual.x << ","<<agente.atual.y<< " --- "<<agente.goal.x << ","<<agente.goal.y<<" ---";
#endif
}

///desenha agente
void Scene::drawAgent(std::vector<Agente> agentes, int cont) {
    glPointSize((GLfloat) 15.0f);

    for (int i = 0; i < agentes.size(); i++) {
        glColor3f(0.0, 0.1, 0.6); //red
        glBegin(GL_POINTS);
        glVertex2f(agentes[i].path_goal[cont].x, agentes[i].path_goal[cont].y);
        glEnd();
        glColor3f(1.0, 0.0, 0.0); //red
        glBegin(GL_POINTS);
        glVertex2f(agentes[i].path[cont].x, agentes[i].path[cont].y);
        glEnd();

    }


}

///desenha obstaculos
void Scene::drawObstacles(ScenarioLoader scen) {
    int t = 0;

    glLineWidth((GLfloat) 1.0);
    glPointSize((GLfloat)8.0f);

    int x, y;

    for (int i = 0; i < scen.height; i++) {
        x = i;
        for (int j = 0; j < scen.width; j++) {
            y = j;
           // cout<<" "<<i<<","<<j<<endl;
            if (scen.mapa[i][j] == false) { //out of bounds @
                glColor3f(0.7,0.6,1);
                glBegin(GL_POINTS);
                glVertex2f(i, j);
                glEnd();
            }else{
                glColor3f(0.8,1.0,0.8 );
                glBegin(GL_POINTS);
                glVertex2f(i, j);
                glEnd();
            }
//            if (scen.map[t] == 1) { //out of bounds @
//                glColor3f(0.0, 0.0, 0.0);
//                glBegin(GL_POINTS);
//                glVertex2f(i, j);
//                glEnd();
//            }
//            if (scen.map[t] == 2) { //agua S
//                glColor3f(0.0, 0.0, 0.5);
//                glBegin(GL_POINTS);
//                glVertex2f(x, y);
//                glEnd();
//            }
//            if (scen.map[t] == 4) { //arvore T
//                glColor3f(0.0, 0.5, 0.0);
//                glBegin(GL_POINTS);
//                glVertex2f(x, y);
//                glEnd();
//                glBegin(GL_QUADS);
//                glVertex2f(x, y);
//                glVertex2f(x + 1, y);
//                glVertex2f(x + 1, y + 1);
//                glVertex2f(x, y + 1);
//                glEnd();
           // }
            t++;
        }
    }
}


void Scene::drawGraph(ScenarioLoader scen) {
    int t = 0;
    int x =0;
    int y=0;
    glLineWidth((GLfloat) 1.0);
    for (int i = 0; i < scen.width; i++) {
        for (int j = 0; j < scen.height; j++) {
//            glColor3f(0.0, 0.0, 1.0);
//            glBegin(GL_LINE_LOOP);
//            glVertex2f(i, j);
//            glVertex2f(i + scen.height, j);
//           // glVertex2f(i + scen.height, j + scen.width);
//           // glVertex2f(x, y + scen.width);
//            glEnd();
//            glColor3f(0.0, 0.3, 1.0);
//            glBegin(GL_LINE_LOOP);
//            glVertex2f(i, j);
//          //  glVertex2f(i + scen.height, j);
////            glVertex2f(i + scen.height, j + scen.width);
//           glVertex2f(i, j + scen.width);
//            glEnd();
            t++;
        }
    }
}

