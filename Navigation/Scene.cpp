//
// Created by carol on 11/17/18.
//
#include <GL/glut.h>
#include <iostream>
#include "Scene.h"

#include <fstream>
//#define DEBUG

using namespace std;

void Scene::drawScene(Nav *navMesh, Agente agente, int nWall, ScenarioLoader scen) {
    this->drawObstacles(scen);
    this->drawGraph(scen);
    this->drawAgent(navMesh);
//    this->drawMeshes(navMesh, nWall);
    //this->drawPath(dstar);
    // this->drawPathDStar(dstar);
   // this->drawPathAtual(agente);


}

void Scene::drawPathAtual(Agente agente) {
    glPointSize((GLfloat) 5.0f);
    glColor3f(1.0, 0.0, 0.0); //red
    glBegin(GL_POINTS);
    glVertex2f(agente.atual.x, agente.atual.y);
    glEnd();
#ifdef DEBUG
    std::cout <<agente->atual.x << ","<<agente->atual.y<< " --- ";
#endif
}

///desenha agente
void Scene::drawAgent(Nav *nav) {
    glPointSize((GLfloat)10.0f);

    for (int i = 0; i < nav->n_agents; i++) {

        glColor3f(0.0, 0.0, 1.0); // green
        glBegin(GL_POINTS);
        glVertex2f(nav->agentes[i].start.x, nav->agentes[i].start.y);
        glEnd();
        glColor3f(1.0, 0.0, 0.2);
        glBegin(GL_POINTS);
        glVertex2f(nav->agentes[i].goal.x, nav->agentes[i].goal.y);
        glEnd();
    }


}

///desenha obstaculos
void Scene::drawObstacles(ScenarioLoader scen) {
    int t = 0;

    glLineWidth((GLfloat) 1.0);
    glPointSize((GLfloat)3.0f);

    int x, y;

    for (int i = 0; i < scen.height; i++) {
        x = i;
        for (int j = 0; j < scen.width; j++) {
            y = j;
            if (scen.map[t] == 1) { //out of bounds @
                glColor3f(0.0, 0.0, 0.0);
                glBegin(GL_POINTS);
                glVertex2f(x, y);
                glEnd();
            }
            if (scen.map[t] == 2) { //agua S
                glColor3f(0.0, 0.0, 0.5);
                glBegin(GL_POINTS);
                glVertex2f(x, y);
                glEnd();
            }
            if (scen.map[t] == 4) { //arvore T
                glColor3f(0.0, 0.5, 0.0);
                glBegin(GL_POINTS);
                glVertex2f(x, y);
                glEnd();
//                glBegin(GL_QUADS);
//                glVertex2f(x, y);
//                glVertex2f(x + 1, y);
//                glVertex2f(x + 1, y + 1);
//                glVertex2f(x, y + 1);
//                glEnd();
            }
            t++;
        }
    }
}


void Scene::drawGraph(ScenarioLoader scen) {
    int t = 0;

    glLineWidth((GLfloat) 1.0);
    for (int i = 0; i < scen.height; i++) {
        for (int j = 0; j < scen.width; j++) {
            glColor3f(0.0, 0.0, 1.0);
            glBegin(GL_LINE_LOOP);
            glVertex2f(i, j);
            glVertex2f(i + scen.height, j);
            glVertex2f(i + scen.height, j + scen.width);
            glVertex2f(i, j + scen.width);
            glEnd();
            t++;
        }
    }
}

