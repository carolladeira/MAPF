#include <iostream>
#include <stdlib.h>     /* srand, rand */
#include <GL/glut.h>
#include <time.h>
#include <math.h>
#include "ScenarioLoader.h"
#include "Scene.h"
#include "Nav.h"
#include <unistd.h>
#include <list>


#define num_paredes 20


#define DEBUG
///formula = j* 50 + i ---- j: coluna horizontal, i: coluna vertical
///formula inversa  =  id/ 50 = j (coluna horizontal), id%50 = i coluna vertical
ScenarioLoader *scenLoad;
Scene *scene;
Agente agente;
Nav *navMesh;
std::vector<Agente>agentes;
int contador = 0;
int height;
int width;
Point m;
Point ma;

void reshape(int, int);

void display(void);

void idle();

void init();

void mouse(int button, int state, int x, int y);


int main(int argc, char **argv) {
    // std:: cout << "   "<<__cplusplus ;
    // start = clock();

    //srand (time(NULL));
    srand(42);
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
    glutInitWindowSize(900, 700);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("Navigation");

    //acoes de callback
    glutDisplayFunc(display); //quando um pixel na janela precisa ser atualizado
    glutIdleFunc(idle); //funcao de callback chamada quando nada esta acontecendo
    glutReshapeFunc(reshape); //chamado quando a janela 'e redimensionada
    //  glutMotionFunc(mouseMotionFunc);
    glutMouseFunc(mouse);

    init();


    glutMainLoop();

    return 0;
}

void reshape(int w, int h) {
    glViewport(0, 0, (GLsizei) w, (GLsizei) h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glOrtho(0.0f, (float) w, (float) h, 0.0f, -1.0f, 1.0f);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void display(void) {

    glClear(GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-2, height + 2, -2, width + 2, -1.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    scene->drawScene(navMesh, agente, num_paredes, *scenLoad);
    scene->drawAgent(agentes, contador);
    scene->drawObstacles(*scenLoad);
    scene->drawPathAtual(agente);


    glFlush();
}


void idle() {
    //for( int i=0;  navMesh->agentes[0].path.size(); i++){
    usleep(1000000);
    std::cout << contador << std::endl;
    contador++;
    for(int i =0; i < agentes.size(); i ++){
        agentes[i].atual.x = agentes[i].path[contador].x;
        agentes[i].atual.y = agentes[i].path[contador].y;

    }
    m = navMesh->agentes[2].path[contador];
    agente.atual.x = m.x;
    agente.atual.y = m.y;
    ma = navMesh->agentes[2].path_goal[contador];
    agente.goal.x = ma.x;
    agente.goal.y = ma.y;
    glutPostRedisplay();



    //scene->drawPathAtual(navMesh->agentes[0].path[i]);
    // }


}

void init() {
    glClearColor(1.0, 1.0, 1.0, 0.0);
    const char *scen_file = "/home/carol/Desktop/Path Planning/mapf-scen-random/scen-random/Berlin_1_256.scen";
    const char *map_file = "/home/carol/Desktop/Path Planning/mapf-map/mapf-kiva-10.map";
    string cam = "/home/carol/Desktop/MAPD-MC/Resultados/10-10-MONTE_CARLO.path";


    scenLoad = new ScenarioLoader(scen_file);
    scenLoad->MapLoader(map_file);
    height = scenLoad->height;
    width = scenLoad->width;


    scene = new Scene();
    //agente = new Agente();
    navMesh = new Nav(*scenLoad);
    navMesh->read_file(cam, width);
    agente = navMesh->agentes[0];
    agentes = navMesh->agentes;
}


void clickCell(int x, int y) {
}

void mouse(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        clickCell(x, y);
    }
}
