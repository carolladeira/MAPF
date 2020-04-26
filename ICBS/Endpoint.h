//
// Created by carol on 12/29/18.
//

#ifndef MAPD_ENDPOINT_H
#define MAPD_ENDPOINT_H

#include <vector>
#include <iostream>


using namespace std;

class Node{
public:
    int id;
    int loc;
    int row,col;
    int g_val, h_val;
    Node *parent;
    int timestep;
    bool in_openList;
    Node() {}
    Node(int loc, int g_val, Node *parent, int timestep) :loc(loc), g_val(g_val), timestep(timestep), parent(parent), h_val(0) {};
    Node(int loc, int g_val, int h_val, Node *parent, int timestep, bool in_openList):
    loc(loc), g_val(g_val), h_val(h_val), parent(parent), timestep(timestep), in_openList(in_openList) {};

    virtual ~Node() {};

protected:
    Node(int loc): loc(loc) {}
};


class Endpoint : public Node {
public:
    Endpoint(){};
    Endpoint(int loc) :Node(loc) {};

    vector<int> h_val;
    void setH_val(vector<bool> map, int loc);
    vector<int> map_h(vector<bool> map, int col);

    virtual ~Endpoint();


};


#endif //MAPD_ENDPOINT_H
