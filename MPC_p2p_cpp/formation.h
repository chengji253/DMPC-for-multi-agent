//
// Created by liuhan253 on 2020/8/15.
//
#ifndef FORMATION_H
#define FORMATION_H

#include <iostream>
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <list>
#include <fstream>

using namespace Eigen;
using namespace std;

struct agentState{
    vector<double> p_x;
    vector<double> p_y;
    vector<double> p_z;

    vector<double> V_x;
    vector<double> V_y;
    vector<double> V_z;

    //vector<double> theta;
    //vector<double> psi;
    //vector<double> u_x;
    //vector<double> u_y;
    //vector<double> u_z;
};

void printVectorD( vector<double> vector1);

class Formation
{
public:
    void InputInitialData(double x1[10],double y1[10],double z1[10],
                          double x2[10],double y2[10],double z2[10],
                          double x3[10],double y3[10],double z3[10],
                          double x4[10],double y4[10],double z4[10]);
    void TransferData(double array[] , vector<double> &delta);
    void Consensus();
    void forwardVelocity();
    void forwardPosition();
    void forwardLeaderState();
    void mainLoop();
    void storeStates();

public:
    int agentSum = 10;
    double dt = 0.08;
    int countMax = 500;
    int bata = 2;
    int alpha = 1;
    int h_g=150;
    int A[10][10] = {
            {0, 1, 1, 1, 1, 1, 1, 1, 1, 1},
            {1, 0, 1, 1, 1, 1, 1, 1, 1, 1},
            {1, 1, 0, 1, 1, 1, 1, 1, 1, 1},
            {1, 1, 1, 0, 1, 1, 1, 1, 1, 1},
            {1, 1, 1, 1, 0, 1, 1, 1, 1, 1},
            {1, 1, 1, 1, 1, 0, 1, 1, 1, 1},
            {1, 1, 1 ,1 ,1 ,1 ,0 ,1 ,1, 1},
            {1, 1, 1, 1, 1, 1, 1, 0, 1, 1},
            {1, 1, 1, 1, 1, 1, 1, 1, 0, 1},
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 0}
    };
    //delta决定队形
    vector<double> delta_x;
    vector<double> delta_y;
    vector<double> delta_z;
    //初始化 位置、速度、加速度
    vector<double> p_x;
    vector<double> p_y;
    vector<double> p_z;

    vector<double> V_x;
    vector<double> V_y;
    vector<double> V_z;

    vector<double> u_x;
    vector<double> u_y;
    vector<double> u_z;

    list<agentState> aState;

};

#endif