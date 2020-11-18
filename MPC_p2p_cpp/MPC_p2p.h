//
// Created by liuhan253 on 2020/8/17.
//
#ifndef MPC_P2P_H
#define MPC_P2P_H

#include <iostream>
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <list>
#include <fstream>
#include "cstdio"
#include "QpGenData.h"
#include "QpGenVars.h"
#include "QpGenResiduals.h"
#include "GondzioSolver.h"
#include "QpGenSparseMa27.h"
#include "cQpGenSparse.h"
#include "formation.h"

#define PRINT_LEVEL 0
using namespace Eigen;
using namespace std;


const int  K_step = 15; //MPC steps

class  MPC
{
public:
    void Initialization_matrix();
    void getLambda();
    void getDelta();
    void getLambda_v();
    MatrixXd getAk(int i);
    void getA0();
    MatrixXd getPhiA(int i);
    void getAbieq();
    void getUstar(Matrix<double ,3 , 1> u_kt_1);
    void getHf();
    void agentQuadprog();
    void agentQppara();
    void mainLoop();
    void forwardState();
    void InputState(double x2,double y2,double z2,double x3,double y3,double z3);
    void InputDestinationLeader(double p);
    void InputDestinationFollower(list<agentState> K_step_followerState,int agent);


public:
    double h = 0.08;
    int timesteps = 100;

    Matrix<double, 3*K_step,1> U;
    Matrix<double , 6 , 1> x;
    Matrix<double , 6 , 6> A;
    Matrix<double , 6 , 3> B;
    Matrix<double ,3 , 6> phi;
    Matrix<double ,3*K_step , 3*K_step>  delta;
    Matrix<double ,3*K_step , 3*K_step>  lambda;
    Matrix<double ,3*K_step , 3*K_step>  lambda_v;
    Matrix<double ,3*K_step , 6>  A0;
    Matrix<double ,3*K_step , 3*K_step> Q;
    Matrix<double ,3*K_step , 3*K_step> R;
    Matrix<double ,3*K_step , 3*K_step> S;
    Matrix<double ,3*K_step , 6*K_step> Aieq;
    Matrix<double ,6*K_step , 1> bieq;
    Matrix<double ,3*K_step , 3*K_step> H;
    Matrix<double ,3 , 1> u_kt_1;       //acceleration result every timestep
    Matrix<double ,3*K_step , 1> Ustar;
    Matrix<double ,1, 3*K_step > f;
    Matrix<double ,3*K_step , 1> Pd;
    Matrix<double ,6 , 1> X0;

    //OOQP parameters
    int nx   = 3*K_step;
    double   c[3*K_step];
    int mz   = 3*K_step;
    double  xupp[3*K_step];
    char   ixupp[3*K_step];
    double  xlow[3*K_step];
    char   ixlow[3*K_step];
    int nnzQ = 3*K_step*(3*K_step+1)/2;
    int    irowQ[3*K_step*(3*K_step+1)/2];
    int    jcolQ[3*K_step*(3*K_step+1)/2];
    double    dQ[3*K_step*(3*K_step+1)/2];
    //--------
};

#endif //MPC_P2P_H
