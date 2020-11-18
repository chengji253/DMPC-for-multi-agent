//
// Created by liuhan253 on 2020/8/15.
//
#include "formation.h"

void Formation::InputInitialData(double x1[10],double y1[10],double z1[10],
                                 double x2[10],double y2[10],double z2[10],
                                 double x3[10],double y3[10],double z3[10],
                                 double x4[10],double y4[10],double z4[10])
{   //数据的初始化
    TransferData(x1,this->delta_x );
    TransferData(y1,this->delta_y );
    TransferData(z1,this->delta_z );

    TransferData(x2,this->p_x );
    TransferData(y2,this->p_y );
    TransferData(z2,this->p_z );

    TransferData(x3,this->V_x );
    TransferData(y3,this->V_y );
    TransferData(z3,this->V_z );

    TransferData(x4,this->u_x );
    TransferData(y4,this->u_y );
    TransferData(z4,this->u_z );
}

void printVectorD( vector<double> vector1){
    for(int i=0 ;i<vector1.size();i++)
        cout<<vector1[i]<<" ";
    cout<<endl;
}

void Formation::TransferData(double array[] , vector<double> &delta)
{
    for(int i = 0;i<(10);i++)
    {
        delta.push_back(array[i]);
    }
}

void Formation::Consensus()
{
    for(int i = 1;i<10;i++) {
        for (int j = 1; j < 10; j++) {
            u_x[i] = u_x[0] - alpha * (((p_x[i] - delta_x[i]) - p_x[0]) +
                                       bata * (V_x[i] - V_x[0])) -
                     A[i][j]  * ((p_x[i] - p_x[j]) - (delta_x[i] - delta_x[j]) + bata * (V_x[i] - V_x[j]));

            //u_x[i] = u_x[1] - alpha * (((p_x[i] - delta_x[i]) - p_x[1]) +
            //                         bata * (V_x[i] - V_x[1])) -
            //     A[i][j]  * ((p_x[i] - p_x[j]) - (delta_x[i] - delta_x[j]) + bata * (V_x[i] - V_x[j]));

            u_y[i] = u_y[0] - alpha * (((p_y[i] - delta_y[i]) - p_y[0]) +
                                       bata * (V_y[i] - V_y[0])) -
                     A[i][j] * ((p_y[i] - p_y[j]) - (delta_y[i] - delta_y[j]) + bata * (V_y[i] - V_y[j]));

            u_z[i] = u_z[0] - alpha * (((p_z[i] - delta_z[i]) - p_z[0]) +
                                       bata * (V_z[i] - V_z[0])) -
                    A[i][j] * ((p_z[i] - p_z[j]) - (delta_z[i] - delta_z[j]) + bata * (V_z[i] - V_z[j]));
            //u_z[i] = u_z[2] - alpha * (((p_z[i] - delta_z[i]) - p_z[2]) +
            //                         bata * (V_z[i] - V_z[2])) -
            //      A[i][j] * ((p_z[i] - p_z[j]) - (delta_z[i] - delta_z[j]) + bata * (V_z[i] - V_z[j]));

        }
    }
}

void Formation::forwardLeaderState()
{
    int i = 0;
    V_x[i] = V_x[i] + dt*u_x[i];
    V_y[i] = V_y[i] + dt*u_y[i];
    V_z[i] = V_z[i] + dt*u_z[i];
    p_x[i] = p_x[i] + dt*V_x[i];
    p_y[i] = p_y[i] + dt*V_y[i];
    p_z[i] = p_z[i] + dt*V_z[i];
}


void Formation::forwardVelocity()
{
    for(int i=1;i<agentSum;i++)  //从1开始不算leader
    {
        V_x[i] = V_x[i] + dt*u_x[i];
        V_y[i] = V_y[i] + dt*u_y[i];
        V_z[i] = V_z[i] + dt*u_z[i];
    }
}

void Formation::forwardPosition()
{
    for(int i=1;i<agentSum;i++)
    {
        p_x[i] = p_x[i] + dt*V_x[i];
        p_y[i] = p_y[i] + dt*V_y[i];
        p_z[i] = p_z[i] + dt*V_z[i];
    }
}

void Formation::mainLoop()
{
    //InputInitialData();
    for (int count=1;count < countMax;count++)
    {
        forwardLeaderState();
        Consensus();
        forwardVelocity();
        forwardPosition();
        storeStates();
    }
}

void Formation::storeStates()
{
    agentState S1;
    S1.p_x = this->p_x;
    S1.p_y = this->p_y;
    S1.p_z = this->p_z;
    S1.V_x = this->V_x;
    S1.V_y = this->V_y;
    S1.V_z = this->V_z;

    aState.push_back(S1);
}
