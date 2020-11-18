//
// Created by lh on 8/20/20.
//

#include "formationMain.h"

void formation_main()
{
    //三角形
    double x1[] = {0 ,-30 ,-60 ,-90 ,-45 ,0 ,45 ,90 ,60 ,30};
     double y1[] = {0 ,-51.9 ,-103.8 ,-155.7 ,-155.7 ,-155.7 ,-155.7, -155.7, -103.8 ,-51.9};
     double z1[] = {0 ,0, 0, 0, 0, 0, 0, 0, 0, 0};
    //double z1[] = {10 ,10, 10, 10, 10, 10, 10, 10, 10, 10};
    //一字形
    //double x1[] = {0 ,5 ,10 ,15 ,20 ,25 ,30 ,35 ,40 ,45};
    //double y1[] = {0 ,0, 0, 0, 0, 0, 0, 0, 0, 0};
    //double z1[] = {0 ,0, 0, 0, 0, 0, 0, 0, 0, 0};

    //position
    double x2[] = {-25,-70,-50,-20,-10,10,-10,20,20,10};
    double y2[] = {9,-1.2,4,1,10,1.5,2,8,4,10};
    double z2[] = {150,140,100,160,175,145,180,168,135,165};
    //velcoity
    double x3[] = {20,10,10,10,8,12,13,9,12,10};
    double y3[] = {20,9,11,10,12,9,10,12,10,9};
    double z3[] = {0,12,10,10,10,11,9,10,9,10};
    //acceleration
    double x4[] = {0,0,0,0,0,0,0,0,0,0};
    double y4[] = {0,0,0,0,0,0,0,0,0,0};
    double z4[] = {0,0,0,0,0,0,0,0,0,0};

    Formation f1;
    f1.InputInitialData(x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4);

    MPC mpcLeader;
    mpcLeader.Initialization_matrix();
    double destination = 500;
    mpcLeader.InputState(x2[0],y2[0],z2[0],x3[0],y3[0],z3[0]);
    mpcLeader.InputDestinationLeader(destination);

    double dt = 0.08;
    f1.dt = dt;  mpcLeader.h = dt;

    cout<<"Now slove Consensus"<<endl;
    for (int count=1;count < f1.countMax;count++)
    {
        mpcLeader.agentQuadprog();
        mpcLeader.forwardState();
        f1.u_x[0] = mpcLeader.u_kt_1(0,0);  //MPC accel result delivered to formation
        f1.u_y[0] = mpcLeader.u_kt_1(1,0);
        f1.u_z[0] = mpcLeader.u_kt_1(2,0);

        f1.Consensus();
        f1.forwardLeaderState();
        f1.forwardVelocity();
        f1.forwardPosition();
        f1.storeStates();

        mpcLeader.X0(0,0) = f1.p_x[0];
        mpcLeader.X0(1,0) = f1.p_y[0];
        mpcLeader.X0(2,0) = f1.p_z[0];
        mpcLeader.X0(3,0) = f1.V_x[0];
        mpcLeader.X0(4,0) = f1.V_y[0];
        mpcLeader.X0(5,0) = f1.V_z[0];

        if(count == 200)
            mpcLeader.InputDestinationLeader(1000);

    }
    cout<<" Consensus is solved !"<<endl;
    writeToTxt(f1.aState);

    list<agentState> followerState;
    list<agentState> K_step_followerState;

    MPC mpcfollower1;
    mpcfollower1.Initialization_matrix();
    mpcfollower1.InputState(x2[1],y2[1],z2[1],x3[1],y3[1],z3[1]);

    MPC mpcfollower2;
    mpcfollower2.Initialization_matrix();
    mpcfollower2.InputState(x2[2],y2[2],z2[2],x3[2],y3[2],z3[2]);

    MPC mpcfollower3;
    mpcfollower3.Initialization_matrix();
    mpcfollower3.InputState(x2[3],y2[3],z2[3],x3[3],y3[3],z3[3]);

    MPC mpcfollower4;
    mpcfollower4.Initialization_matrix();
    mpcfollower4.InputState(x2[4],y2[4],z2[4],x3[4],y3[4],z3[4]);

    MPC mpcfollower5;
    mpcfollower5.Initialization_matrix();
    mpcfollower5.InputState(x2[5],y2[5],z2[5],x3[5],y3[5],z3[5]);

    MPC mpcfollower6;
    mpcfollower6.Initialization_matrix();
    mpcfollower6.InputState(x2[6],y2[6],z2[6],x3[6],y3[6],z3[6]);

    MPC mpcfollower7;
    mpcfollower7.Initialization_matrix();
    mpcfollower7.InputState(x2[7],y2[7],z2[7],x3[7],y3[7],z3[7]);

    MPC mpcfollower8;
    mpcfollower8.Initialization_matrix();
    mpcfollower8.InputState(x2[8],y2[8],z2[8],x3[8],y3[8],z3[8]);

    MPC mpcfollower9;
    mpcfollower9.Initialization_matrix();
    mpcfollower9.InputState(x2[9],y2[9],z2[9],x3[9],y3[9],z3[9]);
    //-------------------

    dataTolist(f1.aState , K_step_followerState);


    cout<<"Now slove MPC"<<endl;
    for (int count=1;count < f1.countMax;count++)
    {
        //cout<<"time --"<<count<<endl;
        mpcfollower1.InputDestinationFollower( K_step_followerState ,1);
        mpcfollower2.InputDestinationFollower( K_step_followerState ,2);
        mpcfollower3.InputDestinationFollower( K_step_followerState ,3);
        mpcfollower4.InputDestinationFollower( K_step_followerState ,4);
        mpcfollower5.InputDestinationFollower( K_step_followerState ,5);
        mpcfollower6.InputDestinationFollower( K_step_followerState ,6);
        mpcfollower7.InputDestinationFollower( K_step_followerState ,7);
        mpcfollower8.InputDestinationFollower( K_step_followerState ,8);
        mpcfollower9.InputDestinationFollower( K_step_followerState ,9);

        mpcfollower1.agentQuadprog();
        mpcfollower2.agentQuadprog();
        mpcfollower3.agentQuadprog();
        mpcfollower4.agentQuadprog();
        mpcfollower5.agentQuadprog();
        mpcfollower6.agentQuadprog();
        mpcfollower7.agentQuadprog();
        mpcfollower8.agentQuadprog();
        mpcfollower9.agentQuadprog();

        mpcfollower1.forwardState();
        mpcfollower2.forwardState();
        mpcfollower3.forwardState();
        mpcfollower4.forwardState();
        mpcfollower5.forwardState();
        mpcfollower6.forwardState();
        mpcfollower7.forwardState();
        mpcfollower8.forwardState();
        mpcfollower9.forwardState();

        StateToList(followerState,
                    mpcfollower1,
                    mpcfollower2,
                    mpcfollower3,
                    mpcfollower4,
                    mpcfollower5,
                    mpcfollower6,
                    mpcfollower7,
                    mpcfollower8,
                    mpcfollower9);

        ChangeListEveryStep(K_step_followerState,f1.aState);
        if(f1.aState.size()<=1)
            break;
    }
    cout<<"MPC is solved ! "<<endl;
    writeToTxt_f( followerState);
}

void writeToTxt(list<agentState> aState)
{
    ofstream fout;
    fout.open("date.txt"); //具体可以改
    for(auto it = aState.begin() ; it!=aState.end();it++){
        for(int i=0;i<10;i++){
            fout<<it->p_x[i]<<" ";
            fout<<it->p_y[i]<<" ";
            fout<<it->p_z[i]<<" ";
            //fout<<it->V_x[i]<<" ";
            //fout<<it->V_y[i]<<" ";
            //fout<<it->V_z[i]<<" ";

        }
        fout<<endl;
    }
    fout.close();
}

void writeToTxt_f(list<agentState> aState)
{
    ofstream fout;
    fout.open("datefollower.txt"); //具体可以改
    for(auto it = aState.begin() ; it!=aState.end();it++){
        for(int i=0;i<9;i++){
            fout<<it->p_x[i]<<" ";
            fout<<it->p_y[i]<<" ";
            fout<<it->p_z[i]<<" ";
            //fout<<it->V_x[i]<<" ";
            //fout<<it->V_y[i]<<" ";
            //fout<<it->V_z[i]<<" ";

        }
        fout<<endl;
    }
    fout.close();
}



void dataTolist(list<agentState> &aState , list<agentState> &K_step_followerState)
{

    for(int i=0;i<K_step;i++)
    {
        agentState s1;

        s1.p_x = aState.begin()->p_x;
        s1.p_y = aState.begin()->p_y;
        s1.p_z = aState.begin()->p_z;
        s1.V_x = aState.begin()->V_x;
        s1.V_y = aState.begin()->V_y;
        s1.V_z = aState.begin()->V_z;  //copy

        K_step_followerState.push_back(s1);
        if(!aState.empty())
            aState.pop_front();
        else
            cout<<"data --aState is empty"<<endl;
    }
}

void ChangeListEveryStep(list<agentState> &K_step_followerState , list<agentState> &aState)
{
    agentState s1;

    s1.p_x = aState.begin()->p_x;
    s1.p_y = aState.begin()->p_y;
    s1.p_z = aState.begin()->p_z;
    s1.V_x = aState.begin()->V_x;
    s1.V_y = aState.begin()->V_y;
    s1.V_z = aState.begin()->V_z;  //copy

    K_step_followerState.push_back(s1);
    if(!aState.empty())
        aState.pop_front();
    else
        cout<<"change --aState is empty"<<endl;

    if(!K_step_followerState.empty())
        K_step_followerState.pop_front();
    else
        cout<<"change --K follower is empty"<<endl;


}


void StateToList(list<agentState> &followerState,
                 MPC &mpcfollower1,
                 MPC &mpcfollower2,
                 MPC &mpcfollower3,
                 MPC &mpcfollower4,
                 MPC &mpcfollower5,
                 MPC &mpcfollower6,
                 MPC &mpcfollower7,
                 MPC &mpcfollower8,
                 MPC &mpcfollower9
                 )
{
    agentState a1;
    a1.p_x.push_back(mpcfollower1.X0(0,0));
    a1.p_x.push_back(mpcfollower2.X0(0,0));
    a1.p_x.push_back(mpcfollower3.X0(0,0));
    a1.p_x.push_back(mpcfollower4.X0(0,0));
    a1.p_x.push_back(mpcfollower5.X0(0,0));
    a1.p_x.push_back(mpcfollower6.X0(0,0));
    a1.p_x.push_back(mpcfollower7.X0(0,0));
    a1.p_x.push_back(mpcfollower8.X0(0,0));
    a1.p_x.push_back(mpcfollower9.X0(0,0));

    a1.p_y.push_back(mpcfollower1.X0(1,0));
    a1.p_y.push_back(mpcfollower2.X0(1,0));
    a1.p_y.push_back(mpcfollower3.X0(1,0));
    a1.p_y.push_back(mpcfollower4.X0(1,0));
    a1.p_y.push_back(mpcfollower5.X0(1,0));
    a1.p_y.push_back(mpcfollower6.X0(1,0));
    a1.p_y.push_back(mpcfollower7.X0(1,0));
    a1.p_y.push_back(mpcfollower8.X0(1,0));
    a1.p_y.push_back(mpcfollower9.X0(1,0));

    a1.p_z.push_back(mpcfollower1.X0(2,0));
    a1.p_z.push_back(mpcfollower2.X0(2,0));
    a1.p_z.push_back(mpcfollower3.X0(2,0));
    a1.p_z.push_back(mpcfollower4.X0(2,0));
    a1.p_z.push_back(mpcfollower5.X0(2,0));
    a1.p_z.push_back(mpcfollower6.X0(2,0));
    a1.p_z.push_back(mpcfollower7.X0(2,0));
    a1.p_z.push_back(mpcfollower8.X0(2,0));
    a1.p_z.push_back(mpcfollower9.X0(2,0));

    a1.V_x.push_back(mpcfollower1.X0(3,0));
    a1.V_x.push_back(mpcfollower2.X0(3,0));
    a1.V_x.push_back(mpcfollower3.X0(3,0));
    a1.V_x.push_back(mpcfollower4.X0(3,0));
    a1.V_x.push_back(mpcfollower5.X0(3,0));
    a1.V_x.push_back(mpcfollower6.X0(3,0));
    a1.V_x.push_back(mpcfollower7.X0(3,0));
    a1.V_x.push_back(mpcfollower8.X0(3,0));
    a1.V_x.push_back(mpcfollower9.X0(3,0));

    a1.V_y.push_back(mpcfollower1.X0(4,0));
    a1.V_y.push_back(mpcfollower2.X0(4,0));
    a1.V_y.push_back(mpcfollower3.X0(4,0));
    a1.V_y.push_back(mpcfollower4.X0(4,0));
    a1.V_y.push_back(mpcfollower5.X0(4,0));
    a1.V_y.push_back(mpcfollower6.X0(4,0));
    a1.V_y.push_back(mpcfollower7.X0(4,0));
    a1.V_y.push_back(mpcfollower8.X0(4,0));
    a1.V_y.push_back(mpcfollower9.X0(4,0));

    a1.V_z.push_back(mpcfollower1.X0(5,0));
    a1.V_z.push_back(mpcfollower2.X0(5,0));
    a1.V_z.push_back(mpcfollower3.X0(5,0));
    a1.V_z.push_back(mpcfollower4.X0(5,0));
    a1.V_z.push_back(mpcfollower5.X0(5,0));
    a1.V_z.push_back(mpcfollower6.X0(5,0));
    a1.V_z.push_back(mpcfollower7.X0(5,0));
    a1.V_z.push_back(mpcfollower8.X0(5,0));
    a1.V_z.push_back(mpcfollower9.X0(5,0));

    followerState.push_back(a1);
}


void formation_main_ONE()
{
    //三角形
    double x1[] = {0 ,-30 ,-60 ,-90 ,-45 ,0 ,45 ,90 ,60 ,30};
    double y1[] = {0 ,-51.9 ,-103.8 ,-155.7 ,-155.7 ,-155.7 ,-155.7, -155.7, -103.8 ,-51.9};
    double z1[] = {0 ,0, 0, 0, 0, 0, 0, 0, 0, 0};
    //double z1[] = {10 ,10, 10, 10, 10, 10, 10, 10, 10, 10};
    //一字形
    //double x1[] = {0 ,5 ,10 ,15 ,20 ,25 ,30 ,35 ,40 ,45};
    //double y1[] = {0 ,0, 0, 0, 0, 0, 0, 0, 0, 0};
    //double z1[] = {0 ,0, 0, 0, 0, 0, 0, 0, 0, 0};

    //position
    double x2[] = {-25,-70,-50,-20,-10,10,-10,20,20,10};
    double y2[] = {9,-1.2,4,1,10,1.5,2,8,4,10};
    double z2[] = {150,140,100,160,175,145,180,168,135,165};
    //velcoity
    double x3[] = {20,10,10,10,8,12,13,9,12,10};
    double y3[] = {20,9,11,10,12,9,10,12,10,9};
    double z3[] = {0,12,10,10,10,11,9,10,9,10};
    //acceleration
    double x4[] = {0,0,0,0,0,0,0,0,0,0};
    double y4[] = {0,0,0,0,0,0,0,0,0,0};
    double z4[] = {0,0,0,0,0,0,0,0,0,0};

    Formation f1;
    f1.InputInitialData(x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4);

    MPC mpcLeader;
    mpcLeader.Initialization_matrix();
    double destination = 500;
    mpcLeader.InputState(x2[0],y2[0],z2[0],x3[0],y3[0],z3[0]);
    mpcLeader.InputDestinationLeader(destination);

    double dt = 0.08;
    f1.dt = dt;  mpcLeader.h = dt;

    cout<<"Now slove Consensus"<<endl;
    for (int count=1;count < f1.countMax;count++)
    {
        mpcLeader.agentQuadprog();
        mpcLeader.forwardState();
        f1.u_x[0] = mpcLeader.u_kt_1(0,0);  //MPC accel result delivered to formation
        f1.u_y[0] = mpcLeader.u_kt_1(1,0);
        f1.u_z[0] = mpcLeader.u_kt_1(2,0);

        f1.Consensus();
        f1.forwardLeaderState();
        f1.forwardVelocity();
        f1.forwardPosition();
        f1.storeStates();

        mpcLeader.X0(0,0) = f1.p_x[0];
        mpcLeader.X0(1,0) = f1.p_y[0];
        mpcLeader.X0(2,0) = f1.p_z[0];
        mpcLeader.X0(3,0) = f1.V_x[0];
        mpcLeader.X0(4,0) = f1.V_y[0];
        mpcLeader.X0(5,0) = f1.V_z[0];

        if(count == 200)
            mpcLeader.InputDestinationLeader(1000);

    }
    cout<<" Consensus is solved !"<<endl;
    writeToTxt(f1.aState);

    list<agentState> followerState;
    list<agentState> K_step_followerState;

    MPC mpcfollower1;
    mpcfollower1.Initialization_matrix();
    mpcfollower1.InputState(x2[1],y2[1],z2[1],x3[1],y3[1],z3[1]);

    //-------------------

    dataTolist(f1.aState , K_step_followerState);


    cout<<"Now slove MPC"<<endl;
    for (int count=1;count < f1.countMax;count++)
    {
        cout<<"time --"<<count<<endl;
        mpcfollower1.InputDestinationFollower( K_step_followerState ,1);
        mpcfollower1.agentQuadprog();
        mpcfollower1.forwardState();
        cout<<"X0---"<<endl<<mpcfollower1.X0<<endl;

        StateToList_ONE(followerState,mpcfollower1);

        ChangeListEveryStep(K_step_followerState,f1.aState);
        if(f1.aState.size()<=1)
            break;
    }
    cout<<"MPC is solved ! "<<endl;
    writeToTxt_f_ONE( followerState);
}


void StateToList_ONE(list<agentState> &followerState, MPC &mpcfollower1)
{
    agentState a1;
    a1.p_x.push_back(mpcfollower1.X0(0,0));
    a1.p_y.push_back(mpcfollower1.X0(1,0));
    a1.p_z.push_back(mpcfollower1.X0(2,0));
    a1.V_x.push_back(mpcfollower1.X0(3,0));
    a1.V_y.push_back(mpcfollower1.X0(4,0));
    a1.V_z.push_back(mpcfollower1.X0(5,0));

    followerState.push_back(a1);
}

void writeToTxt_f_ONE(list<agentState> aState)
{
    ofstream fout;
    fout.open("datefollowerONE.txt"); //具体可以改
    for(auto it = aState.begin() ; it!=aState.end();it++){
        for(int i=0;i<1;i++){
            fout<<it->p_x[i]<<" ";
            fout<<it->p_y[i]<<" ";
            fout<<it->p_z[i]<<" ";
            //fout<<it->V_x[i]<<" ";
            //fout<<it->V_y[i]<<" ";
            //fout<<it->V_z[i]<<" ";

        }
        fout<<endl;
    }
    fout.close();
}
