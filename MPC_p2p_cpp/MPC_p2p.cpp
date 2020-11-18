//
// Created by liuhan253 on 2020/8/17.
//
#include "MPC_p2p.h"

void MPC::Initialization_matrix()
{
    U.setZero();
    x.setZero();
    A.setIdentity();
    A.block(0,3,3,3) = h*MatrixXd::Identity(3,3);
    B.block(0,0,3,3) = 0.5*h*h*MatrixXd::Identity(3,3);
    B.block(3,0,3,3) = h*MatrixXd::Identity(3,3);
    phi.block(0,0,3,3) = MatrixXd::Identity(3,3);
    getLambda();
    getDelta();
    getLambda_v();
    getA0();
    Q.setIdentity();
    Q = Q;                   //Trajectory error penalty
    R = R.setIdentity()*0.01;     //Control effort penalty
    S = S.setIdentity()*0.1;         //Input variation penalty
    getAbieq();
    X0.setZero();
    Pd.setZero();
    /*
    cout<<"---A-----"<<endl<<A<<endl;
    cout<<"---B-----"<<endl<<B<<endl;
    cout<<"---lambda-----"<<endl<<lambda<<endl;
    cout<<"---Q-----"<<endl<<Q<<endl;
    cout<<"---delta-----"<<endl<<delta<<endl;
    cout<<"---R-----"<<endl<<R<<endl;
    cout<<"---S-----"<<endl<<S<<endl;
    cout<<"---X0-----"<<endl<<X0<<endl;
    cout<<"---Pd-----"<<endl<<Pd<<endl;
     */
}


void MPC::getLambda()
{
    lambda = MatrixXd::Zero(3*K_step , 3*K_step);
    for(int i = 0;i<K_step;i++){
        for(int j = 0;j<K_step-i;j++)
        {
            lambda.block(3*j+3*i , 3*i , 3, 3) = getAk(j);
        }
    }
}

void MPC::getDelta()
{
    delta = MatrixXd::Identity(3*K_step , 3*K_step);
    for(int i = 0;i<K_step-1;i++)
    {
        delta.block(3*(i+1),3*i , 3,3) = -1*MatrixXd::Identity(3 , 3);
    }
}

void MPC::getLambda_v()
{
    lambda_v = MatrixXd::Zero(3*K_step , 3*K_step);
    for(int i = 0;i<K_step;i++){
        for(int j = 0;j<1+i;j++)
        {
            lambda_v.block(3*i , 3*j , 3, 3) = MatrixXd::Identity(3,3);
        }
    }
}

MatrixXd MPC::getAk(int i)
{
    Matrix<double , 6 , 6 > Ak;
    Ak = MatrixXd::Identity(6,6);
    while (i>0)
    {
        Ak = Ak*A;
        i = i-1;
    }
    Matrix<double , 3 , 3 > PAB;
    PAB = phi*Ak*B;
    return PAB;
}

void MPC::getA0()
{
    Matrix<double ,6 , 3*K_step> A0_;
    A0_ = MatrixXd::Zero(6 , 3*K_step);
    for(int i = 0;i<K_step;i++)
    {
        A0_.block(0,3*i,6,3) = getPhiA(i);
    }
    A0 = A0_.transpose();
}

MatrixXd MPC::getPhiA(int i)
{
    Matrix<double , 6 , 6 > Ak;
    Ak = MatrixXd::Identity(6,6);
    while (i>=0)
    {
        Ak = Ak*A;
        i = i-1;
    }
    Matrix<double , 6 , 3 > result;
    result = (phi*Ak).transpose();
    return result;
}

void MPC::getAbieq()
{
    Aieq.block(0, 0,3*K_step,3*K_step) = -1*MatrixXd::Identity(3*K_step,3*K_step);
    Aieq.block(0, 3*K_step,3*K_step,3*K_step);
    bieq = 10*bieq.setOnes();
}

void MPC::getUstar(Matrix<double ,3 , 1> u_kt_1)
{
    Ustar.setZero();
    Ustar.block(0,0,3,1) = u_kt_1;
}

void MPC::getHf()
{
    getUstar(u_kt_1); //Ustar will change as time goes by
    H = lambda.transpose()*Q*lambda + delta.transpose()*S*delta+R;
    f = -1*(Pd.transpose()*Q*lambda-(A0*X0).transpose()*Q*lambda)-(Ustar.transpose()*S*delta);
    /*
    //cout<<"H"<<endl<<H<<endl;
    cout<<"Ustar"<<endl<<Ustar<<endl;
    cout<<"f"<<endl<<f<<endl;
    cout<<"lambda"<<endl<<lambda<<endl;
    cout<<"Pd"<<endl<<Pd<<endl;
    cout<<"A0"<<endl<<A0<<endl;
    //cout<<"Q"<<endl<<Q<<endl;
    cout<<"delta"<<endl<<delta<<endl;
    cout<<"(A0*X0).transpose()"<<endl<<(A0*X0).transpose()<<endl;
    cout<<"X0"<<endl<<X0<<endl;
    //cout<<"R"<<endl<<R<<endl;
*/
}

void MPC::agentQuadprog()
{
    getHf();
    agentQppara();
/**
 * The general quadratic formulation recognized by OOQP is as follows:
 *				min 1/2 x'Qx + c' x subject to
 *					Ax = b,
 *					d <= Cx <= f,
 *					l <= x <= u
 *
 **/
// nx is the number of primal variables. It is the length of the input vectors
// c, xlow, ixlow, xupp, ixupp, x, gamma, and phi.
     //int nx   = 3*K_step;

// objconst is to be added to the objective value
    const double objconst = 0;

// c is the linear term in the objective function, a vector of length nx
    //double    c[]  = { 1.5,  -2 };

//xlow, ixlow are the lower bounds on x. These contain the information in the
//lower bounding vector l in the formulation given above.
//If there is a bound on element k of x (that is, lk > -1), then xlow[k] should
//be set to the value of lk and ixlow[k] should be set to one.
//Otherwise, element k of both arrays should be set to zero.
    //double  xlow[] = {  0,   0 };
    //char   ixlow[] = {  1,   1 };

//xupp, ixupp are the upper bounds on x, that is, the information in the vector
//u in the formulation given above. These should be defined in a similar fashion to
//xlow and ixlow.
    //double  xupp[] = { 20,   0 };
    //char   ixupp[] = {  1,   0 };

//irowQ, jcolQ, dQ hold the nnzQ lower triangular elements of the quadratic
//term of the objective function.
    //const int nnzQ = 3;
    //int    irowQ[] = {  0,   1,   1 };
    //int    jcolQ[] = {  0,   0,   1 };
    //double    dQ[] = {  8,   2,  10 };

//bA contains the right-hand-side vector b for the equality constraints given
//in the above formulation. The integer parameter my defines the length of this vector.
    const int my   = 0;
    double bA[]     = {0};

//irowA, jcolA, dA are the nnzA nonzero elements of the matrix A of linear
//equality constraints.
    int nnzA = 0;
    int irowA[]    = {0};
    int jcolA[]    = {0};
    double dA[]   = {0};

/// The integer parameter mz defines the number of inequality constraints.
    const int mz   = 0;

// clow, iclow are the lower bounds of the inequality constraints.
    double clow[]  = {  0 };
    char  iclow[]  = {  0 };

// cupp, icupp are the upper bounds of the inequality constraints.
    double cupp[]  = { 0 };
    char  icupp[]  = { 0 };

//irowC, jcolC, dC are the nnzC nonzero elements of the matrix C of linear
//inequality constraints.
    int nnzC = 0;
    int   irowC[]  = {0};
    int   jcolC[]  = {0};
    double   dC[]  = {0};

    int ierr;

    /* x, y and z are vectors of Lagrange multipliers */

    /* double x[nx], gamma[nx], phi[nx];
     * gamma and phi contain the multipliers for the lower and upper bounds
     * x >= l and x <= u, respectively.
     */
    double x[3*K_step], gamma[3*K_step], phi[3*K_step], objval;

    /* double y[my];
     * y contains the multipliers for the equality constraints Ax = b.
     */
    double y[0];

    /* double z[mz], lambda[mz], pi[mz];
     * lambda and pi contain the multipliers for the inequality constraints
     * Cx >= d and Cx <= f, respectively.
     */
    double z[3*K_step], lambda[3*K_step], pi[3*K_step];

    qpsolvesp( c, nx, irowQ, nnzQ, jcolQ, dQ, xlow, ixlow, xupp, ixupp,
               irowA, nnzA, jcolA, dA, bA, my,
               irowC, nnzC, jcolC, dC,
               clow,  mz,   iclow, cupp, icupp,
               x, gamma, phi,
               y,
               z, lambda, pi, &objval, PRINT_LEVEL, &ierr );

    if( ierr != 0 ) {
        //fprintf( stderr, "Couldn't solve it.\n" );

    } else {
        int i;
        //printf(" Final Objective: %g\n\n", objval + objconst);
        //printf( "Solution:...\n" );
        for( i = 0; i < 3; i++ ) {
            //cout<<i<<" "<<x[i]<<endl;
            //printf( "x[%2d] = %g\n", i, x[i] );
        }
        u_kt_1(0,0) = x[0];
        u_kt_1(1,0) = x[1];
        u_kt_1(2,0) = x[2];
    }
}

void MPC::agentQppara()
{
    //cout<<"---------"<<f(0,1)<<endl;
    for(int i = 0;i<3*K_step;i++)
    {
        c[i] =f(0,i);
        xupp[i] = 10;
        ixupp[i] = 1;
        xlow[i] = -10;
        ixlow[i] = 1;
    }
   int k = 0;
    for(int i =0;i<3*K_step;i++){
       for(int j=0;j<i+1;j++)
       {
           irowQ[k] = i;
           jcolQ[k] = j;
           dQ[k] = H(i,j);
           k = k+1;
       }
   }
}

void MPC::mainLoop()
{
    Pd.setOnes(); Pd = Pd*50;
    Initialization_matrix();

    for(int i =0 ;i<timesteps;i++)
    {
        agentQuadprog();
        forwardState();

    }
}

void MPC::forwardState()
{
    X0[3]  = X0[3] + h*u_kt_1[0];
    X0[4]  = X0[4] + h*u_kt_1[1];
    X0[5]  = X0[5] + h*u_kt_1[2];

    X0[0] = X0[0] + X0[3]*h;
    X0[1] = X0[1] + X0[4]*h;
    X0[2] = X0[2] + X0[5]*h;
    /*
    V_x[i] = V_x[i] + dt*u_x[i];
    V_y[i] = V_y[i] + dt*u_y[i];
    V_z[i] = V_z[i] + dt*u_z[i];
    p_x[i] = p_x[i] + dt*V_x[i];
    p_y[i] = p_y[i] + dt*V_y[i];
    p_z[i] = p_z[i] + dt*V_z[i];
   */
}

void MPC::InputState(double x2,double y2,double z2,double x3,double y3,double z3)
{
    X0(0,0) = x2;
    X0(1,0) = y2;
    X0(2,0) = z2;
    X0(3,0) = x3;
    X0(4,0) = y3;
    X0(5,0) = z3;
}

void MPC::InputDestinationLeader(double p)
{
    Pd.setOnes(); Pd = Pd*p;
}

void MPC::InputDestinationFollower(list<agentState> K_step_followerState,int agent)
{
    for(int i = 0;i<K_step;i++)
    {
        Pd(3*i,0) = K_step_followerState.begin()->p_x[agent];
        Pd(3*i+1,0) = K_step_followerState.begin()->p_y[agent];
        Pd(3*i+2,0) = K_step_followerState.begin()->p_z[agent];
        K_step_followerState.pop_front();
    }
    /*
    if(agent == 1)
        cout<<"Pd ----- "<<Pd<<endl;
*/
}
