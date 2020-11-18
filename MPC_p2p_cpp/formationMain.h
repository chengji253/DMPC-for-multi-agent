//
// Created by lh on 8/20/20.
//

#ifndef FORMATIONMAIN_H
#define FORMATIONMAIN_H

#include "formation.h"
#include "MPC_p2p.h"



void formation_main();
void writeToTxt(list<agentState> aState);
void writeToTxt_f(list<agentState> aState);
void dataTolist(list<agentState> &aState , list<agentState> &K_step_followerState);
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
);
void ChangeListEveryStep(list<agentState> &K_step_followerState , list<agentState> &aState);

void formation_main_ONE();
void StateToList_ONE(list<agentState> &followerState,
                     MPC &mpcfollower1);
void writeToTxt_f_ONE(list<agentState> aState);

#endif //FORMATIONMAIN_H
