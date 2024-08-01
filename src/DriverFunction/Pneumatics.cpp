#include "vex.h"
#include "robot-config.h"
#include "DriverFunction/DriverFunction.h"

bool clamp = true;
bool pullIntake = true;
void GoalClamp(){
    if(clamp){
        Clamp.open();
        clamp = false;
    }
    else if(!clamp){
        Clamp.close();
        clamp = true;
    }
}

void PullUpIntake(){
    if(pullIntake){
        PullIntake.open();
        pullIntake = false;
    }
    else if(!pullIntake){
        PullIntake.close();
        pullIntake = true;
    }
}