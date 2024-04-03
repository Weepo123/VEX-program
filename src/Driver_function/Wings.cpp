#include "vex.h"
#include "robot-config.h"

using namespace vex;

bool Back_L = 1;
bool Back_R = 1;
bool Back = 1;
bool Front = 1;

void Back_wing_L(){
    if(Back_L){
        Back_wings_L.set(true);
        Back_L = 0;
    }
    else if(!Back_L){
        Back_wings_L.set(false);
        Back_L = 1;
    }
}

void Back_wing_R(){
    if(Back_R){
        Back_wings_R.set(true);
        Back_R = 0;
    }
    else if(!Back_R){
        Back_wings_R.set(false);
        Back_R = 1;
    }
}

void Back_wings(){
    if(Back){
        Back_wings_R.set(true);
        Back_wings_L.set(true);
        Back = 0;
    }
    else if(!Back){
        Back_wings_R.set(false);
        Back_wings_L.set(false);
        Back = 1;
    }
}

void Front_wings(){
    if(Front){
        Front_wings1.set(true);
        Front = 0;
    }
    else if(!Front){
        Front_wings1.set(false);
        Front = 1;
    }
}