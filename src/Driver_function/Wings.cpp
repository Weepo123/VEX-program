#include "vex.h"
#include "robot-config.h"

using namespace vex;

bool Front_L = 1;
bool Front_R = 1;
bool Front = 1;
bool Back = 1;
bool hangg = 1;

void Front_wing_L(){
    if(Front_L){
        Front_wings_L.set(true);
        Front_L = 0;
    }
    else if(!Front_L){
        Front_wings_L.set(false);
        Front_L = 1;
    }
}

void Front_wing_R(){
    if(Front_R){
        Front_wings_R.set(true);
        Front_R = 0;
    }
    else if(!Front_R){
        Front_wings_R.set(false);
        Front_R = 1;
    }
}

void Front_wings(){
    if(Front){
        Front_wings_R.set(true);
        Front_wings_L.set(true);
        Front = 0;
    }
    else if(!Front){
        Front_wings_R.set(false);
        Front_wings_L.set(false);
        Front = 1;
    }
}

void Back_wings(){
    if(Back){
        Back_wings1.set(true);
        Back = 0;
    }
    else if(!Front){
        Back_wings1.set(false);
        Back = 1;
    }
}

void Hang(){
    if(hangg){
        hang.set(true);
        hangg = 0;
    }
    else if(!hangg){
        hang.set(false);
        hangg = 1;
    }
}