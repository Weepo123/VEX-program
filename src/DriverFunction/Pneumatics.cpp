#include "vex.h"
#include "robot-config.h"
#include "DriverFunction/Pneumatics.h"

bool clamp = true;
bool redirect = true;
void goalClamp(){
    if(clamp){
        Clamp.open();
        clamp = false;
    }
    else if(!clamp){
        Clamp.close();
        clamp = true;
    }
}
void redirectRing(){
    if(redirect){
        Redirect.open();
        redirect = false;
    }
    else if (!redirect){
        Redirect.close();
        redirect = true;
    }
}