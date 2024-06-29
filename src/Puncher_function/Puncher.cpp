#include "vex.h"
#include "robot-config.h"

bool puncher = 1;
void Puncher(){
    if(puncher){
        puncher_motor.spin(fwd, 12, volt);
        puncher = 0;
    }
    else if(!puncher){
        puncher_motor.stop(brake);
        puncher = 1;
    }
}