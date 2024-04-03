#include "vex.h"
#include "robot-config.h"
#include "Driver_function/Driver_class.h"

using namespace vex;

void Driver_class::elevation_time(){
    if(this->elevation.value() == 75){
        Controller.rumble("- - -");
    }
}