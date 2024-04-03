#include "vex.h"
#include "robot-config.h"
#include "Auto_function/Auto_class.h"

#include <string>
#include <iostream>

using namespace vex;

void Auto_class::Wings(std::string direction, bool is_open){

    if(direction == "Front"){
        Front_wings1.set(is_open);
    }
    else if(direction == "Back"){
        Back_wings_L.set(is_open);
        Back_wings_R.set(is_open);
    }
}