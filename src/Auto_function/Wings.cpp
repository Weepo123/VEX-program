#include "vex.h"
#include "robot-config.h"
#include "Auto_function/Auto_class.h"

#include <string>
#include <iostream>

using namespace vex;

void Auto_class::Wings(std::string direction, bool is_open) {
    if (direction == "Back") {
        Back_wings1.set(is_open);  // Set the state of the back wings mechanism
    } else if (direction == "Front") {
        Front_wings_L.set(is_open);  // Set the state of the left front wings mechanism
        Front_wings_R.set(is_open);  // Set the state of the right front wings mechanism
    }
}
