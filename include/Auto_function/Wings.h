#include "vex.h"
#include "robot-config.h"
#include "Auto_function/Auto_class.h"

#include <string>
#include <iostream>

using namespace vex;

/*Decide use whether Front_wings or Back_wings then type in the true or false to control the wing.
Type in (Front, Back), (ture, false)*/
void Auto_class::Wings(std::string direction, bool is_open);