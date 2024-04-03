#include "vex.h"
#include "robot-config.h"
#include "Auto_function/Auto_class.h"
#include "Puncher_function/Puncher.h"

#include <iostream>
#include <thread>

using namespace vex;

void Autonomous(){
    Auto_class Auto;
    timer Auto_punch_time;
    resetPuncher();
    Auto.move_Tile(60, 300);
}