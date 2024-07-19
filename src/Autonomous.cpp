#include "vex.h"
#include "robot-config.h"
#include "AutoFunction/movement.h"

#include <iostream>
#include <thread>

using namespace vex;

void Autonomous(){
    Auto_class Auto;
    Auto.MoveTurnTile(1);
}