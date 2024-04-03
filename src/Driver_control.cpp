#include "vex.h"
#include "robot-config.h"
#include "Driver_Function/Wings.h"
#include "Driver_Function/Driver_class.h"
#include "Puncher_function/Puncher.h"
using namespace vex;
Driver_class Driver;

void Control(){    
    Driver.elevation.reset();
    resetPuncher();
    Controller.ButtonL1.pressed(Front_wings);
    Controller.ButtonR1.pressed(Back_wings);

    Controller.ButtonR2.pressed(Back_wing_R);
    Controller.ButtonL2.pressed(Back_wing_L);

    while(1){
        Driver.Intake_spin();
        Driver.Drivertain();
        Driver.Drivertain_Temperature();
        Driver.elevation_time();
        wait(10, msec);
    }
}