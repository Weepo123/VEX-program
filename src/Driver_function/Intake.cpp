#include "vex.h"
#include "robot-config.h"
#include "Driver_function/Driver_class.h"

void Driver_class::Intake_spin(){
    if(Controller.ButtonR1.pressing()){
        Intake.spin(forward, 12, volt);
   }
   else if(Controller.ButtonR2.pressing()){
        Intake.spin(fwd, -12, volt);
    }
    else{
        Intake.stop(coast);
    }
}