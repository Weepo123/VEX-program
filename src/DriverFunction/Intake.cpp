#include "vex.h"
#include "robot-config.h"
#include "DriverFunction/DriverFunction.h"

void Driver_class::IntakeSpin(){
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