#include "vex.h"
#include "robot-config.h"
#include "Driver_function/Driver_class.h"

using namespace vex;

void Driver_class::Drivertain(){
    double axis3 = Controller.Axis3.position(pct);
    double axis1 = Controller.Axis1.position(pct);
    double leftVolt = axis3 - axis1;
    double rightVolt = axis3 + axis1;
    double scale = 12.0 / fmax(12.0, fmax(fabs(leftVolt), fabs(rightVolt)));
    leftVolt *= scale;
    rightVolt *= scale;
    if (fabs(leftVolt) < 0.1){
        Left_motor.stop(brake);
    } 
    else{
        Left_motor.spin(forward, leftVolt, volt);
    }
    if(fabs(rightVolt) < 0.1){
        Right_motor.stop(brake);
    }
    else{
        Right_motor.spin(forward, rightVolt, volt);
    }
}