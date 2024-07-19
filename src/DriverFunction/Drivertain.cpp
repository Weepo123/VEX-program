#include "vex.h"
#include "robot-config.h"
#include "DriverFunction/DriverFunction.h"

using namespace vex;

void Driver_class::DrivertainSpin() {
    // Read joystick positions for axis 3 (forward/backward) and axis 1 (left/right)
    double Axis3 = Controller.Axis3.position(pct);
    double Axis1 = Controller.Axis1.position(pct);
    
    // Calculate left and right voltages based on joystick inputs
    double LeftVolt = Axis3 + Axis1;
    double RightVolt = Axis3 - Axis1;
    
    // Scale voltages to ensure they are within the valid range for motor control
    double Scale = 12.0 / fmax(12.0, fmax(fabs(LeftVolt), fabs(RightVolt)));
    LeftVolt *= Scale;
    RightVolt *= Scale;
    
    // Stop left motor if voltage is below a threshold, otherwise spin it forward
    if (fabs(LeftVolt) < 0.1) {
        LeftMotor.stop(brake);
    } else {
        LeftMotor.spin(forward, LeftVolt, volt);
    }
    
    // Stop right motor if voltage is below a threshold, otherwise spin it forward
    if (fabs(RightVolt) < 0.1) {
        RightMotor.stop(brake);
    } else {
        RightMotor.spin(forward, RightVolt, volt);
    }
}
