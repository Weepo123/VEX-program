#include "vex.h"
#include "robot-config.h"
#include "Driver_function/Driver_class.h"

using namespace vex;

void Driver_class::Drivertain() {
    // Read joystick positions for axis 3 (forward/backward) and axis 1 (left/right)
    double axis3 = Controller.Axis3.position(pct);
    double axis1 = Controller.Axis1.position(pct);
    
    // Calculate left and right voltages based on joystick inputs
    double leftVolt = axis3 + axis1;
    double rightVolt = axis3 - axis1;
    
    // Scale voltages to ensure they are within the valid range for motor control
    double scale = 12.0 / fmax(12.0, fmax(fabs(leftVolt), fabs(rightVolt)));
    leftVolt *= scale;
    rightVolt *= scale;
    
    // Stop left motor if voltage is below a threshold, otherwise spin it forward
    if (fabs(leftVolt) < 0.1) {
        Left_motor.stop(brake);
    } else {
        Left_motor.spin(forward, leftVolt, volt);
    }
    
    // Stop right motor if voltage is below a threshold, otherwise spin it forward
    if (fabs(rightVolt) < 0.1) {
        Right_motor.stop(brake);
    } else {
        Right_motor.spin(forward, rightVolt, volt);
    }
}
