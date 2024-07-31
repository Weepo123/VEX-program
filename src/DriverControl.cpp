#include "vex.h"
#include "robot-config.h"
#include "DriverFunction/DriverFunction.h"
#include "DriverFunction/Pneumatics.h"
using namespace vex;

Driver_class Driver;

void Control() {
    Driver.Elevation.reset(); // Reset elevation timer
    Controller.ButtonL1.pressed(GoalClamp);
    // Main control loop
    while (true) {
        Driver.IntakeSpin();           // Control the intake mechanism based on controller input
        Driver.DrivertainSpin();            // Control the drivetrain based on controller input
        Driver.DrivertainTemperature(); // Display motor temperatures on Brain screen and terminal
        Driver.ElevationTime();        // Rumble the controller based on the elevation sensor value
        wait(10, msec);                 // Wait for 10 milliseconds before the next iteration
    }
}
