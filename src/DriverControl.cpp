#include "vex.h"
#include "robot-config.h"
#include "DriverFunction/DriverFunction.h"
#include "DriverFunction/Pneumatics.h"
using namespace vex;

driverClass driver;

void control() {
    driver.elevation.reset(); // Reset elevation timer
    Controller.ButtonL1.pressed(goalClamp);
    Controller.ButtonY.pressed(redirectRing);

    // Main control loop
    while (true) {
        driver.intakeSpin();           // Control the intake mechanism based on controller input
        driver.drivertainSpin();            // Control the drivetrain based on controller input
        driver.drivertainTemperature(); // Display motor temperatures on Brain screen and terminal
        driver.elevationTime();        // Rumble the controller based on the elevation sensor value
    }
}
