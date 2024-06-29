#include "vex.h"
#include "robot-config.h"
#include "Driver_Function/Wings.h"
#include "Driver_Function/Driver_class.h"
#include "Puncher_function/Puncher.h"
using namespace vex;
Driver_class Driver;

void Control() {
    Driver.elevation.reset(); // Reset elevation timer

    // Assigning functions to controller button presses
    Controller.ButtonL1.pressed(Front_wings);  // Press L1 for Front wings control
    Controller.ButtonL2.pressed(Back_wings);   // Press L2 for Back wings control
    Controller.ButtonDown.pressed(Hang);       // Press Down button for Hanging mechanism control
    Controller.ButtonY.pressed(Puncher);       // Press Y button for Puncher mechanism control

    // Main control loop
    while (true) {
        Driver.Intake_spin();           // Control the intake mechanism based on controller input
        Driver.Drivertain();            // Control the drivetrain based on controller input
        Driver.Drivertain_Temperature(); // Display motor temperatures on Brain screen and terminal
        Driver.elevation_time();        // Rumble the controller based on the elevation sensor value
        wait(10, msec);                 // Wait for 10 milliseconds before the next iteration
    }
}
