#pragma once
#include "vex.h"
#include "robot-config.h"

using namespace vex;

class Driver_class{
    public:
        timer Elevation;
        
        Driver_class(){}

        /**
         * @brief Drives the robot based on joystick input for arcade drive control.
         * 
         * This function reads joystick positions for forward/backward movement (Axis3)
         * and left/right turning (Axis1). It calculates corresponding voltages for the
         * left and right motors, scales them to ensure they are within the valid range,
         * and controls the motor movements accordingly.
         */
        void DrivertainSpin();

        /**
         * @brief Prints the temperatures of all motors on the Brain screen and terminal.
         * 
         * This function retrieves and displays the current temperatures of all motors (L1, L2, L3, R1, R2, R3) 
         * in Celsius units. It prints these values on both the Brain screen and the terminal/console window.
         */
        void DrivertainTemperature();

        /**
         * @brief Checks the elevation value and triggers controller rumble if it equals 75.
         * 
         * This function checks the current elevation value. If the elevation value is 75,
         * it triggers the controller to rumble with a specific pattern ("- - -").
         */
        void ElevationTime();

        /**
         * @brief Controls the intake motor based on controller inputs.
         * 
         * This function checks the state of Controller ButtonR1 and ButtonR2 to determine 
         * whether to spin the intake motor forward at full voltage, backward at full voltage, 
         * or stop the intake motor (coast mode).
         */
        void IntakeSpin();

    private:
};