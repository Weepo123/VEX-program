#pragma once
#include "vex.h"
#include "robot-config.h"

#include <string>
#include <iostream>
#include <vector>

using namespace vex;

class autoClass {
    public:
        autoClass() {}

        /**
         * @brief Stops the motors with a specified braking mode.
         * 
         * This function stops both the left and right motors using the specified braking mode.
         * 
         * @param mode The braking mode to use (coast, brake, or hold).
         */
        void motorStop(brakeType mode);
        /**
         * @brief Spins the left and right motors at specified velocities.
         * 
         * This function sets the velocities of the left and right motors to the specified values.
         * 
         * @param leftVelocity The velocity for the left motor.
         * @param rightVelocity The velocity for the right motor.
         * @param units The units for the velocities (e.g., velocityUnits::rpm).
         */
        void motorSpin(double leftVelocity, double rightVelocity, velocityUnits units);
        /**
         * @brief Calculate the average difference between corresponding elements of two vectors.
         * 
         * This function takes two vectors of doubles and calculates the average difference 
         * between their corresponding elements. It uses the minimum size of the two vectors 
         * to avoid out-of-bounds errors. The total difference is accumulated and then divided 
         * by the number of elements to find the average difference.
         * 
         * @param vector1 The first vector of double values.
         * @param vector2 The second vector of double values.
         * @return double The average difference between corresponding elements of the two vectors.
         */
        double averageDifference(std::vector<double> vector1, std::vector<double> vector2);
        /**
         * @brief Turns the robot by a specified rotation angle.
         * 
         * This function uses a PID controller to turn the robot by a given rotation angle (in degrees). 
         * It calculates the necessary velocities for the left and right motors based on the rotation center 
         * and adjusts the motor speeds using PID control to achieve precise turning.
         * 
         * @param rotation The target rotation angle in degrees.
         * @param velocity The maximum velocity for the rotation (in RPM).
         * @param rotationCenterCm The center of rotation in centimeters.
         * @param timeout The maximum time allowed for the operation (in milliseconds).
         */
        void turn(float rotation, float velocity = 600.0, float rotationCenterCm = 0, float timeout = 1.5);
        /**
         * @brief Moves the robot a specified distance in tiles with speed control and turning correction.
         * 
         * This function controls the movement of the robot to a target distance in tiles, 
         * adjusting speed dynamically and correcting for turning errors using a PID controller.
         * 
         * @param distanceTiles The distance to move in tiles.
         * @param maxSpeed The maximum speed limit (in RPM).
         * @param targetAngle The target angle to turn (in degrees).
         * @param proportionalGain The proportional gain for the turning PID controller.
         * @param turnRatio The ratio of distance traveled to target distance where turning correction starts.
         * @param timeout The maximum time allowed for the operation (in milliseconds).
         */
        void moveTurnTileWithProfileCalculation(float distanceTiles, float maxSpeed = 600.0, float targetAngle = Inertial.rotation(), float proportionalGain = 0.0, float turnRatio = 0.0, float timeout = 5);
        /**
         * @brief Moves the robot a specified distance while also turning it by a specified angle.
         *
         * This function uses PID controllers to move the robot a certain distance (measured in tiles) and 
         * simultaneously turn it by a specified rotation angle. The function ensures the robot moves at 
         * controlled velocities and adjusts for errors in distance and rotation using encoder feedback.
         * 
         * @param distanceTile The target distance to move in cm (1 tile = 60.96 cm).
         * @param rotation The target rotation angle in degrees.
         * @param moveVelocity The maximum velocity for moving the robot (in RPM).
         * @param rotateVelocity The maximum velocity for turning the robot (in RPM).
         * @param ratioToTurn The ratio to control the turning rate.
         * @param timeout The maximum time allowed for the operation (in milliseconds).
         * */
        void moveTurnTileWithPID(float distanceTile, float rotation = Inertial.rotation(), float moveVelocity = 600.0, float rotateVelocity = 600.0, float ratioToTurn = 0.0, float timeout = 3);
        /**
         * @brief Moves the robot a specified distance while also turning it by a specified angle.
         *
         * This function uses PID controllers to move the robot a certain distance (measured in cm) and 
         * simultaneously turn it by a specified rotation angle. The function ensures the robot moves at 
         * controlled velocities and adjusts for errors in distance and rotation using encoder feedback.
         * 
         * @param distanceCm The target distance to move in cm.
         * @param rotation The target rotation angle in degrees.
         * @param moveVelocity The maximum velocity for moving the robot (in RPM).
         * @param rotateVelocity The maximum velocity for turning the robot (in RPM).
         * @param ratioToTurn The ratio to control the turning rate.
         * @param timeout The maximum time allowed for the operation (in milliseconds).
         * */
        void moveTurnCmWithPID(float distanceCm, float rotation = Inertial.rotation(), float moveVelocity = 600.0, float rotateVelocity = 600.0, float ratioToTurn = 0.0, float timeout = 5);
        /**
         * @brief Moves the robot a specified distance in tiles with speed control and turning correction.
         * 
         * This function controls the movement of the robot to a target distance in tiles, 
         * adjusting speed dynamically and correcting for turning errors using a PID controller.
         * 
         * @param distanceCm The distance to move in cm.
         * @param maxSpeed The maximum speed limit (in RPM).
         * @param targetAngle The target angle to turn (in degrees).
         * @param proportionalGain The proportional gain for the turning PID controller.
         * @param turnRatio The ratio of distance traveled to target distance where turning correction starts.
         * @param timeout The maximum time allowed for the operation (in milliseconds).
         */
        void moveTurnCmWithProfileCalculation(float distanceCm, float maxSpeed = 600.0, float targetAngle = Inertial.rotation(), float proportionalGain = 0.0, float turnRatio = 0.0, float timeout = 5);
        
    private:
        // Default move error range for moving 
        double defaultMoveErrorRange = 10;
        // Default rotate error range for turning
        double defaultRotateErrorRange = 2;
        // Length of one tile in centimeters
        double tileLength = 60.96;
        // Perimeter of the wheels in centimeters
        double wheelsPerimeter = (1.0 / (M_PI * 8.255));
        // Circumference of the wheels in centimeters
        double wheelCircumference = (2.0 * M_PI * (8.255 / 2));
        // Gear ratio of the robot
        double gearRatio = (60.0 / 48.0);
        // Full rotation angle in degrees
        double completeAngle = 360.0;
        // Length of the robot in a specified unit
        double robotLength = 36;
        // Circumference related to encoder measurement
        double encoderCircumference = 2;
        // Flag indicating whether encoder is used for distance measurement
        bool useEncoder = false;
};
