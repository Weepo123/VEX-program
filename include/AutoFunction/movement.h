#pragma once
#include "vex.h"
#include "robot-config.h"

#include <string>
#include <iostream>
#include <vector>

using namespace vex;

class Auto_class{
    public:
        Auto_class(){}

        /**
         * @brief Stops the motors with a specified braking mode.
         * 
         * This function stops both the left and right motors using the specified braking mode.
         * 
         * @param mode The braking mode to use (coast, brake, or hold).
         */
        void MotorStop(brakeType mode);

        /**
         * @brief Spins the motors with specified velocities.
         * 
         * This function sets the velocities for the left and right motors. It scales the velocities 
         * to ensure they are within the valid range for voltage control.
         * 
         * @param LeftVelocity The velocity for the left motor (any Velocity Units).
         * @param RightVelocity The velocity for the right motor (any Velocity Units).
         */
        void MotorSpin(double LeftVelocity, double RightVelocity);
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
        double AverageDifference(std::vector<double> vector1, std::vector<double> vector2);
        /**
         * @brief Moves the robot a specified distance in tiles with speed control and turning correction.
         * 
         * This function controls the movement of the robot to a target distance in tiles, 
         * adjusting speed dynamically and correcting for turning errors using a PID controller.
         * 
         * @param DistanceTiles The distance to move in tiles.
         * @param MaxSpeed The maximum speed limit (in RPM).
         * @param TargetAngle The target angle to turn (in degrees).
         * @param ProportionalGain The proportional gain for the turning PID controller.
         * @param TurnRatio The ratio of distance traveled to target distance where turning correction starts.
         */
        void MoveTile(float DistanceTiles, float MaxSpeed = 600.0, float TargetAngle = Inertial.rotation(), float ProportionalGain = 0.0, float TurnRatio = 0.0);
        /**
         * @brief Turns the robot by a specified rotation angle.
         * 
         * This function uses a PID controller to turn the robot by a given rotation angle (in degrees). 
         * It calculates the necessary velocities for the left and right motors based on the rotation center 
         * and adjusts the motor speeds using PID control to achieve precise turning.
         * 
         * @param Rotation The target rotation angle in degrees.
         * @param Velocity The maximum velocity for the rotation (in RPM).
         * @param RotationCenterCm The center of rotation in centimeters.
         * @param ErrorRange The acceptable error range for the rotation PID control.
         * @param Timeout The maximum time allowed for the operation (in milliseconds).
         */
        void Turn(float Target, float Velocity = 600.0, float RotationCenterCm = 0, float ErrorRange = 0.5, float Timeout = 3);

        /**
         * @brief Moves the robot a specified distance while also turning it by a specified angle.
         *
         * This function uses PID controllers to move the robot a certain distance (measured in tiles) and 
         * simultaneously turn it by a specified rotation angle. The function ensures the robot moves at 
         * controlled velocities and adjusts for errors in distance and rotation using encoder feedback.
         * 
         * @param DistanceTile The target distance to move in tiles (1 tile = 60.96 cm).
         * @param Rotation The target rotation angle in degrees.
         * @param MoveVelocity The maximum velocity for moving the robot (in RPM).
         * @param RotateVelocity The maximum velocity for turning the robot (in RPM).
         * @param RatioToTurn The ratio to control the turning rate.
         * @param ErrorRang The acceptable error range for distance PID control.
         * @param Timeout The maximum time allowed for the operation (in milliseconds).
         * */
        void MoveTurnTile(float DistanceTile, float Rotation = Inertial.rotation(), float MoveVelocity = 600.0, float RotateVelocity = 600.0, float RatioToTurn = 0, float ErrorRang = 1, float Timeout = 5);

        /**
         * @brief Controls wings or similar mechanisms based on direction and open/close command.
         * 
         * @param direction The direction of the wings to control ("Back" or "Front").
         * @param is_open Boolean flag indicating whether to open (true) or close (false) the wings.
         */
        void Wings(std::string direction, bool is_open);
        
    private:
        //Length of one tile in centimeters
        double TileLength = (60.96 / 1.0);
        //Perimeter of the wheels in centimeters
        double WheelsPerimeter = (1.0 / (M_PI * 8.255));
        //Circumference of the wheels in centimeters
        double WheelCircum = (2.0 * M_PI * (8.255 / 2));
        //Gear ratio of the robot
        double GearRatio = (60.0 / 48.0);
        //Full rotation angle in degrees
        double CompleteAngle = (360.0 / 1.0);
        //Length of the robot in an specified unit
        double RobotLength = (36);
        //Circumference related to encoder measurement
        double EncoderCircum = (2);
        //Flag indicating whether encoder is used for distance measurement
        bool UseEncoder = false;
};