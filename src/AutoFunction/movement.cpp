#include "vex.h"
#include "robot-config.h"
#include "AutoFunction/movement.h"
#include "AutoFunction/PID.h"
#include <vector>

using namespace vex;

void Auto_class::MotorStop(brakeType mode) {
    // Stop the left motor with the specified braking mode
    LeftMotor.stop(mode);
    
    // Stop the right motor with the specified braking mode
    RightMotor.stop(mode);
}


void Auto_class::MotorSpin(double LeftVelocity, double RightVelocity, velocityUnits units) {
    // Spin the left motor at the specified velocity
    LeftMotor.spin(fwd, LeftVelocity, units);
    // Spin the right motor at the specified velocity
    RightMotor.spin(fwd, RightVelocity, units);
}

double Auto_class::AverageDifference(std::vector<double> vector1, std::vector<double> vector2) {
    // Determine the size of the vectors and ensure we use the minimum size to avoid out-of-bounds errors
    int VectorSize = std::min((int)vector1.size(), (int)vector2.size());

    // Initialize a variable to accumulate the total difference between corresponding elements of the vectors
    double TotalDifference = 0;

    // Loop through each element up to the determined vector size
    for (int i = 0; i < VectorSize; i++) {
        // Calculate the difference between the corresponding elements of the two vectors
        double Difference = vector2[i] - vector1[i];

        // Accumulate the difference into the total difference variable
        TotalDifference += Difference;
    }

    // Calculate the average difference by dividing the total difference by the number of elements
    double AverageDifference = TotalDifference / VectorSize;

    // Return the calculated average difference
    return AverageDifference;
}

void Auto_class::Turn(float Rotation, float Velocity, float RotationCenterCm, float ErrorRange, float Timeout) {
    // Calculate the radii for the left and right wheels based on the rotation center
    double LeftRadiusCm = (RobotLength / 2) + RotationCenterCm;
    double RightRadiusCm = (RobotLength / 2) - RotationCenterCm;

    // The average radius used for calculating velocity proportions
    double averageRotateRadiusCm = (LeftRadiusCm + RightRadiusCm) / 2;

    // Calculate velocity proportions for the left and right wheels
    double LeftProportion = LeftRadiusCm / averageRotateRadiusCm;
    double RightProportion = -RightRadiusCm / averageRotateRadiusCm;

    // Initialize PID controller for controlling the rotation
    PID RotationPID(0.1, 0, 0.35, ErrorRange);

    // Start a timer to enforce the timeout limit
    timer timeout;

    // Loop until the rotation is within the acceptable error range or the timeout is reached
    while (!RotationPID.isSettled() && timeout.value() < Timeout) {
        // Calculate the current rotation error
        double Error = Rotation - Inertial.rotation();

        // Update the PID controller with the current error
        RotationPID.PIDCalculate(Error);

        // Calculate the average velocity based on the PID output, constrained by the maximum velocity
        double AverageVelocityRPM = fmin(Velocity, fmax(-Velocity, RotationPID.Value()));

        // Calculate the velocities for the left and right motors based on the proportional values
        double LeftVelocityRPM = LeftProportion * AverageVelocityRPM;
        double RightVelocityRPM = RightProportion * AverageVelocityRPM;

        // Spin the motors with the calculated velocities
        MotorSpin(LeftVelocityRPM, RightVelocityRPM, rpm);

        // Wait for 10 milliseconds before the next iteration to avoid overwhelming the control loop
        wait(10, msec);
    }

    // Stop the motors once the loop is exited, either because the target rotation was reached or the timeout was exceeded
    MotorStop(brake);
}

void Auto_class::MoveTurnTileWithProfileCalculation(float DistanceTiles, float MaxSpeed, float TargetAngle, float ProportionalGain, float TurnRatio, float Timeout) {
    // Initialize variables for speed control and position tracking
    float CurrentSpeed = 0;
    double CurrentPosition = 0;
    int MovementDirection = 1;  // Direction of movement (1 for forward, -1 for backward)

    // Constants and variables for motion profiling and PID tuning
    float AccelerationTime, DecelerationTime, TotalTime;  // Time intervals (seconds)
    float MaxVelocity;         // Maximum velocity in degrees per second
    float MaxVelocityRPM;     // Maximum velocity in RPM
    float AccelerationRate = 2400;  // Acceleration rate in degrees per second^2

    // Reset motor positions
    LeftMotor.resetPosition();
    RightMotor.resetPosition();

    // Calculate target distance in degrees based on tile length and gearing
    float TargetDegrees = DistanceTiles * TileLength * WheelsPerimeter * GearRatio * CompleteAngle;

    // Calculate maximum achievable velocity based on acceleration limits and user-defined maximum speed
    MaxVelocity = fmin(sqrt(TargetDegrees * AccelerationRate), MaxSpeed * 6);  // Calculate real maxVelocity

    // Calculate time intervals for motion profiling
    AccelerationTime = MaxVelocity / AccelerationRate;  // Acceleration time
    DecelerationTime = TargetDegrees / MaxVelocity;                // Deceleration time
    TotalTime = AccelerationTime + DecelerationTime;                          // Total movement time

    // Convert maximum velocity to RPM for motor control
    MaxVelocityRPM = MaxVelocity / 6;

    // Determine movement direction based on target distance sign
    if (DistanceTiles < 0) {
        MovementDirection = -1;  // Reverse direction for negative distance
    }

    // PID controllers for distance and turning
    PID TurnPID(ProportionalGain, 0.0000005, 0.0, 0.5);  // Turn PID controller with specified proportional gain

    // Timers for movement timeout and elapsed time
    timer moveTimeout;
    timer timer;

    // Main control loop for movement
    while (CurrentPosition < fabs(TargetDegrees) && moveTimeout.value() <= Timeout) {
        // Calculate speed profile based on current time within AccelerationTime and DecelerationTime
        if (timer.value() < AccelerationTime) {
            CurrentSpeed = cos(timer.value() / AccelerationTime * M_PI + M_PI) * MaxVelocityRPM / 2 + MaxVelocityRPM / 2;  // Accelerating phase
        } else if (timer.value() > DecelerationTime) {
            CurrentSpeed = fmax(cos((timer.value() - DecelerationTime) / AccelerationTime * M_PI) * MaxVelocityRPM / 2 + MaxVelocityRPM / 2, 300);  // Decelerating phase
            // This adjustment handles potential distance error correction
        } else {
            CurrentSpeed = CurrentSpeed;  // Maintain current speed
        }

        // Calculate turning error
        double TurnError = TargetAngle - Inertial.rotation(degrees);
        TurnPID.PIDCalculate(TurnError);  // Calculate PID value for turning
        double TurnSpeed = TurnPID.Value();

        // Adjust motor speeds for both forward movement and turning correction
        double LeftSpeed = MovementDirection * CurrentSpeed + TurnSpeed;
        double RightSpeed = MovementDirection * CurrentSpeed - TurnSpeed;

        MotorSpin(LeftSpeed, RightSpeed, rpm);

        // Wait for a short interval before the next iteration
        wait(10, msec);

        // Update current position based on average of motor encoder positions
        CurrentPosition = fabs((LeftMotor.position(deg) + RightMotor.position(deg)) / 2);

        // Emergency stop if turning error is too large
        if (fabs(TurnError) > 0.5) {
            MotorStop(brake);  // Stop the motors with brake mode
        }
    }

    // Stop the motors with hold mode after reaching the target distance or timeout
    MotorStop(brake);
}

void Auto_class::MoveTurnTileWithPID(float DistanceTile, float Rotation, float MoveVelocity, float RotateVelocity, float RatioToTurn, float ErrorRange, float Timeout){
    // Initialize PID controllers for distance, rotation, and synchronization with respective error ranges
    PID DistancePID(0.9, 0.0, 0.7, ErrorRange);  // PID for controlling distance
    PID RotationPID(0.5, 0.0, 0.0, DefaultRotateErrorRange);        // PID for controlling rotation
    PID SynchronizeVelocityPID(0.5, 0.0, 0.0, 10.0);  // PID for synchronizing motor velocities

    // Record the initial encoder value to measure distance traveled
    double EncoderRevolution = Encoder.rotation(rev);

    // Record the initial positions of each motor
    std::vector<double> InitialRevolution = {L1.position(rev), L2.position(rev), L3.position(rev), R1.position(rev), R2.position(rev), R3.position(rev)};

    // Start the timeout timer to ensure the function does not run indefinitely
    timer timeout;

    // Loop until both distance and rotation PID controllers are settled or the timeout is exceeded
    while (!(DistancePID.isSettled() && RotationPID.isSettled()) && timeout.value() < Timeout) {
        double DistanceError = 0.0;  // Variable to store the distance error
        double RotationError = 0.0;  // Variable to store the rotation error

        // Calculate the target distance in centimeters (1 tile = 60.96 cm)
        double TargetDistance = (DistanceTile * TileLength);

        double LeftVelocityRPM;
        double RightVelocityRPM;

        // Check if using encoder for distance measurement
        if (UseEncoder) {
            // Calculate the current encoder revolution relative to the initial value
            double EncoderCurrentRevolution = Encoder.rotation(rev) - EncoderRevolution;
            // Calculate the current distance traveled based on encoder revolution and circumference
            double CurrentDistance = EncoderCurrentRevolution * EncoderCircum;
            // Calculate the distance error
            DistanceError = TargetDistance - CurrentDistance;
            // Calculate the rotation error based on target and current rotation
            RotationError = (Rotation - Inertial.rotation());

            // Update the distance PID controller with the current distance error and get the PID output
            DistancePID.PIDCalculate(DistanceError);
            double MoveVelocityRPM = fmin(MoveVelocity, fmax(-MoveVelocity, DistancePID.Value()));

            // Update the rotation PID controller with the current rotation error and get the PID output
            RotationPID.PIDCalculate(RotationError);
            double RotationVelocityRPM = fmin(RotateVelocity, fmax(-RotateVelocity, RotationPID.Value()));

            // Determine the left and right motor velocities based on the ratio to turn
            if (CurrentDistance / fabs(TargetDistance) > RatioToTurn) {
                // If the drivetrain revolution per rotation is greater than the ratio to turn, 
                // adjust the left and right velocities to include rotation velocity
                LeftVelocityRPM = MoveVelocityRPM + RotationVelocityRPM;
                RightVelocityRPM = MoveVelocityRPM - RotationVelocityRPM;
            } else {
                // Set both left and right velocities to the move velocity only
                LeftVelocityRPM = MoveVelocityRPM;
                RightVelocityRPM = MoveVelocityRPM;
            }
        } else {
            // Get the current positions of each motor
            std::vector<double> RunRevolution = {L1.position(rev), L2.position(rev), L3.position(rev), R1.position(rev), R2.position(rev), R3.position(rev)};
            double DrivertainRevolution = AverageDifference(InitialRevolution, RunRevolution);
            // Calculate the current distance traveled based on motor revolutions, gear ratio, and wheel circumference
            double CurrentDrivertainRevolution = (DrivertainRevolution) * (1.0 / GearRatio) * (WheelCircum / 1.0);
            // Calculate the distance error
            DistanceError = TargetDistance - CurrentDrivertainRevolution;
            // Calculate the rotation error based on target and current rotation
            RotationError = (Rotation - Inertial.rotation());

            // Update the distance PID controller with the current distance error and get the PID output
            DistancePID.PIDCalculate(DistanceError);
            double MoveVelocityRPM = fmin(MoveVelocity, fmax(-MoveVelocity, DistancePID.Value()));

            // Update the rotation PID controller with the current rotation error and get the PID output
            RotationPID.PIDCalculate(RotationError);
            double RotationVelocityRPM = fmin(RotateVelocity, fmax(-RotateVelocity, RotationPID.Value()));

            // Determine the left and right motor velocities based on the ratio to turn
            if (CurrentDrivertainRevolution / fabs(TargetDistance) > RatioToTurn) {
                // If the drivetrain revolution per rotation is greater than the ratio to turn, 
                // adjust the left and right velocities to include rotation velocity
                LeftVelocityRPM = MoveVelocityRPM + RotationVelocityRPM;
                RightVelocityRPM = MoveVelocityRPM - RotationVelocityRPM;
            } else {
                // Set both left and right velocities to the move velocity only
                LeftVelocityRPM = MoveVelocityRPM;
                RightVelocityRPM = MoveVelocityRPM;
            }
        }

        // Calculate the current velocity difference between the left and right motors in RPM
        double VelocityDifferenceRPM = LeftMotor.velocity(rpm) - RightMotor.velocity(rpm);
        // Convert the velocity difference from RPM to centimeters per second
        double VelocityDifferenceCmRPS = (VelocityDifferenceRPM) * (600.0 / 60) * (1 / GearRatio) * (WheelCircum / 1.0);

        // Calculate the expected velocity difference between the left and right motors based on target velocities
        double CalculateVelocityDifferenceRPM = LeftVelocityRPM - RightVelocityRPM;
        // Convert the expected velocity difference from RPM to centimeters per second
        double CalculateVelocityDifferenceCmRPS = (CalculateVelocityDifferenceRPM) * (600.0 / 60) * (1 / GearRatio) * (WheelCircum / 1.0);

        // Calculate the error in velocity difference
        double VelocityDifference = CalculateVelocityDifferenceCmRPS - VelocityDifferenceCmRPS;

        // Update the synchronization PID controller with the velocity difference error and get the PID output
        SynchronizeVelocityPID.PIDCalculate(VelocityDifference);
        double VelocityStraighten = SynchronizeVelocityPID.Value();

        // Adjust the left and right motor velocities with the synchronization value
        LeftVelocityRPM += VelocityStraighten;
        RightVelocityRPM -= VelocityStraighten;

        // Spin the motors with the adjusted velocities
        MotorSpin(LeftVelocityRPM, RightVelocityRPM, rpm);

        // Wait for 10 milliseconds before the next iteration of the loop
        wait(10, msec);
    }
    
    // Stop the motors once the loop is exited, either because the target was reached or the timeout was exceeded
    MotorStop(coast);
}