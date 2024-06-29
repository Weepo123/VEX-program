#include "vex.h"
#include "robot-config.h"
#include "Auto_function/Auto_class.h"
#include "Auto_function/PID.h"

using namespace vex;

void Auto_class::motor_stop(brakeType mode) {
    // Stop the left motor with the specified braking mode
    Left_motor.stop(mode);
    
    // Stop the right motor with the specified braking mode
    Right_motor.stop(mode);
}


void Auto_class::motor_spin(double LVelocity, double RVelocity) {
    // Scale the velocities to ensure they are within the valid range (0 to 12 volts)
    double scale = 12.0 / fmax(12.0, fmax(fabs(LVelocity), fabs(RVelocity)));
    
    // Apply the scaling factor to the velocities
    LVelocity *= scale;
    RVelocity *= scale;
    
    // Spin the left motor with the scaled velocity
    Left_motor.spin(fwd, LVelocity, volt);
    
    // Spin the right motor with the scaled velocity
    Right_motor.spin(fwd, RVelocity, volt);
}

void Auto_class::move_Tile(float disTile, double speed_max, double target, double kp, double ratio_to_turn) {
    // Initialize variables for speed control and position tracking
    float speed_now = 0;
    double position_now = 0;
    int direction = 1;  // Direction of movement (1 for forward, -1 for backward)

    // Constants and variables for motion profiling and PID tuning
    float t1, t2, t3;  // Time intervals (seconds)
    float Vmax;         // Maximum velocity in degrees per second
    float Vmax_rpm;     // Maximum velocity in RPM
    float acceleration_per_degrees = 3000;  // Acceleration rate in degrees per second^2

    // Reset motor positions
    Left_motor.resetPosition();
    Right_motor.resetPosition();

    // Calculate target distance in degrees based on tile length and gearing
    float target_deg = disTile * TileLength * WheelsPerimeter * GearRatio * CompleteAngle;

    // Calculate maximum achievable velocity based on acceleration limits and user-defined maximum speed
    Vmax = fmin(sqrt(target_deg * acceleration_per_degrees), speed_max * 6);  // Calculate real Vmax

    // Calculate time intervals for motion profiling
    t1 = Vmax / acceleration_per_degrees;  // Acceleration time
    t2 = target_deg / Vmax;                // Deceleration time
    t3 = t1 + t2;                          // Total movement time

    // Convert maximum velocity to RPM for motor control
    Vmax_rpm = Vmax / 6;

    // Determine movement direction based on target distance sign
    if (disTile < 0) {
        direction = -1;  // Reverse direction for negative distance
    }

    // PID constants for turning correction
    float ki = 0.0000005;  // Integral gain
    float kd = 0;          // Derivative gain

    // Variables for PID control and turning correction
    float turn_error = 0;  // Error in target angle
    float prev_error = 0;  // Previous error for derivative term
    float INT = 0;         // Integral term accumulator
    double speed_turn = 0; // Speed adjustment for turning correction

    // Timers for movement timeout and elapsed time
    timer move_timeout;
    timer Timer;

    // Main control loop for movement
    while (position_now < fabs(target_deg) && move_timeout.value() <= 5) {
        // Calculate speed profile based on current time within t1 and t2
        if (Timer.value() < t1) {
            speed_now = cos(Timer.value() / t1 * M_PI + M_PI) * Vmax_rpm / 2 + Vmax_rpm / 2;  // Accelerating phase
        } else if (Timer.value() > t2) {
            speed_now = fmax(cos((Timer.value() - t2) / t1 * M_PI) * Vmax_rpm / 2 + Vmax_rpm / 2, 300);  // Decelerating phase
            // This adjustment handles potential distance error correction
        } else {
            speed_now = speed_now;  // Maintain current speed
        }

        // Calculate turning error based on target angle and current orientation
        turn_error = target - Inertial.rotation(degrees);
        INT = INT + turn_error;  // Accumulate integral error

        // Apply PID control for turning correction after reaching a certain distance ratio
        if (position_now / fabs(target) > ratio_to_turn) {
            speed_turn = (turn_error) * kp + INT * ki + (turn_error - prev_error) * kd;  // PID calculation
        } else {
            speed_turn = 0;  // No turning correction before reaching ratio_to_turn
        }

        // Adjust motor speeds for both forward movement and turning correction
        motor_spin(direction * speed_now + speed_turn, direction * speed_now - speed_turn);

        // Wait for a short interval before the next iteration
        wait(10, msec);

        // Update current position based on average of motor encoder positions
        position_now = fabs(((L1.position(deg) + L2.position(deg) + L3.position(deg)) + 
                             (R1.position(deg) + R2.position(deg) + R3.position(deg))) / 6);

        // Emergency stop if turning error is too large
        if (turn_error > 0.5) {
            motor_stop(brake);  // Stop the motors with brake mode
        }

        // Update previous error for derivative term in PID
        prev_error = turn_error;
    }

    // Stop the motors with hold mode after reaching the target distance or timeout
    motor_stop(hold);
}


void Auto_class::Turn(float Rotation, double Velocity, double RotationCenterCm, double ErrorRange, double Timeout) {
    // Calculate the radii for the left and right wheels based on the rotation center
    double LeftRadiusCm = (RobotLength / 2) + RotationCenterCm;
    double RightRadiusCm = (RobotLength / 2) - RotationCenterCm;

    // The average radius used for calculating velocity proportions
    double averageRotateRadiusCm = (LeftRadiusCm + RightRadiusCm) / 2;

    // Calculate velocity proportions for the left and right wheels
    double LeftProportion = LeftRadiusCm / averageRotateRadiusCm;
    double RightProportion = -RightRadiusCm / averageRotateRadiusCm;

    // Initialize PID controller for controlling the rotation
    PID RotationPID(0.62, 0, 0, ErrorRange);

    // Start a timer to enforce the timeout limit
    timer timeout;

    // Loop until the rotation is within the acceptable error range or the timeout is reached
    while (!RotationPID.isSettled() && timeout.value() < Timeout) {
        // Calculate the current rotation error
        double Error = Rotation - Inertial.rotation();

        // Update the PID controller with the current error
        RotationPID.PID_Calculate(Error);

        // Calculate the average velocity based on the PID output, constrained by the maximum velocity
        double AverageVelocityRPM = fmin(Velocity, fmax(-Velocity, RotationPID.Value()));

        // Calculate the velocities for the left and right motors based on the proportional values
        double LeftVelocityRPM = LeftProportion * AverageVelocityRPM;
        double RightVelocityRPM = RightProportion * AverageVelocityRPM;

        // Spin the motors with the calculated velocities
        motor_spin(LeftVelocityRPM, RightVelocityRPM);

        // Wait for 10 milliseconds before the next iteration to avoid overwhelming the control loop
        wait(10, msec);
    }

    // Stop the motors once the loop is exited, either because the target rotation was reached or the timeout was exceeded
    motor_stop(brake);
}


void Auto_class::MoveTurnTile(double DistanceTile, double Rotation, double MoveVelocity, double RotateVelocity, double RatioToTurn, double ErrorRang, double Timeout) {
    // Initialize PID controllers for distance, rotation, and synchronization with respective error ranges
    PID DistancePID(0.0, 0.0, 0.0, ErrorRang);  // PID for controlling distance
    PID RotationPID(0.0, 0.0, 0.0, 0.5);        // PID for controlling rotation
    PID SynchronizeVelocityPID(0.0, 0.0, 0.0, 0.0);  // PID for synchronizing motor velocities

    // Record the initial encoder value to measure distance traveled
    double EncoderRevalution = Encoder.rotation(rev);

    // Reset the positions of the left and right motors to zero
    Left_motor.resetPosition();
    Right_motor.resetPosition();

    // Start the timeout timer to ensure the function does not run indefinitely
    timer timeout;

    // Loop until both distance and rotation PID controllers are settled or the timeout is exceeded
    while (!(DistancePID.isSettled() && RotationPID.isSettled()) && timeout.value() < Timeout) {
        double DistanceError;  // Variable to store the distance error
        double RotationError;  // Variable to store the rotation error

        // Calculate the target distance in centimeters (1 tile = 60.96 cm)
        double TargetDistance = (DistanceTile * TileLength);

        // Calculate the average motor revolutions for all motors
        double DrivetrainRevalution = fabs(((L1.position(rev) + L2.position(rev) + L3.position(rev)) + (R1.position(rev) + R2.position(rev) + R3.position(rev))) / 6);
        double LeftVelocityRPM;
        double RightVelocityRPM;

        // Check if using encoder for distance measurement
        if (UseEncoder) {
            // Calculate the current encoder revolution relative to the initial value
            double EncoderCurrentRevalution = Encoder.rotation(rev) - EncoderRevalution;
            // Calculate the current distance traveled based on encoder revolution and circumference
            double CurrentDistance = EncoderCurrentRevalution * EncoderCircum;
            // Calculate the distance error
            DistanceError = TargetDistance - CurrentDistance;
            // Calculate the rotation error based on target and current rotation
            RotationError = (Rotation - Inertial.rotation());
        } else {
            // Calculate the current distance traveled based on motor revolutions, gear ratio, and wheel circumference
            double CurrentDrivertainRevalution = (DrivetrainRevalution) * (1.0 / GearRatio) * (WheelCircum / 1.0);
            // Calculate the distance error
            DistanceError = TargetDistance - CurrentDrivertainRevalution;
            // Calculate the rotation error based on target and current rotation
            RotationError = (Rotation - Inertial.rotation());
        }

        // Update the distance PID controller with the current distance error and get the PID output
        DistancePID.PID_Calculate(DistanceError);
        double MoveVelocityRPM = fmin(MoveVelocity, fmax(-MoveVelocity, DistancePID.Value()));

        // Update the rotation PID controller with the current rotation error and get the PID output
        RotationPID.PID_Calculate(RotationError);
        double RotationVelocityRPM = fmin(RotateVelocity, fmax(-RotateVelocity, RotationPID.Value()));

        // Determine the left and right motor velocities based on the ratio to turn
        if (DrivetrainRevalution / Rotation > RatioToTurn) {
            // If the drivetrain revolution per rotation is greater than the ratio to turn, 
            // adjust the left and right velocities to include rotation velocity
            LeftVelocityRPM = MoveVelocityRPM + RotationVelocityRPM;
            RightVelocityRPM = MoveVelocityRPM - RotationVelocityRPM;
        } else {
            // Set both left and right velocities to the move velocity only
            LeftVelocityRPM = MoveVelocityRPM;
            RightVelocityRPM = MoveVelocityRPM;
        }

        // Calculate the current velocity difference between the left and right motors in RPM
        double VelocityDifferenceRPM = fabs(Left_motor.velocity(rpm) - Right_motor.velocity(rpm));
        // Convert the velocity difference from RPM to centimeters per second
        double VelocityDifferenceCmRPS = (VelocityDifferenceRPM / 100.0) * (600.0 / 60) * (1 / GearRatio) * (WheelCircum / 1.0);

        // Calculate the expected velocity difference between the left and right motors based on target velocities
        double CalculateVelocityDifferenceRPM = fabs(LeftVelocityRPM - RightVelocityRPM);
        // Convert the expected velocity difference from RPM to centimeters per second
        double CalculateVelocityDifferenceCmRPC = (CalculateVelocityDifferenceRPM / 100.0) * (600.0 / 60) * (1 / GearRatio) * (WheelCircum / 1.0);

        // Calculate the error in velocity difference
        double VelocityDifference = VelocityDifferenceCmRPS - CalculateVelocityDifferenceCmRPC;

        // Update the synchronization PID controller with the velocity difference error and get the PID output
        SynchronizeVelocityPID.PID_Calculate(VelocityDifference);
        double VelocityStraighten = SynchronizeVelocityPID.Value();

        // Adjust the left and right motor velocities with the synchronization value
        LeftVelocityRPM += VelocityStraighten;
        RightVelocityRPM += VelocityStraighten;

        // Spin the motors with the adjusted velocities
        motor_spin(LeftVelocityRPM, RightVelocityRPM);

        // Wait for 10 milliseconds before the next iteration of the loop
        wait(10, msec);
    }
    
    // Stop the motors once the loop is exited, either because the target was reached or the timeout was exceeded
    motor_stop(brake);
}

