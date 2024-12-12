#include "vex.h"
#include "robot-config.h"
#include "AutoFunction/movement.h"
#include "AutoFunction/PID.h"
#include <vector>

using namespace vex;

void autoClass::motorStop(brakeType mode) {
    // Stop the left motor with the specified braking mode
    LeftMotor.stop(mode);
    
    // Stop the right motor with the specified braking mode
    RightMotor.stop(mode);
}

void autoClass::motorSpin(double leftVelocity, double rightVelocity, velocityUnits units) {
    // Spin the left motor at the specified velocity
    LeftMotor.spin(fwd, leftVelocity, units);
    // Spin the right motor at the specified velocity
    RightMotor.spin(fwd, rightVelocity, units);
}

double autoClass::averageDifference(std::vector<double> vector1, std::vector<double> vector2) {
    // Determine the size of the vectors and ensure we use the minimum size to avoid out-of-bounds errors
    int vectorSize = std::min((int)vector1.size(), (int)vector2.size());

    // Initialize a variable to accumulate the total difference between corresponding elements of the vectors
    double totalDifference = 0;

    // Loop through each element up to the determined vector size
    for (int i = 0; i < vectorSize; i++) {
        // Calculate the difference between the corresponding elements of the two vectors
        double difference = vector2[i] - vector1[i];

        // Accumulate the difference into the total difference variable
        totalDifference += difference;
    }

    // Calculate the average difference by dividing the total difference by the number of elements
    double averageDifference = totalDifference / vectorSize;

    // Return the calculated average difference
    return averageDifference;
}

void autoClass::turn(float rotation, float velocity, float rotationCenterCm, float timeout) {
    // Calculate the radii for the left and right wheels based on the rotation center
    double leftRadiusCm = (robotLength / 2) + rotationCenterCm;
    double rightRadiusCm = (robotLength / 2) - rotationCenterCm;

    // The average radius used for calculating velocity proportions
    double averageRotateRadiusCm = (leftRadiusCm + rightRadiusCm) / 2;

    // Calculate velocity proportions for the left and right wheels
    double leftProportion = leftRadiusCm / averageRotateRadiusCm;
    double rightProportion = -rightRadiusCm / averageRotateRadiusCm;

    // Initialize PID controller for controlling the rotation
    PID rotationPID(3, 0.003, 0.35, defaultRotateErrorRange);

    // Start a timer to enforce the timeout limit
    timer timeoutTimer;

    // Loop until the rotation is within the acceptable error range or the timeout is reached
    while (!rotationPID.isSettled() && timeoutTimer.value() < timeout) {
        // Calculate the current rotation error
        double error = rotation - Inertial.rotation();

        // Update the PID controller with the current error
        rotationPID.pidCalculate(error);

        // Calculate the average velocity based on the PID output, constrained by the maximum velocity
        double averageVelocityRPM = fmin(velocity, fmax(-velocity, rotationPID.value()));

        // Calculate the velocities for the left and right motors based on the proportional values
        double leftVelocityRPM = leftProportion * averageVelocityRPM;
        double rightVelocityRPM = rightProportion * averageVelocityRPM;

        // Spin the motors with the calculated velocities
        motorSpin(leftVelocityRPM, rightVelocityRPM, rpm);

        // Wait for 10 milliseconds before the next iteration to avoid overwhelming the control loop
        wait(10, msec);
    }

    // Stop the motors once the loop is exited, either because the target rotation was reached or the timeout was exceeded
    motorStop(hold);
}

void autoClass::moveTurnTileWithPID(float distanceTile, float rotation, float moveVelocity, float rotateVelocity, float ratioToTurn, float timeout) {
    moveTurnCmWithPID(distanceTile * tileLength, rotation, moveVelocity, rotateVelocity, ratioToTurn, timeout);
}

void autoClass::moveTurnTileWithProfileCalculation(float distanceCm, float maxSpeed, float targetAngle, float proportionalGain, float turnRatio, float timeout) {
    moveTurnCmWithProfileCalculation(distanceCm * tileLength, maxSpeed, targetAngle, proportionalGain, turnRatio, timeout);
}

void autoClass::moveTurnCmWithProfileCalculation(float distanceCm, float maxSpeed, float targetAngle, float proportionalGain, float turnRatio, float timeout) {
    // Initialize variables for speed control and position tracking
    float currentSpeed = 0;
    double currentPosition = 0;
    int movementDirection = 1;  // Direction of movement (1 for forward, -1 for backward)

    // Constants and variables for motion profiling and PID tuning
    float accelerationTime, decelerationTime, totalTime;  // Time intervals (seconds)
    float maxVelocity;         // Maximum velocity in degrees per second
    float maxVelocityRPM;     // Maximum velocity in RPM
    float accelerationRate = 2400;  // Acceleration rate in degrees per second^2

    // Reset motor positions
    LeftMotor.resetPosition();
    RightMotor.resetPosition();

    // Calculate target distance in degrees based on tile length and gearing
    float targetDegrees = distanceCm * wheelsPerimeter * gearRatio * completeAngle;

    // Calculate maximum achievable velocity based on acceleration limits and user-defined maximum speed
    maxVelocity = fmin(sqrt(targetDegrees * accelerationRate), maxSpeed * 6);  // Calculate real maxVelocity

    // Calculate time intervals for motion profiling
    accelerationTime = maxVelocity / accelerationRate;  // Acceleration time
    decelerationTime = targetDegrees / maxVelocity;                // Deceleration time
    totalTime = accelerationTime + decelerationTime;                          // Total movement time

    // Convert maximum velocity to RPM for motor control
    maxVelocityRPM = maxVelocity / 6;

    // Determine movement direction based on target distance sign
    if (distanceCm < 0) {
        movementDirection = -1;  // Reverse direction for negative distance
    }

    // PID controllers for distance and turning
    PID turnPID(proportionalGain, 0.0000005, 0.0, defaultRotateErrorRange);  // Turn PID controller with specified proportional gain

    // Timers for movement timeout and elapsed time
    timer moveTimeout;
    timer elapsedTime;

    // Main control loop for movement
    while (currentPosition < fabs(targetDegrees) && moveTimeout.value() <= timeout) {
        // Calculate speed profile based on current time within accelerationTime and decelerationTime
        if (elapsedTime.value() < accelerationTime) {
            currentSpeed = cos(elapsedTime.value() / accelerationTime * M_PI + M_PI) * maxVelocityRPM / 2 + maxVelocityRPM / 2;  // Accelerating phase
        } else if (elapsedTime.value() > decelerationTime) {
            currentSpeed = fmax(cos((elapsedTime.value() - decelerationTime) / accelerationTime * M_PI) * maxVelocityRPM / 2 + maxVelocityRPM / 2, 300);  // Decelerating phase
        } else {
            currentSpeed = currentSpeed;  // Maintain current speed
        }

        // Calculate turning error
        double turnError = targetAngle - Inertial.rotation(degrees);
        turnPID.pidCalculate(turnError);  // Calculate PID value for turning
        double turnSpeed = turnPID.value();

        // Adjust motor speeds for both forward movement and turning correction
        double leftSpeed = movementDirection * currentSpeed + turnSpeed;
        double rightSpeed = movementDirection * currentSpeed - turnSpeed;

        motorSpin(leftSpeed, rightSpeed, rpm);

        // Wait for a short interval before the next iteration
        wait(10, msec);

        // Update current position based on average of motor encoder positions
        currentPosition = fabs((LeftMotor.position(deg) + RightMotor.position(deg)) / 2);

        // Emergency stop if turning error is too large
        if (fabs(turnError) > 0.5) {
            motorStop(brake);  // Stop the motors with brake mode
        }
    }

    // Stop the motors with hold mode after reaching the target distance or timeout
    motorStop(brake);
}

void autoClass::moveTurnCmWithPID(float distanceCm, float rotation, float moveVelocity, float rotateVelocity, float ratioToTurn, float timeout) {
    // Initialize PID controllers for distance, rotation, and synchronization with respective error ranges
    PID distancePID(5, 0.02575, 0.3, defaultMoveErrorRange);  // PID for controlling distance
    PID rotationPID(3, 0.003, 0.35, defaultRotateErrorRange);  // PID for controlling rotation
    PID synchronizeVelocityPID(0.0, 0.0, 0.0, 100.0);  // PID for synchronizing motor velocities

    // Record the initial encoder value to measure distance traveled
    double encoderRevolution = Encoder.rotation(rev);

    // Record the initial positions of each motor
    std::vector<double> initialRevolution = {L1.position(rev), L2.position(rev), L3.position(rev), R1.position(rev), R2.position(rev), R3.position(rev)};

    // Start the timeout timer to ensure the function does not run indefinitely
    timer timeoutTimer;

    // Loop until both distance and rotation PID controllers are settled or the timeout is exceeded
    while (!(distancePID.isSettled() && rotationPID.isSettled()) && timeoutTimer.value() < timeout) {
        double distanceError = 0.0;  // Variable to store the distance error
        double rotationError = 0.0;  // Variable to store the rotation error

        // Calculate the target distance in centimeters
        double targetDistance = distanceCm;

        double leftVelocityRPM;
        double rightVelocityRPM;

        // Check if using encoder for distance measurement
        if (useEncoder) {
            // Calculate the current encoder revolution relative to the initial value
            double encoderCurrentRevolution = Encoder.rotation(rev) - encoderRevolution;
            // Calculate the current distance traveled based on encoder revolution and circumference
            double currentDistance = encoderCurrentRevolution * encoderCircumference;
            // Calculate the distance error
            distanceError = targetDistance - currentDistance;
            // Calculate the rotation error based on target and current rotation
            rotationError = (rotation - Inertial.rotation());

            // Update the distance PID controller with the current distance error and get the PID output
            distancePID.pidCalculate(distanceError);
            double moveVelocityRPM = fmin(moveVelocity, fmax(-moveVelocity, distancePID.value()));

            // Update the rotation PID controller with the current rotation error and get the PID output
            rotationPID.pidCalculate(rotationError);
            double rotationVelocityRPM = fmin(rotateVelocity, fmax(-rotateVelocity, rotationPID.value()));

            // Determine the left and right motor velocities based on the ratio to turn
            if (currentDistance / fabs(targetDistance) > ratioToTurn) {
                // If the drivetrain revolution per rotation is greater than the ratio to turn, 
                // adjust the left and right velocities to include rotation velocity
                leftVelocityRPM = moveVelocityRPM + rotationVelocityRPM;
                rightVelocityRPM = moveVelocityRPM - rotationVelocityRPM;
            } else {
                // Set both left and right velocities to the move velocity only
                leftVelocityRPM = moveVelocityRPM;
                rightVelocityRPM = moveVelocityRPM;
            }
        } else {
            // Get the current positions of each motor
            std::vector<double> runRevolution = {L1.position(rev), L2.position(rev), L3.position(rev), R1.position(rev), R2.position(rev), R3.position(rev)};
            double drivertainRevolution = averageDifference(initialRevolution, runRevolution);
            // Calculate the current distance traveled based on motor revolutions, gear ratio, and wheel circumference
            double currentDrivertainRevolution = (drivertainRevolution) * (1.0 / gearRatio) * (wheelCircumference / 1.0);
            // Calculate the distance error
            distanceError = targetDistance - currentDrivertainRevolution;
            // Calculate the rotation error based on target and current rotation
            rotationError = (rotation - Inertial.rotation());

            // Update the distance PID controller with the current distance error and get the PID output
            distancePID.pidCalculate(distanceError);
            double moveVelocityRPM = fmin(moveVelocity, fmax(-moveVelocity, distancePID.value()));

            // Update the rotation PID controller with the current rotation error and get the PID output
            rotationPID.pidCalculate(rotationError);
            double rotationVelocityRPM = fmin(rotateVelocity, fmax(-rotateVelocity, rotationPID.value()));

            // Determine the left and right motor velocities based on the ratio to turn
            if (currentDrivertainRevolution / fabs(targetDistance) > ratioToTurn) {
                // If the drivetrain revolution per rotation is greater than the ratio to turn, 
                // adjust the left and right velocities to include rotation velocity
                leftVelocityRPM = moveVelocityRPM + rotationVelocityRPM;
                rightVelocityRPM = moveVelocityRPM - rotationVelocityRPM;
            } else {
                // Set both left and right velocities to the move velocity only
                leftVelocityRPM = moveVelocityRPM;
                rightVelocityRPM = moveVelocityRPM;
            }
        }

        // Calculate the current velocity difference between the left and right motors in RPM
        double velocityDifferenceRPM = LeftMotor.velocity(rpm) - RightMotor.velocity(rpm);
        // Convert the velocity difference from RPM to centimeters per second
        double velocityDifferenceCmRPS = (velocityDifferenceRPM) * (600.0 / 60) * (1 / gearRatio) * (wheelCircumference / 1.0);

        // Calculate the expected velocity difference between the left and right motors based on target velocities
        double calculateVelocityDifferenceRPM = leftVelocityRPM - rightVelocityRPM;
        // Convert the expected velocity difference from RPM to centimeters per second
        double calculateVelocityDifferenceCmRPS = (calculateVelocityDifferenceRPM) * (600.0 / 60) * (1 / gearRatio) * (wheelCircumference / 1.0);

        // Calculate the error in velocity difference
        double velocityDifference = calculateVelocityDifferenceCmRPS - velocityDifferenceCmRPS;

        // Update the synchronization PID controller with the velocity difference error and get the PID output
        synchronizeVelocityPID.pidCalculate(velocityDifference);
        double velocityStraighten = synchronizeVelocityPID.value();

        // Adjust the left and right motor velocities with the synchronization value
        leftVelocityRPM += velocityStraighten;
        rightVelocityRPM -= velocityStraighten;

        // Spin the motors with the adjusted velocities
        motorSpin(leftVelocityRPM, rightVelocityRPM, rpm);

        // Wait for 10 milliseconds before the next iteration of the loop
        wait(10, msec);
    }
    
    // Stop the motors once the loop is exited, either because the target was reached or the timeout was exceeded
    motorStop(brake);
}
