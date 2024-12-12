#include "vex.h"
#include "robot-config.h"
#include "AutoFunction/PID.h"

PID::PID(double kP, double kI, double kD, double errorRange, double errorCount) {
    // Initialize PID coefficients
    kPValue = kP;         // Proportional coefficient
    kIValue = kI;         // Integral coefficient
    kDValue = kD;         // Derivative coefficient
    
    // Initialize integrator and error values
    integrator = 0;       // Integral term accumulator
    deltaError = 0;       // Change in error (for derivative term)
    currentError = 10e10; // Current error (initialized to a very large number)
    previousError = 10e10; // Previous error (initialized to a very large number)
    
    // Initialize error handling parameters
    minErrorCount = errorCount;      // Minimum count of consecutive errors within range to consider settled
    absErrorRange = fabs(errorRange);  // Acceptable range for errors (absolute value)
    settleCount = 0;                   // Counter for consecutive errors within acceptable range
}

void PID::pidCalculate(double error) {
    // Update previous error
    if (previousError == 10e10) { // If previous error is uninitialized
        previousError = error;    // Initialize it to the current error
    } else {
        previousError = currentError; // Otherwise, update it to the last known current error
    }

    // Update current error
    currentError = error;

    // Check if error has crossed zero and reset integrator if it has
    bool isCrossZero = (currentError >= 0 && previousError <= 0) || (currentError <= 0 && previousError >= 0);
    if (isCrossZero) {
        integrator = 0; // Reset integrator if error has crossed zero
    } else {
        // Otherwise, accumulate the integrator
        integrator += error;
    }

    // Calculate change in error (derivative term)
    deltaError = currentError - previousError;

    // Update settle count based on error magnitude
    if (fabs(error) < absErrorRange) { // If error is within the acceptable range
        settleCount++;                  // Increment the settle count
        // Cap the settle count to ensure it doesn't exceed minErrorCount + 1
        settleCount = fmin(settleCount, minErrorCount + 1);
    } else {
        settleCount = 0; // Reset settle count if error is outside the acceptable range
    }
}

double PID::value() {
    // Calculate the proportional term
    double P = currentError * kPValue;
    
    // Calculate the integral term
    double I = integrator * kIValue;
    
    // Calculate the derivative term
    double D = deltaError * kDValue;
    
    // Return the sum of all three terms
    return P + I + D;
}

bool PID::isSettled() {
    // Check if the current error is within the acceptable range and if the settle count is sufficient
    if (fabs(currentError) < absErrorRange && settleCount >= minErrorCount) {
        return true; // Error has settled
    } else {
        return false; // Error has not settled
    }
}

