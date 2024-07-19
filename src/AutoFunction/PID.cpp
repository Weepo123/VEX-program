#include "AutoFunction/PID.h"

PID::PID(double kP, double kI, double kD, double ErrorRang, double ErrorCount) {
    // Initialize PID coefficients
    kPValue = kP;         // Proportional coefficient
    kIValue = kI;         // Integral coefficient
    kDValue = kD;         // Derivative coefficient
    
    // Initialize integrator and error values
    Integrator = 0;       // Integral term accumulator
    DeltaError = 0;       // Change in error (for derivative term)
    CurrentError = 10e10; // Current error (initialized to a very large number)
    PreviousError = 10e10;// Previous error (initialized to a very large number)
    
    // Initialize error handling parameters
    MinErrorCount = ErrorCount;      // Minimum count of consecutive errors within range to consider settled
    AbsErrorRang = fabs(ErrorRang);  // Acceptable range for errors (absolute value)
    SetlCount = 0;                   // Counter for consecutive errors within acceptable range
}

void PID::PIDCalculate(double Error) {
    // Update previous error
    if (PreviousError == 10e10) { // If previous error is uninitialized
        PreviousError = Error;    // Initialize it to the current error
    } else {
        PreviousError = CurrentError; // Otherwise, update it to the last known current error
    }

    // Update current error
    CurrentError = Error;

    // Check if error has crossed zero and reset integrator if it has
    bool isCrossZero = (CurrentError >= 0 && PreviousError <= 0) || (CurrentError <= 0 && PreviousError >= 0);
    if (isCrossZero) {
        Integrator = 0; // Reset integrator if error has crossed zero
    } else {
        // Otherwise, accumulate the integrator
        Integrator += Error;
    }

    // Calculate change in error (derivative term)
    DeltaError = CurrentError - PreviousError;

    // Update settle count based on error magnitude
    if (fabs(Error) < AbsErrorRang) { // If error is within the acceptable range
        SetlCount++;                  // Increment the settle count
        // Cap the settle count to ensure it doesn't exceed MinErrorCount + 1
        SetlCount = fmin(SetlCount, MinErrorCount + 1);
    } else {
        SetlCount = 0; // Reset settle count if error is outside the acceptable range
    }
}

double PID::Value() {
    // Calculate the proportional term
    double P = CurrentError * kPValue;
    
    // Calculate the integral term
    double I = Integrator * kIValue;
    
    // Calculate the derivative term
    double D = DeltaError * kDValue;
    
    // Return the sum of all three terms
    return P + I + D;
}

bool PID::isSettled() {
    // Check if the current error is within the acceptable range and if the settle count is sufficient
    if (fabs(CurrentError) < AbsErrorRang && SetlCount >= MinErrorCount) {
        return true; // Error has settled
    } else {
        return false; // Error has not settled
    }
}
