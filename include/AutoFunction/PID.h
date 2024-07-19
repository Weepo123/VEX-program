#pragma once
#include "vex.h"
#include "AutoFunction/PID.h"

class PID {
    public:
        // Constructor with default parameter values
        PID(double kP = 0, double kI = 0, double kD = 0, double ErrorRang = 3, double ErrorCount = 5);

        // Method to calculate the PID based on the current error
        void PIDCalculate(double Error);

        // Method to compute the PID output value
        double Value();

        // Method to check if the error is settled within the acceptable range for a specified count
        bool isSettled();

    private:
        // PID coefficients
        double kPValue, kIValue, kDValue;

        // Error terms
        double CurrentError, Integrator, DeltaError, PreviousError;

        // Error handling parameters
        double AbsErrorRang, MinErrorCount;

        // Count to determine if the error is settled
        double SetlCount;
};
