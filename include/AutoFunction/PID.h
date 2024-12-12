#pragma once
#include "vex.h"
#include "AutoFunction/PID.h"

class PID {
    public:
        // Constructor with default parameter values
        PID(double kP = 0, double kI = 0, double kD = 0, double errorRange = 3, double errorCount = 5);

        // Method to calculate the PID based on the current error
        void pidCalculate(double error);

        // Method to compute the PID output value
        double value();

        // Method to check if the error is settled within the acceptable range for a specified count
        bool isSettled();

    private:
        // PID coefficients
        double kPValue, kIValue, kDValue;

        // Error terms
        double currentError, integrator, deltaError, previousError;

        // Error handling parameters
        double absErrorRange, minErrorCount;

        // Count to determine if the error is settled
        double settleCount;
};
