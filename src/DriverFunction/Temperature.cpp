#include "vex.h"
#include "robot-config.h"
#include "DriverFunction/DriverFunction.h"

using namespace vex;

void driverClass::drivertainTemperature() {
    /* Clear the Brain screen and prepare to print */
    // Brain.Screen.clearScreen();
    Brain.Screen.printAt(220, 20, "L1: %.3f", L1.temperature(celsius));
    Brain.Screen.printAt(220, 40, "L2: %.3f", L2.temperature(celsius));
    Brain.Screen.printAt(220, 60, "L3: %.3f", L3.temperature(celsius));
    Brain.Screen.printAt(220, 80, "R1: %.3f", R1.temperature(celsius));
    Brain.Screen.printAt(220, 100, "R2: %.3f", R2.temperature(celsius));
    Brain.Screen.printAt(220, 120, "R3: %.3f", R3.temperature(celsius));
    Brain.Screen.printAt(220, 140, "Intake1: %3.f", Intake1.temperature(celsius));
    Brain.Screen.printAt(220, 160, "Intake2: %3.f", Intake2.temperature(celsius));

    /* Clear the terminal by using system("cls") */
    // system("cls");
    // printf("L1: %.3f\n", L1.temperature(celsius));
    // printf("L2: %.3f\n", L2.temperature(celsius));
    // printf("L3: %.3f\n", L3.temperature(celsius));
    // printf("R1: %.3f\n", R1.temperature(celsius));
    // printf("R2: %.3f\n", R2.temperature(celsius));
    // printf("R3: %.3f\n", R3.temperature(celsius));
}
