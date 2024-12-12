#include "vex.h"
#include "robot-config.h"
#include "DriverFunction/DriverFunction.h"

using namespace vex;

void driverClass::elevationTime() {
    if (elevation.value() == 95) {
        Controller.rumble("- - -");  // Trigger controller rumble with pattern "- - -"
    }

    if (elevation.value() == 100) {
        Controller.rumble("- - -"); // Trigger controller rumble with pattern "- - -"
    }
}