#include "vex.h"
#include "robot-config.h"
#include "DriverFunction/DriverFunction.h"

using namespace vex;

void Driver_class::ElevationTime() {
    if (this->Elevation.value() == 75) {
        Controller.rumble("- - -");  // Trigger controller rumble with pattern "- - -"
    }
}