#include "vex.h"
#include "robot-config.h"

using namespace vex;

class Driver_class{
    public:
        timer elevation;

        Driver_class(){}

        void Drivertain();
        
        void Drivertain_Temperature();

        void elevation_time();

        void Intake_spin();

    private:
};