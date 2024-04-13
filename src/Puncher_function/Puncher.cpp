#include "vex.h"
#include "robot-config.h"

namespace{
    void resetPuncherFunction();
    void puncherOneRevolution();
    void spinPuncherToAngle(double degrees);

    int puncherRevolutionCount = 0;
    int punchedCount = 0;

    bool puncherSpinDebounce = false;
    bool isPuncherResetted = false;
    bool canPuncherRun = true;
}

void resetPuncher()
{
    if (isPuncherResetted){
        return;
    }
    task resetPuncherTask([] () -> int {
        resetPuncherFunction();
        return 1;
    });
}

void puncherThread(){
    // Detect balls
    punchedCount = 0;
    while (true){
        if(isPuncherResetted && canPuncherRun && Distance.objectDistance(mm) < 30) {
            puncherOneRevolution();
            punchedCount++;
        }
        task::sleep(10);
    }
}

int getPunchedCount(){
    return punchedCount;
}

namespace {
    void resetPuncherFunction(){
        isPuncherResetted = false;

        // Spin to slip part
        puncher.resetPosition();

        timer timeout;
        timeout.reset();

        // Nonslip to slip
        puncher.spin(fwd, 90, pct);
        timeout.reset();
        task::sleep(100);
        while (puncher.torque() > 0.10 && timeout.value() <= 2){
            task::sleep(10);
        }

        // Spin to the start of nonslip
        puncher.setMaxTorque(0.10, Nm);
        task::sleep(10);
        puncher.stop();

        // Spin catapult to bottom
        puncher.setMaxTorque(100, pct);
        puncher.resetPosition();
        spinPuncherToAngle(360.0 * (8.0 / 12.0));
        puncher.stop(hold);
        puncher.resetPosition();

        isPuncherResetted = true;
    }

    void puncherOneRevolution(){
        if (!puncherSpinDebounce){
            puncherSpinDebounce = true;

            puncherRevolutionCount++;
            spinPuncherToAngle(puncherRevolutionCount * 180);

            puncherSpinDebounce = false;
        }
    }
    
    void spinPuncherToAngle(double degrees){
        puncher.spin(fwd, 12, volt);
        timer runTimeout;
        while (puncher.position(deg) < degrees && runTimeout.value() < 0.5){
            task::sleep(10);
        }
        puncher.stop(hold);
    }
}