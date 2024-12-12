#include "vex.h"
#include "robot-config.h"
#include "AutoFunction/movement.h"

#include <iostream>
#include <thread>

using namespace vex;

bool redirectRing = false;

//useless, like jonathan
void intakeTaskFunction() {
    while(true){
        if(Distance.objectDistance(inches) < 1.3 && redirectRing) {
            wait(45, msec);
            Intake.spin(fwd, -100, pct);
            wait(800, msec);
            Intake.stop(coast);
            redirectRing = false;
        }
    }
}
void intakeSecondRing() {
    Intake.spin(fwd, -8.5, volt);
    wait(0.25, sec);
    Intake2.spin(fwd, 12, volt);
    wait(4, sec);
    Intake.stop(brake);
    
}


void Autonomous(){
    autoClass auton;
    vex::thread intakeTask(intakeTaskFunction);
    // Intake.spin(fwd, 100, pct);
    // wait(0.2, sec);
    // Intake.stop(coast);
    // auton.moveTurnTileWithPID(0.7);
    // auton.turn(90);
    // auton.moveTurnTileWithPID(-0.9, Inertial.rotation(), 200, 600, 0.0);
    // Clamp.open();
    // auton.turn(0);
    // Intake.spin(fwd, 12, volt);
    // auton.moveTurnTileWithPID(0.8);
    // auton.turn(45);
    // auton.moveTurnTileWithPID(2.9, Inertial.rotation(), 450);
    // wait(0.5, sec);
    // auton.turn(90);
    // Intake1.stop(coast);
    // Intake1.spin(fwd, 30, pct);
    // wait(0.5, sec);
    // auton.moveTurnTileWithPID(1.25);
    // redirectRing = true;
    // wait(0.75, sec);
    // auton.turn(180);
    // Redirect.open();
    // redirectRing = false;
    // Intake2.spin(fwd, 12, volt);
    // auton.moveTurnTileWithPID(1.05, Inertial.rotation(), 450);
    // auton.turn(90);
    // auton.motorSpin(600, 600, rpm);
    // wait(0.75, sec);
    // auton.motorStop(brake);
    // Redirect.close();
    // wait(0.5, sec);
    // auton.moveTurnTileWithPID(-0.85);
    // Intake.spin(fwd, 12, volt);
    // auton.turn(180);
    // auton.moveTurnTileWithPID(1);
    // auton.turn(140);
    // auton.moveTurnTileWithPID(1.35);
    // wait(0.75, sec);
    // auton.moveTurnTileWithPID(-0.75);
    // auton.turn(-30);
    // auton.moveTurnTileWithPID(-1.75);
    // Intake.stop(coast);
    // Intake2.spin(fwd, 12, volt);
    // auton.turn(-35);
    // auton.moveTurnTileWithPID(-1);
    // Clamp.close();

    Intake.spin(fwd, 12, volt);
    wait(0.2, sec);
    Intake.stop(coast);
    auton.moveTurnTileWithPID(0.7);
    auton.turn(90);
    auton.moveTurnTileWithPID(-0.9, 90, 200, 600, 0.0);
    Clamp.open();
    auton.turn(0);
    Intake.spin(fwd, 12, volt);
    auton.moveTurnTileWithPID(0.8);
    auton.turn(-45.0);
    auton.moveTurnTileWithPID(1.8, -45, 600, 600, 0.0, 1.25);
    auton.moveTurnTileWithPID(-0.45, -45, 600, 600, 0.0, 1);
    wait(0.5, sec);
    auton.turn(-180);
    auton.moveTurnTileWithPID(2.25, -180, 300);
    wait(1, sec);
    auton.turn(-45);
    auton.moveTurnTileWithPID(0.5, -45, 600, 600, 0.0, 0.75);
    auton.turn(45);
    auton.moveTurnTileWithPID(-0.5, 45, 600, 600, 0.0, 0.75);
    Clamp.close();
    Intake.stop(brake);
    Intake.spin(fwd, -8.5, volt);
    auton.moveTurnTileWithPID(0.675, 45);
    auton.turn(-90); 
    auton.moveTurnTileWithPID(-3, -90, 300, 300, 0.0, 2);
    Clamp.open();
    auton.turn(0);
    Intake.spin(fwd, 12, volt);
    auton.moveTurnTileWithPID(0.8, 0);
    auton.turn(45);
    auton.moveTurnTileWithPID(1.7, 45, 600, 600, 0.0, 1.25);
    auton.moveTurnTileWithPID(-0.45, 45, 600, 600, 0.0, 0.35);
    wait(0.5, sec);
    auton.turn(180);
    auton.moveTurnTileWithPID(2.25, 180, 300);
    wait(0.5, sec);
    auton.turn(45);
    auton.moveTurnTileWithPID(0.6, 45, 600, 600, 0.0, 0.75);
    auton.moveTurnTileWithPID(-0.3, 45);
    auton.turn(-45);
    auton.moveTurnTileWithPID(-0.5, -45, 600, 600, 0.0, 0.75);
    Clamp.close();
    Intake.stop(brake);
    vex::thread intakeRings(intakeSecondRing);
    auton.moveTurnTileWithPID(4.55, -45, 400, 600, 0.0, 6);
    wait(0.75, sec);
    Intake2.spin(fwd, 12, volt);
    auton.turn(-135);
    auton.moveTurnTileWithPID(-1.65, -135, 300, 600, 0.0, 1.3);
    Clamp.open();
    Intake.spin(fwd, 12,volt);
    auton.turn(-90);
    auton.moveTurnTileWithPID(2.25, -90, 300);
    auton.turn(22.5);
    auton.moveTurnTileWithPID(1, 22.5);
    auton.turn(180);
    auton.moveTurnTileWithPID(1.5, 180);
    Intake.spin(fwd, -12, volt);
    auton.turn(157.5);
    Intake.spin(fwd, 12, volt);
    auton.moveTurnTileWithPID(-1.65, 157.5);
    Clamp.close();
    auton.moveTurnTileWithPID(1, 157.5);
    auton.turn(90);
    auton.moveTurnTileWithPID(3, 135, 500, 450, 0.75, 4.0);
}

