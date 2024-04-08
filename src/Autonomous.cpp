#include "vex.h"
#include "robot-config.h"
#include "Auto_function/Auto_class.h"
#include "Puncher_function/Puncher.h"

#include <iostream>
#include <thread>

using namespace vex;

void Autonomous(){
    Auto_class Auto;
    timer Auto_punch_time;
    resetPuncher();
    Intake.spin(fwd, -12, volt);
    Inertial.setHeading(180, degrees);
    Auto.move_Tile(1.5, 600, 90, 1.5, 0.4);
    Auto.move_Tile(-0.3, 600);
    Intake.stop(brake);
    Auto.turn(20);
    Auto.Wings("Back", true);
    puncher.spin(fwd, 12, volt);
    wait(25, sec);
    puncher.stop(coast);
    Auto.Wings("Back", false);
    Front_wings_L.set(true);
    Auto.move_Tile(1.95, 600);
    Auto.turn(90);
    Intake.spin(fwd, -12, volt);
    Auto.move_Tile(2.5, 600);
    Auto.move_Tile(-0.3, 600);
    Auto.Wings("Front", false);
    Auto.turn(0);
    Auto.move_Tile(-3.5, 600, 90, 1.5, 0.75);
    // Auto.move_Tile(-1.7, 600, 90, 2, 0.3);
    // Auto.move_Tile(0.35, 600);
    // Auto.turn(0);
    // Auto.move_Tile(2, 600);
    // Auto.turn(90);
    // Auto.move_Tile(-1.7, 600, 0, 2, 0.5);
    // Auto.move_Tile(1.7, 600, 90, 2, 0.5);
    // Auto.move_Tile(-3, 600);
    // Auto.turn(270);
    // Auto.move_Tile(-1.7, 600, 0, 2, 0.5);
    // Auto.move_Tile(1.7, 600, 270, 2, 0.5);
    // Auto.move_Tile(-1.5, 600);
    // Auto.turn(0);
    // Auto.move_Tile(-1.35, 600);
    // Auto.move_Tile(1.35, 600);
    // Auto.turn(65);
    // Auto.move_Tile(-2.5, 600);
    // Auto.turn(0);
    // Auto.move_Tile(-1.15, 600, 270, 2, 0.5);
    // Auto.move_Tile(0.5, 600);
}