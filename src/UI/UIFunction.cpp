#include "vex.h"
#include "robot-config.h"
#include "UI/UIFunction.h"

bool Temperature = false;
void brainUI(){
        brainGraphic();
        motorTemperature();
        inertialRotation();
        brainUIButton();
        wait(10, msec);
}

void motorTemperature(){
    if(Temperature){
    Brain.Screen.printAt(220, 20, "L1: %.3f", L1.temperature(celsius));
    Brain.Screen.printAt(220, 40, "L2: %.3f", L2.temperature(celsius));
    Brain.Screen.printAt(220, 60, "L3: %.3f", L3.temperature(celsius));
    Brain.Screen.printAt(220, 80, "R1: %.3f", R1.temperature(celsius));
    Brain.Screen.printAt(220, 100, "R2: %.3f", R2.temperature(celsius));
    Brain.Screen.printAt(220, 120, "R3: %.3f", R3.temperature(celsius));
    Brain.Screen.printAt(220, 140, "Intake1: %3.f", Intake1.temperature(celsius));
    Brain.Screen.printAt(220, 140, "Intake2: %3.f", Intake2.temperature(celsius));
    }
}

void inertialRotation(){

}

void brainUIButton(){

}   

void brainGraphic(){
    Brain.Screen.setPenColor(red);
    Brain.Screen.setPenWidth(200);
    Brain.Screen.drawLine(500, 40, 300, 300);
    Brain.Screen.drawLine(500, 80, 340, 300);
    Brain.Screen.setPenColor(white);
    Brain.Screen.drawLine(340, 90, 480, 90);
    Brain.Screen.drawLine(340, 95, 480, 95);
    Brain.Screen.drawLine(320, 140, 480, 140);
    Brain.Screen.drawLine(320, 145, 480, 145);
    Brain.Screen.drawLine(300, 190, 480, 190);
    Brain.Screen.drawLine(300, 195, 480, 195);
    Brain.Screen.setPenColor(black);
    Brain.Screen.drawLine(460, 0, 280, 240);
    Brain.Screen.drawLine(430, 0, 240, 240);
    Brain.Screen.drawLine(400, 0, 210, 240);
    Brain.Screen.setPenWidth(0);
    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(360, 97, "Auton Select");
}