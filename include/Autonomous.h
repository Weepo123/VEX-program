#pragma once
#include "vex.h"
#include "robot-config.h"

using namespace vex;

void intakeTaskFunction();

void intakeSecondRing();

void intakeSpin();
/**
 * @brief Autonomous function to execute predefined movements and actions.
 * 
 * This function controls the robot's actions autonomously based on predefined logic and sensor inputs.
 */
void Autonomous();