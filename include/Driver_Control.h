#pragma once
#include "vex.h"
#include "robot-config.h"

using namespace vex;

/**
 * @brief Main control function to manage robot operations based on controller inputs.
 * 
 * This function resets elevation, assigns actions to controller buttons, and runs continuous operations.
 */
void Control();