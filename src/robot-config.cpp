#include "vex.h"
using namespace vex;
using signature = vision::signature;
using code = vision::code;

brain Brain;

motor L1 = motor(PORT1, ratio6_1, true);
motor L2 = motor(PORT2, ratio6_1, false);
motor L3 = motor(PORT1, ratio6_1, true);

motor R1 = motor(PORT10, ratio6_1, false);
motor R2 = motor(PORT12, ratio6_1, true);
motor R3 = motor(PORT13, ratio6_1, false);

encoder Encoder = encoder(Brain.ThreeWirePort.B);

motor Intake = motor(PORT16, ratio18_1, true);

inertial Inertial = inertial(PORT15);
distance Distance = distance(PORT2);

controller Controller = controller(primary);

motor_group LeftMotor(L1, L2, L3);
motor_group RightMotor(R1, R2, R3);
void vexcodeInit(void) {}