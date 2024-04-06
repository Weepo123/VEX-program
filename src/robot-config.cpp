#include "vex.h"
using namespace vex;
using signature = vision::signature;
using code = vision::code;

brain Brain;

motor puncher = motor(PORT5, ratio36_1, false);

motor L1 = motor(PORT19, ratio6_1, true);
motor L2 = motor(PORT18, ratio6_1, false);
motor L3 = motor(PORT20, ratio6_1, true);

motor R1 = motor(PORT11, ratio6_1, false);
motor R2 = motor(PORT12, ratio6_1, true);
motor R3 = motor(PORT13, ratio6_1, false);

motor Intake = motor(PORT16, ratio18_1, true);

digital_out Back_wings1 = digital_out(Brain.ThreeWirePort.H );
digital_out Front_wings_L = digital_out(Brain.ThreeWirePort.D);
digital_out Front_wings_R = digital_out(Brain.ThreeWirePort.F);

inertial Inertial = inertial(PORT15);
distance Distance = distance(PORT21);

controller Controller = controller(primary);

motor_group Left_motor(L1, L2, L3);
motor_group Right_motor(R1, R2, R3);
void vexcodeInit(void) {}