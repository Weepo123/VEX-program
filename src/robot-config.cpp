#include "vex.h"
using namespace vex;
using signature = vision::signature;
using code = vision::code;

brain Brain;

motor puncher_right = motor(PORT5, ratio36_1, false);
motor puncher_left = motor(PORT6, ratio36_1, true);

motor L1 = motor(PORT19, ratio6_1, false);
motor L2 = motor(PORT18, ratio6_1, true);
motor L3 = motor(PORT20, ratio6_1, false);

motor R1 = motor(PORT11, ratio6_1, true);
motor R2 = motor(PORT12, ratio6_1, false);
motor R3 = motor(PORT13, ratio6_1, true);

motor Intake = motor(PORT16, ratio18_1, true);

digital_out Front_wings1 = digital_out(Brain.ThreeWirePort.H );
digital_out Back_wings_L = digital_out(Brain.ThreeWirePort.D);
digital_out Back_wings_R = digital_out(Brain.ThreeWirePort.F);

inertial Inertial = inertial(PORT15);
distance Distance = distance(PORT21);

controller Controller = controller(primary);

motor_group Left_motor(L1, L2, L3);
motor_group Right_motor(R1, R2, R3);
motor_group puncher_motor(puncher_left, puncher_right);
void vexcodeInit(void) {}