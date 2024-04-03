#include "vex.h"
#include "robot-config.h"
#include "Auto_function/Auto_class.h"

using namespace vex;

/*Convert angle over 180 to a negative */
double Auto_class::heading_convert(double heading);

/*Stop the robot with the brake type mode.
Enter (brakeType = brake, bold, coast)*/
void Auto_class::motor_stop(brakeType mode);

/*Spin the robot with the speed and velocity units.
Enter (L_speed = Left motor speed), (R_speed = Right motor speed), (velocityUnit = the speed unit of first two integer)*/
void Auto_class::motor_spin(double L_speed,double R_speed, voltageUnits units);

/*Move in second to ensure the robot can push triball in to the goal.
Enter (direction = fwd, reverse), (velocity = velocity in percentage), (second_to_move = move how much time), (brakeType = brake, hold, coast)*/
void Auto_class::move_full_speed(int velocity, double second_to_move, brakeType mode);

/*In centimeter move curved or straight path. Using inertial sensor to ensure the robot move to correct position.
Enter (disTile = Distance in Cm), (speed_max = maximum speed of action), (Target = angle to turn), (kp = sharpness to turn), (ratio =in persentage when to turn)).*/
void Auto_class::move_Tile(float disTile, int speed_max, double target = 0, float kp = 0, float ratio_to_turn = 0);

/*Turn with inertial sensor to ensure the robot get the correct angle.
Enter (degrees to turn)*/
void Auto_class::inertial_turn(float turn_degree);