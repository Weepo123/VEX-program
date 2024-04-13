#include "vex.h"
#include "robot-config.h"

#include <string>
#include <iostream>

using namespace vex;

class Auto_class{
    public:
        Auto_class(){}

        /*Stop the robot with the brake type mode.
        Enter (brakeType = brake, bold, coast)*/
        void motor_stop(brakeType mode);

        /*Spin the robot with the speed and velocity units.
        Enter (L_speed = Left motor speed), (R_speed = Right motor speed), (velocityUnit = the speed unit of first two integer)*/
        void motor_spin(double L_speed,double R_speed, velocityUnits units);

        /*Move in second to ensure the robot can push triball in to the goal.
        Enter (direction = fwd, reverse), (velocity = velocity in rpm), (second_to_move = move how much time), (brakeType = brake, hold, coast)*/
        void move_full_speed(int velocity, double second_to_move, brakeType mode);

        /*In centimeter, move curved or straight path. Using inertial sensor to ensure the robot move to correct position.
        Enter (disCm = Distance in Cm), (speed_max = maximum speed of action), (Target = angle to turn), (kp = sharpness to turn), (ratio =in persentage when to turn)).*/
        void move_Tile(float disTile, int speed_max, double target = 0, float kp = 1, float ratio_to_turn = 0);

        /*Turn with inertial sensor to ensure the robot get the correct angle.
        Enter (degrees to turn)*/
        void turn(float turn_degree);

        /*Decide use whether Front_wings or Back_wings then type in the true or false to control the wing.
        Etner (Front, Back), (ture, false)*/
        void Wings(std::string direction, bool is_open);
        
    private:
        /*Convert angle over 180 to a negative */
        double rotation_convert(double rotation);
};