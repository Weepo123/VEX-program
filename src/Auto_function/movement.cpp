#include "vex.h"
#include "robot-config.h"
#include "Auto_function/Auto_class.h"

using namespace vex;

double Auto_class::heading_convert(double heading){
  return(heading > 180) ? heading - 360 : heading;
  /*twenRY Operator A = (condition) ? (true data) : (false data)*/
}

void Auto_class::motor_stop(brakeType mode){
  Left_motor.stop(mode);
  Right_motor.stop(mode);
}

void Auto_class::motor_spin(double L_speed, double R_speed, velocityUnits units){
  Left_motor.spin(fwd, L_speed, units);
  Right_motor.spin(fwd, R_speed, units);
}

void Auto_class::move_full_speed(int velocity, double second_to_move, brakeType mode){
  Left_motor.spin(fwd, velocity, rpm);
  Right_motor.spin(fwd, velocity, rpm);
  wait(second_to_move, sec);
  motor_stop(mode);
}

void Auto_class::move_Tile(float disTile, int speed_max, double target, float kp, float ratio_to_turn){
  float speed_now = 0;
  double position_now = 0;
  int direction = 1;

  float t1, t2, t3; // second
  float Vmax; // degrees / second
  float Vmax_rpm;
  float acceleration_per_degrees = 1800; // degrees / second^2

  //resest Position
  Left_motor.resetPosition();
  Right_motor.resetPosition();

  /*    Target_deg = Distance / ((wheel's perimeter) * (gear ratio) * (complete angle)*/
  float target_deg = disTile * (24.0 / 1.0) * (1.0 / (pi * 3.5)) * (84.0 / 1.0) * (1.0 / 48.0) * (360.0 / 1.0);

  Vmax = fmin(sqrt(target_deg * acceleration_per_degrees),  speed_max * 6 ); // calculaate real Vmax

  t1 = Vmax / acceleration_per_degrees;//(Velocity maximun) / (degrees / second ^ 2)
  t2 = target_deg / Vmax;//(theoretical distance) / (Velocity maximun)
  t3 = t1 + t2;//(increase end time) + (decrease start time) = (end time)

  Vmax_rpm = Vmax / 6;

  if (disTile < 0){
    direction = -1;
  }
  
  float ki = 0.0000005;
  float kd = 0;

  float turn_error = 0;
  float prev_error = 0;
  float INT = 0;

  double speed_turn = 0;

  timer move_timeout;
  timer Timer;
  
  while (position_now < fabs(target_deg) && move_timeout.value() <= 5){
    // calculate speed
    if (Timer.value() < t1)
    {
      speed_now = cos(Timer.value() / t1 * pi + pi) * Vmax_rpm / 2 + Vmax_rpm / 2;
    }
    else if (Timer.value() > t2)
    {
      speed_now = fmax(cos((Timer.value() - t2) / t1 * pi) * Vmax_rpm / 2 + Vmax_rpm / 2, 300);
      /*it is possible that there have error between real distance and theoretical distance*/
    }
    else{
      speed_now = speed_now;
    }

    // Inertial Target Calibration
    turn_error = target - Inertial.heading();
    INT = INT + turn_error;

    if(position_now / fabs(target) > ratio_to_turn){
      speed_turn = (turn_error) * kp  + INT * ki + (turn_error - prev_error) * kd;
    }
    else{
      speed_turn = 0;
    }

    motor_spin(direction * speed_now + speed_turn, direction * speed_now - speed_turn, rpm);
    wait(10, msec);

    position_now = fabs(((L1.position(deg) + L2.position(deg) + L3.position(deg)) + (R1.position(deg) + R2.position(deg) + R3.position(deg))) / 6);
    if(turn_error > 0.25){
      motor_stop(hold);
    }
    prev_error = turn_error;
  }
  motor_stop(hold);
}

void Auto_class::inertial_turn(float turn_degree){
  float speed_now = 0;

  float kp = 3;
  float ki = 0.00000005;
  float kd = 0;

  float error = 0;
  float prev_error = 0;
  float INT = 0;

  int counter = 0;

  timer timeout;
  
  while (counter < 1 && timeout.value() <= 3){

    error = turn_degree - Inertial.heading();
    INT = INT + error;
    speed_now = error * kp + INT * ki + (error - prev_error) * kd;

    motor_spin(speed_now, -speed_now, rpm);

    prev_error = error;

    if (fabs(Inertial.heading() - turn_degree) < 0.5){
      counter++;
    }
    else{
      counter = 0;
    }
    wait(10, msec);
  }
  motor_stop(hold);
}