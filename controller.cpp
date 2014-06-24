#include "Arduino.h"
#include "controller.h"

// parameters (set elsewhere?)
float Kp_pitchroll = 8.0;
float Ki_pitchroll = 0.0;
float Kd_pitchroll = 0.0;

float Kp_yawrate = 0.0;
float Ki_yawrate = 0.0;
float Kd_yawrate = 0.0;

float i_error_pitchroll_max = 0.1;
float i_error_yaw_rate_max = 0.1;

MotorPower controller(float dt, State measured, CommandedState command)
{
  // calculate proportional errors
  static State p_error;
  p_error.roll = command.roll - measured.roll;
  p_error.pitch = command.pitch - measured.pitch;
  p_error.yaw_rate = command.yaw_rate - measured.yaw_rate;
  
  Serial.print("error = ");
  Serial.print(p_error.pitch);
 
  // make some statics to keep track of values between calls
  static State i_error = {0,0,0};
  static State last_p_error = {0,0,0};
  
  // calculate integral errors
  i_error.roll += p_error.roll;
  i_error.pitch += p_error.pitch;
  i_error.yaw_rate += p_error.yaw_rate;
  
  // limit windup
  i_error.roll = constrain(i_error.roll, -i_error_pitchroll_max, i_error_pitchroll_max);
  i_error.pitch = constrain(i_error.pitch, -i_error_pitchroll_max, i_error_pitchroll_max);
  i_error.yaw_rate = constrain(i_error.yaw_rate, -i_error_yaw_rate_max, i_error_yaw_rate_max);
  
  // calculate derivative errors
  State d_error;
  d_error.roll  = (p_error.roll - last_p_error.roll) / dt;
  d_error.pitch = (p_error.pitch - last_p_error.pitch) / dt;
  d_error.yaw_rate = (p_error.yaw_rate - last_p_error.yaw_rate) / dt;
  
  // multiply by gains and sum PID error terms
  float out_roll     = Kp_pitchroll*p_error.roll   + Ki_pitchroll*i_error.roll   + Kd_pitchroll*d_error.roll;
  float out_pitch    = Kp_pitchroll*p_error.pitch  + Ki_pitchroll*i_error.pitch  + Kd_pitchroll*d_error.pitch;
  float out_yaw_rate = Kp_yawrate*p_error.yaw_rate + Ki_yawrate*i_error.yaw_rate + Kd_yawrate*i_error.yaw_rate;
  
  Serial.print("\tout_pitch = ");
  Serial.print(out_pitch);
  
  // initialize motor thrust at desired throttle
  float thrust_front = command.throttle;
  float thrust_rear  = command.throttle;
  float thrust_left  = command.throttle;
  float thrust_right = command.throttle;
  
  // in right-forward-up coordinate frame, positive roll increases thrust of front motor and decreases thrust of rear motor
  thrust_front += out_roll;
  thrust_rear  -= out_roll;
  
  // positive pitch increases thrust of left motor and decreases thrust of right motor
  thrust_left  += out_pitch;
  thrust_right -= out_pitch;

  // positive yaw rate (clockwise) increases thrust of CCW motors and decreases thrust of CW motors
  thrust_front += out_yaw_rate;
  thrust_rear  += out_yaw_rate;
  thrust_left  -= out_yaw_rate;
  thrust_right -= out_yaw_rate;
  
  // scale thrust to PWM duty cycle and constrain to PWM limits
  float scale = 1.0;
  MotorPower pwm;
  pwm.front = constrain(scale * thrust_front, 0, 255);
  pwm.rear  = constrain(scale * thrust_rear,  0, 255);
  pwm.left  = constrain(scale * thrust_left,  0, 255);
  pwm.right = constrain(scale * thrust_right, 0, 255);
  
  Serial.print("\tR/L thrust = ");
  Serial.print(pwm.right);
  Serial.print("\t");
  Serial.println(pwm.left);
  
  return pwm;
}

