#include <stdint.h>

float Kp_pitchroll = 1.0;
float Ki_pitchroll = 0.0;
float Kd_pitchroll = 0.0;

float Kp_yawrate = 1.0;
float Ki_yawrate = 0.0;
float Kd_yawrate = 0.0;

void controller(float dt, float measured_roll, float measured_pitch,
                float measured_roll_rate, float measured_pitch_rate, float measured_yaw_rate,
                float command_roll, float command_pitch, float command_yaw_rate, float command_throttle,
                uint8_t* pwm_FL, uint8_t* pwm_FR, uint8_t* pwm_RL, uint8_t* pwm_RR)
{ 
  // calculate proportional errors
  float p_error_roll = command_roll - measured_roll;
  float p_error_pitch = command_pitch - measured_pitch;
  float p_error_yaw_rate = command_yaw_rate - measured_yaw_rate;
 
  // make some static to keep track of values between calls
  static float i_error_roll = 0;
  static float i_error_pitch = 0;
  static float i_error_yaw_rate = 0;
  static float last_command_roll = command_roll;
  static float last_command_pitch = command_pitch;
  static float last_p_error_yaw_rate = 0;
  
  // calculate integral errors
  i_error_roll += p_error_roll;
  i_error_pitch += p_error_pitch;
  i_error_yaw_rate += p_error_yaw_rate;
  
  // calculate derivative errors
  float d_command_roll  = (command_roll - last_command_roll) / dt;
  float d_command_pitch = (command_pitch - last_command_pitch) / dt;
  float d_error_roll  = d_command_roll - measured_roll_rate;
  float d_error_pitch = d_command_pitch - measured_pitch_rate;
  float d_error_yaw_rate = (p_error_yaw_rate - last_p_error_yaw_rate) / dt;
  
  // multiply by gains and sum PID error terms
  float out_roll     = Kp_pitchroll*p_error_roll   + Ki_pitchroll*i_error_roll   + Kd_pitchroll*d_error_roll;
  float out_pitch    = Kp_pitchroll*p_error_pitch  + Ki_pitchroll*i_error_pitch  + Kd_pitchroll*d_error_pitch;
  float out_yaw_rate = Kp_yawrate*p_error_yaw_rate + Ki_yawrate*i_error_yaw_rate + Kd_yawrate*i_error_yaw_rate;
  
  // initialize motor thrust at desired throttle
  float thrust_FL = command_throttle;
  float thrust_FR = command_throttle;
  float thrust_RL = command_throttle;
  float thrust_RR = command_throttle;
  
  // in forward-right-down coordinate frame, positive roll increases thrust of left motors and decreases thrust of right motors
  thrust_FL += out_roll;
  thrust_FR -= out_roll;
  thrust_RL += out_roll;
  thrust_RR -= out_roll;
  
  // positive pitch increases thrust of front motors and decreases thrust of rear motors
  thrust_FL += out_pitch;
  thrust_FR += out_pitch;
  thrust_RL -= out_pitch;
  thrust_RR -= out_pitch;
  
  // positive yaw rate (clockwise) increases thrust of CCW motors and decreases thrust of CW motors
  thrust_FL -= out_yaw_rate;
  thrust_FR += out_yaw_rate;
  thrust_RL += out_yaw_rate;
  thrust_RR -= out_yaw_rate;
  
  // scale thrust to PWM duty cycle
  float scale = 1.0;
  *pwm_FL = scale * thrust_FL;
  *pwm_FR = scale * thrust_FR;
  *pwm_RL = scale * thrust_RL;
  *pwm_RR = scale * thrust_RR;
}

