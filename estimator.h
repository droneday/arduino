#pragma once
#include "helper_3dmath.h"

struct State
{
  float roll;
  float pitch;  
  float yaw_rate;
};

void initialize_imu();
void calibrate_imu();

State readIMU(float dt);

void printrpy(float roll, float pitch, float yaw);
void printQuaternion(Quaternion q);

Quaternion rpy2quat(float x, float y, float z);
void quat2rpy(Quaternion q, float* roll, float* pitch, float* yaw);


