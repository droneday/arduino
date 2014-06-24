/////////////////////////////////// ESTIMATOR.H //////////////////////////

struct State
{
  float roll;
  float pitch;  
  float yaw_rate;
};

////////////////////////////////////////////////////////////////////////////

struct MotorPower
{
  uint8_t front;
  uint8_t rear;
  uint8_t left;
  uint8_t right;
};

struct CommandedState
{
  float roll;
  float pitch;
  float yaw_rate;
  float throttle;
};

MotorPower controller(float dt, State measured, CommandedState command);
