#include "I2Cdev.h"

#include "MPU6050.h" // not necessary if using MotionApps include file
#include "helper_3dmath.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

void setup() {
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    // turn it on here, so we know that if it stays solid, the program hung during setup
    digitalWrite(LED_PIN, HIGH);
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(38400);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing MPU6050..."));
    mpu.initialize();
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  
    calibrate_imu();
/*
float x,y,z;
    quat2rpy(rpy2quat(0.1, .2, .3), &x, &y, &z);
    Serial.println(x);
        Serial.println(y);
            Serial.println(z);
            */
}

void printrpy(float roll, float pitch, float yaw)
{
    Serial.print("ypr\t");
    Serial.print(yaw*180/M_PI);
    Serial.print("\t");
    Serial.print(pitch*180/M_PI);
    Serial.print("\t");
    Serial.println(roll*180/M_PI);
}

void printQuaternion(Quaternion q)
{
  Serial.print("q(");
  Serial.print(q.w);
  Serial.print(", ");
  Serial.print(q.x);
  Serial.print(", ");
  Serial.print(q.y);
  Serial.print(", ");
  Serial.print(q.z);
  Serial.println(")");
}

float gyro_offset[3];
void calibrate_imu()
{
    // zupt gyros
    Serial.println("Calibrating gyros");
    
    int16_t thisgyro[3];
    int32_t sum[3] = {0,0,0};
    int N = 1024;
    for(int i = 0; i < N; i++){
      mpu.getRotation(&thisgyro[0], &thisgyro[1], &thisgyro[2]);
      for(int j = 0; j < 3; j++){
        sum[j] += thisgyro[j];
      }
    }
    
    for(int j = 0; j < 3; j++){
      gyro_offset[j] = (float)sum[j] / N;
    }
    
    Serial.print("gyros offsets are ");
    Serial.print(gyro_offset[0]);
    Serial.print(" ");
    Serial.print(gyro_offset[1]);
    Serial.print(" ");
    Serial.println(gyro_offset[2]);
}

float yaw=0, pitch=0, roll=0;
void loop() {

    readIMU();
    
    /*
    static unsigned char count = 0;
    if(!count++)
    {
      static int lastTime = millis();
      int thisTime = millis();
      Serial.print(256./((thisTime-lastTime)/1000.));
      Serial.println(" Hz");
      lastTime = thisTime;
    }
    */

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}

Quaternion rpy2quat(float x, float y, float z)
{
  Quaternion qx(cos(x/2), sin(x/2), 0, 0);
  Quaternion qy(cos(y/2), 0, sin(y/2), 0);
  Quaternion qz(cos(z/2), 0, 0, sin(z/2));
  Quaternion q = qx.getProduct(qy.getProduct(qz));
  return q;
}

// convert to YPR   
void quat2rpy(Quaternion q, float* roll, float* pitch, float* yaw)
{
//  *roll = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
//  *pitch = asin(-2.0*(q.x*q.z - q.w*q.y));
//  *yaw = atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
  
  q.x = -q.x; q.y = -q.y; q.z = -q.z;
  *roll = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
  *pitch = asin(-2.0*(q.x*q.z - q.w*q.y));
  *yaw = atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
  *yaw = -*yaw; *pitch = -*pitch; *roll = -*roll;
}

void readIMU()
{
  // over what period are we integrating?
  static int lastTime = millis();
  int thisTime = millis();
  float dt = float(thisTime - lastTime)/1000;
  lastTime = thisTime;
  
  // get body-frame rotational velocities and linear accelerations
  int16_t accel[3], gyro[3];
  mpu.getMotion6(&accel[0], &accel[1], &accel[2], &gyro[0], &gyro[1], &gyro[2]);
  
  // get this-axis rotation in radians
  // 131 LSB/deg/sec, pi/180 rad/deg
  float xRot = (gyro[0] - gyro_offset[0]) / 131 * M_PI / 180 * dt;
  float yRot = (gyro[1] - gyro_offset[1]) / 131 * M_PI / 180 * dt;
  float zRot = (gyro[2] - gyro_offset[2]) / 131 * M_PI / 180 * dt;
  float xAcc = accel[0] / 8192.;
  float yAcc = accel[1] / 8192.;
  float zAcc = accel[2] / 8192.;
  
  //Serial.print("\nRotated by ");
  //printrpy(xRot, yRot, zRot);
  
  // convert to quaternions
  Quaternion fullOrientation = rpy2quat(roll, pitch, yaw);
  Quaternion thisRotation = rpy2quat(xRot, yRot, zRot);
  
  //Serial.print(" = ");
  //printQuaternion(thisRotation);
  //Serial.print("old orientation ");
  //printQuaternion(fullOrientation);
  
  // rotate orientation by this rotation
  fullOrientation = thisRotation.getProduct(fullOrientation);
  fullOrientation.normalize();
  
  //Serial.print("new orientation ");
  //printQuaternion(fullOrientation);
  
  // convert back to roll pitch yaw
  quat2rpy(fullOrientation, &roll, &pitch, &yaw);
  
  // accel
  float accel_pitch = -atan2(xAcc, zAcc);
  float accel_roll = atan2(yAcc, zAcc);

  // downweight accel measurement if the thing is accelerating (forces other than just gravity)
  float accelMag = sqrt(xAcc*xAcc + yAcc*yAcc + zAcc*zAcc);
  float accelFrac = 0.02 * exp(-4*abs(1 - accelMag));
  //Serial.print(" accelFrac ");
  //Serial.println(accelFrac);
  
  // fuse gyro integration with low-pass accel
  pitch = pitch*(1-accelFrac) + accel_pitch*accelFrac;
  roll = roll*(1-accelFrac) + accel_roll*accelFrac;
  
  printrpy(roll, pitch, yaw);
}
