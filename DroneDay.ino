#include <MemoryFree.h>
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

//#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_TEAPOT



#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus = 3;  // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

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
    //devStatus = mpu.dmpInitialize();
    
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.print(F("packetSize1 = "));
    Serial.println(mpu.dmpPacketSize);
    
    if(devStatus != 0)
    {
       // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
      
      // spin in error loop blinking LED until reset
      while(true)
      {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(250);
      }
    }
    
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.print(F("packetSize2 = "));
    Serial.println(mpu.dmpPacketSize);
    
    calibrate_imu();

    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
    
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.print(F("packetSize4 = "));
    Serial.println(mpu.dmpPacketSize);

    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING); // INT0 is pin D2 on ATMEGA328P
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.print(F("packetSize5 = "));
    Serial.println(mpu.dmpPacketSize);
    packetSize=42;
}

void calibrate_imu()
{
    Serial.print("Offsets were: ");
    Serial.print(mpu.getXGyroOffset());
    Serial.print(", ");
    Serial.print(mpu.getYGyroOffset());
    Serial.print(", ");
    Serial.print(mpu.getZGyroOffset());
    
    // zupt gyros
    Serial.println("Calibrating gyros");
    
    int16_t thisgyro[3];
    int32_t avg[3] = {0,0,0};
    int N = 600;
    for(int i = 0; i < N; i++){
      mpu.dmpGetGyro(thisgyro);
      for(int j = 0; j < 3; j++){
        avg[j] += thisgyro[j];
      }
      delay(1000);
      Serial.print("gyros ");
      Serial.print(thisgyro[0]);
      Serial.print(" ");
      Serial.print(thisgyro[1]);
      Serial.print(" ");
      Serial.println(thisgyro[2]);
    }
    
    Serial.print(F("packetSize2.5 = "));
    Serial.println(mpu.dmpPacketSize);
    
    for(int j = 0; j < 3; j++){
      avg[j] /= N;
    }
    
    mpu.setXGyroOffset((int16_t)avg[0]);
    mpu.setYGyroOffset((int16_t)avg[1]);
    mpu.setZGyroOffset((int16_t)avg[2]);
      
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.print(F("packetSize3 = "));
    Serial.println(mpu.dmpPacketSize);

    Serial.print("Offsets are now: ");
    Serial.print(mpu.getXGyroOffset());
    Serial.print(", ");
    Serial.print(mpu.getYGyroOffset());
    Serial.print(", ");
    Serial.print(mpu.getZGyroOffset());
    //Serial.print(", ");
    //Serial.print(mpu.getXAccelOffset());
    //Serial.print(", ");
    //Serial.print(mpu.getYAccelOffset());
    //Serial.print(", ");
    //Serial.print(mpu.getZAccelOffset());
    Serial.println();
    
      for(int i = 0; i < 50; i++){
        mpu.dmpGetGyro(thisgyro);
        Serial.print("gyros ");
        Serial.print(thisgyro[0]);
        Serial.print(" ");
        Serial.print(thisgyro[1]);
        Serial.print(" ");
        Serial.println(thisgyro[2]);
      }
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }
    // if we've exited the loop, we have an IMU packet to process
    readIMU();

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}

void readIMU()
{

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif
    } 
}
