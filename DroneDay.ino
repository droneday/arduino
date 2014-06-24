#include <SoftwareSerial.h>

#include "I2Cdev.h"

#include "controller.h"
#include "estimator.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

int MOTOR_PIN_FRONT = 3;
int MOTOR_PIN_REAR = 10;
int MOTOR_PIN_LEFT = 5;
int MOTOR_PIN_RIGHT = 11;


#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

void setup() {
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    // turn it on here, so we know that if it stays solid, the program hung during setup
    digitalWrite(LED_PIN, HIGH);
    
    // turn off motors
    analogWrite(MOTOR_PIN_FRONT, 0);
    analogWrite(MOTOR_PIN_REAR, 0);
    analogWrite(MOTOR_PIN_LEFT, 0);
    analogWrite(MOTOR_PIN_RIGHT, 0);
    

    // initialize serial communication
    Serial.begin(38400);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    initialize_imu();

    calibrate_imu();
    
    delay(6000);

    // TODO:
    //bluetoothSetup();
/*
float x,y,z;
    quat2rpy(rpy2quat(0.1, .2, .3), &x, &y, &z);
    Serial.println(x);
        Serial.println(y);
            Serial.println(z);
            */
}


void loop() {
    static int lastTime = millis();
    int thisTime = millis();
    float dt = float(thisTime - lastTime)/1000;
    lastTime = thisTime;
  
    State measured = readIMU(dt);
    
    //CommandedState command = readBluetooth();
    CommandedState command = {0, 0, 0, 25};
    
    MotorPower pwm;
    if(millis()/1000 < 12){
      pwm = controller(dt, measured, command);
    }
    else{
      pwm.front=0; pwm.rear=0; pwm.left=0; pwm.right=0;
    }

    analogWrite(MOTOR_PIN_FRONT, pwm.front);
    analogWrite(MOTOR_PIN_REAR,  pwm.rear);
    analogWrite(MOTOR_PIN_LEFT,  pwm.left);
    analogWrite(MOTOR_PIN_RIGHT, pwm.rear);

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    

}



