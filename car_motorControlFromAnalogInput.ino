/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->	http://www.adafruit.com/products/1438
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor1->setSpeed(150);
  myMotor2->setSpeed(150);
  myMotor3->setSpeed(150);
  myMotor4->setSpeed(150);
  myMotor1->run(FORWARD);
  myMotor2->run(FORWARD);
  myMotor3->run(FORWARD);
  myMotor4->run(FORWARD);
  // turn on motor
  myMotor1->run(RELEASE);
  myMotor2->run(RELEASE);
  myMotor3->run(RELEASE);
  myMotor4->run(RELEASE);
}

void loop() {
  //Analog input from A0 and A1
  int sensorValue0 = analogRead(A0);  //value of A0 from 0 to 1023 (left side motors)
  int sensorValue1 = analogRead(A1);  //value of A1 from 0 to 1023 (right side motors)
  int deadZoneStart = 501;            //deadzone around halfway
  int deadZoneEnd = 521;

  //Check on which side of the deadzone the joystick is and control the motors accordingly
  if(sensorValue0 > deadZoneEnd) {
    int mappedValue0 = map(sensorValue0, deadZoneEnd, 1023, 0, 255); //map 521-1023 to 0-255
    myMotor1->run(FORWARD);
    myMotor3->run(FORWARD);
    myMotor1->setSpeed(mappedValue0);
    myMotor3->setSpeed(mappedValue0);
  } else if(sensorValue0 < deadZoneStart) {
    int mappedValue0 = map(sensorValue0, 0, deadZoneStart, 255, 0); //map 0-501 to 0-255
    myMotor1->run(BACKWARD);
    myMotor3->run(BACKWARD);
    myMotor1->setSpeed(mappedValue0);
    myMotor3->setSpeed(mappedValue0);
  } else {
    myMotor1->run(BACKWARD); //when in dead zone stop both motors
    myMotor3->run(BACKWARD);
    myMotor1->setSpeed(0);
    myMotor3->setSpeed(0);
  }

  if(sensorValue1 > deadZoneEnd) {
    int mappedValue1 = map(sensorValue1, deadZoneEnd, 1023, 0, 255);
    myMotor2->run(FORWARD);
    myMotor4->run(FORWARD);
    myMotor2->setSpeed(mappedValue1);
    myMotor4->setSpeed(mappedValue1);
  } else if(sensorValue1 < deadZoneStart) {
    int mappedValue1 = map(sensorValue1, 0, deadZoneStart, 255, 0);
    myMotor2->run(BACKWARD);
    myMotor4->run(BACKWARD);
    myMotor2->setSpeed(mappedValue1);
    myMotor4->setSpeed(mappedValue1);
  } else {
    myMotor2->run(BACKWARD);
    myMotor4->run(BACKWARD);
    myMotor2->setSpeed(0);
    myMotor4->setSpeed(0);
  }
  
  Serial.print("tick");

  Serial.println("A0: ");
  Serial.println(sensorValue0);
  Serial.println("A1: ");
  Serial.println(sensorValue1);
  Serial.println("---");
  
}
