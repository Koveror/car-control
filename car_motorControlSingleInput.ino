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
  int sensorValue0 = analogRead(A0);  //value of A0 from 0 to 1023 (Y)
  int sensorValue1 = analogRead(A1);  //value of A1 from 0 to 1023 (X)
  int deadZoneStart = 501;            //deadzone around halfway
  int deadZoneEnd = 521;
  int inputIsLeft = 0;                //flag
  int inputIsRight = 0;               //flag
  int normalSpeed = 150;               //turning speed

  //Check X and make a flag
  if(sensorValue1 > deadZoneEnd) {
    inputIsLeft = 1;
  } else if(sensorValue1 < deadZoneStart) {
    inputIsRight = 1;
  } else {
    //no input from X
  }

  //Check on which side of the deadzone the Y input is and determine speed. Use X flags to control the motors accordingly
  if(sensorValue0 > deadZoneEnd) {
    int mappedValue0 = map(sensorValue0, deadZoneEnd, 1023, 0, 255); //map 521-1023 to 0-255
    if(inputIsLeft) {   //Run only the left side motors forwards
      myMotor1->run(FORWARD);
      myMotor3->run(FORWARD);
      myMotor1->setSpeed(mappedValue0);
      myMotor3->setSpeed(mappedValue0);
      myMotor2->run(BACKWARD);
      myMotor4->run(BACKWARD);
      myMotor2->setSpeed(0);
      myMotor4->setSpeed(0);
    } else if(inputIsRight) {   //Only the right side motors forwards
      myMotor2->run(FORWARD);
      myMotor4->run(FORWARD);
      myMotor2->setSpeed(mappedValue0);
      myMotor4->setSpeed(mappedValue0);
      myMotor1->run(BACKWARD);
      myMotor3->run(BACKWARD);
      myMotor1->setSpeed(0);
      myMotor3->setSpeed(0);
    } else {    //If X is not left or right, run motors on both sides forwards
      myMotor1->run(FORWARD);
      myMotor2->run(FORWARD);
      myMotor3->run(FORWARD);
      myMotor4->run(FORWARD);
      myMotor1->setSpeed(mappedValue0);
      myMotor2->setSpeed(mappedValue0);
      myMotor3->setSpeed(mappedValue0);
      myMotor4->setSpeed(mappedValue0);
    }
  } else if(sensorValue0 < deadZoneStart) {
    int mappedValue0 = map(sensorValue0, deadZoneEnd, 1023, 0, 255); //map 521-1023 to 0-255
    if(inputIsLeft) {   //Run only the right side motors backwards, to turn left
      myMotor2->run(BACKWARD);
      myMotor4->run(BACKWARD);
      myMotor2->setSpeed(mappedValue0);
      myMotor4->setSpeed(mappedValue0);
      myMotor1->run(BACKWARD);
      myMotor3->run(BACKWARD);
      myMotor1->setSpeed(0);
      myMotor3->setSpeed(0);
    } else if(inputIsRight) {   //Only the left side motors backwards, to turn right
      myMotor1->run(BACKWARD);
      myMotor3->run(BACKWARD);
      myMotor1->setSpeed(mappedValue0);
      myMotor3->setSpeed(mappedValue0);
      myMotor2->run(BACKWARD);
      myMotor4->run(BACKWARD);
      myMotor2->setSpeed(0);
      myMotor4->setSpeed(0);
    } else {    //If X is not left or right, run motors on both sides backwards
      myMotor1->run(BACKWARD);
      myMotor2->run(BACKWARD);
      myMotor3->run(BACKWARD);
      myMotor4->run(BACKWARD);
      myMotor1->setSpeed(mappedValue0);
      myMotor2->setSpeed(mappedValue0);
      myMotor3->setSpeed(mappedValue0);
      myMotor4->setSpeed(mappedValue0);
    }
  } else {  //When Y is in deadzone
    if(inputIsRight) {   //Run the left side motors forward and right side backwards, to turn right
      int mappedValue0 = normalSpeed;
      myMotor1->run(FORWARD);
      myMotor3->run(FORWARD);
      myMotor1->setSpeed(mappedValue0);
      myMotor3->setSpeed(mappedValue0);
      myMotor2->run(BACKWARD);
      myMotor4->run(BACKWARD);
      myMotor2->setSpeed(mappedValue0);
      myMotor4->setSpeed(mappedValue0);
    } else if(inputIsLeft) {   //The right side motors forwards and leftside backwards, to turn left
      int mappedValue0 = normalSpeed;
      myMotor1->run(BACKWARD);
      myMotor3->run(BACKWARD);
      myMotor1->setSpeed(mappedValue0);
      myMotor3->setSpeed(mappedValue0);
      myMotor2->run(FORWARD);
      myMotor4->run(FORWARD);
      myMotor2->setSpeed(mappedValue0);
      myMotor4->setSpeed(mappedValue0);
    } else {    //If X is not left or right, run motors on both sides
      myMotor1->run(BACKWARD);
      myMotor2->run(BACKWARD);
      myMotor3->run(BACKWARD);
      myMotor4->run(BACKWARD);
      myMotor1->setSpeed(0);
      myMotor2->setSpeed(0);
      myMotor3->setSpeed(0);
      myMotor4->setSpeed(0);
    }
  }

  
  Serial.print("InputIsLeft");
  Serial.print(inputIsLeft);
  Serial.print("\n");
  Serial.print("InputIsRight");
  Serial.print(inputIsRight);
  delay(50);
  
}
