// BT-car, remote controller
// Adafruit motor shield v2.0 libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
// Use softwareSerial for the bt communication
#include <SoftwareSerial.h>

/* Constants */

// Bluetooth module serial pins
#define BT_RX 4
#define BT_TX 5
// Bluetooth carrier detect pin
#define BT_CD 6
#define BT_CD_LED 13

/* Adafruit motor shield global objects */

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);

/* BT communication global objects */

SoftwareSerial btSerial(BT_RX, BT_TX); // Software-serial interface

// Message structure for bluetooth communication
struct s_control_msg {
  uint8_t start_magic;   // '<'
  uint8_t state[2];      // two byte hex encoded state
  uint8_t left_motor[2]; // two byte hex left motor speed
  uint8_t right_motor[2];// two byte hex right motor speed
  uint8_t end_magic;     // '>'
};

// Message and decoding helper
s_control_msg msg;  // Incoming message
uint8_t msg_field;  // Next message field to receive

// Run once when controller boots up
void setup() {
  /* Pins and busses */
  pinMode(BT_CD, INPUT);        // Pin goes HIGH when BT link is open
  pinMode(BT_CD_LED, OUTPUT);   // Use the built-in led for BT-carrier detect light
  Serial.begin(115200);         // USB serial for debuging
  btSerial.begin(38400);        // SoftwareSerial, needs to match module configuration
  // Higher speeds than 38400 result in errors.
  
  /* Adafruit motor shield initialization */
  
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

// Loop, reran constantly while the microcontroller is on
void loop() {
  // Check for the bluetooth link
  if (digitalRead(BT_CD) == HIGH) {
    digitalWrite(13,HIGH);
  } else {
    digitalWrite(13, LOW);
  }
  
  // Check for incoming messages
  receive_msg();
}

// Decode a received message
void decode_msg() {
  uint8_t left_direction, right_direction, left_speed, right_speed, state;
  
  /* First the 8bit state variable */
  state = hexToDec(msg.state);
  // First bit controls the left motor direction
  if (state & B00000001) { // bitwise and (&): 11 & 01 = 01
    left_direction = FORWARD;
  } else {
    left_direction = BACKWARD;
  }
  
  // Second bit controls the rigth motor direction
  if (state & B00000010) { // bitwise and (&): 11 & 01 = 01
    right_direction = FORWARD;
  } else {
    right_direction = BACKWARD;
  }
  
  // Speeds are just hex-encoded 8bit integers
  left_speed = hexToDec(msg.left_motor);
  right_speed = hexToDec(msg.right_motor);
  
  // Set direction for left motors
  myMotor1->run(left_direction);
  myMotor3->run(left_direction);
  // And for right motors
  myMotor2->run(right_direction);
  myMotor4->run(right_direction);

  // Left motor speeds
  myMotor1->setSpeed(left_speed);
  myMotor3->setSpeed(left_speed);
  // Right motor speeds
  myMotor2->setSpeed(right_speed);
  myMotor4->setSpeed(right_speed);

  
  // Debug code; output the decoded values to serial
  Serial.println("Decoded:");
  Serial.print(left_direction);
  Serial.println(right_direction);
  Serial.println(left_speed);
  Serial.println(right_speed);
}


// Send a message over the BT link
// directions are true for forward
void send_msg(bool left_direction, bool right_direction, uint8_t left_speed, uint8_t right_speed) {
  uint8_t state = 0;
  String str;
  s_control_msg send_msg;
  
  // Place start and end markers
  send_msg.start_magic = '<';
  send_msg.end_magic   = '>';
  
  // Encode the motor directions
  // bitwise or (|): 10 | 01 = 11
  if (left_direction) {
    state = state | B00000001;
  }
  if (right_direction) {
    state = state | B00000010;
  }
  
  // Encode the state as a 2-digit hex number
  str = decToHex(state);
  // Place the hex digits on the message struct
  send_msg.state[0] = str.c_str()[0];
  send_msg.state[1] = str.c_str()[1];
  
  // Speeds: just encode the given values as hex and place into the message
  str = decToHex(left_speed);
  send_msg.left_motor[0] = str.c_str()[0];
  send_msg.left_motor[1] = str.c_str()[1];
  
  str = decToHex(right_speed);
  send_msg.right_motor[0] = str.c_str()[0];
  send_msg.right_motor[1] = str.c_str()[1];
  
  // Send the message. This uses some pointer-magic for convenience...
  // &send_msg == pointer to send_msg
  // then we cast it as a uint8_t pointer, so it becomes an array of uint8_t
  // after that it is easy to just loop over all the 8bit bytes in the message and send them
  for (uint8_t i = 0; i < 8; i++) {
    btSerial.write(((uint8_t*)&send_msg)[i]);
  }
}

// Check if any new serial data has been received, if so process it into a message
void receive_msg() {
  // Read new data until we get a complete message or data is exhausted
  while (btSerial.available()) {
    
    // Read one byte from the serial buffer
    uint8_t in = (uint8_t)btSerial.read();
    
    // Check if we got the message start or end marker
    if (in == '<') {
      // Start of message
      msg_field=1;    // Start receiving actual data
      continue;       // Return to the while loop
    } else if (msg_field == 7 && in == '>') {
      // Complete message received.
      msg_field = 0;  // Reset the message receiving state to start
      decode_msg();   // Decode the received message
      continue;       // Return to the while loop
    } else if (in == '>') {
      // Premature end, not enough message fields received
      msg_field = 0;  // Reset the message reiving state
      continue;       // Return to the while loop
    }
    // Validate input to be 0-9 a-f
    if ((in >= '0' && in <= '9') || (in >= 'a' && in <= 'f')) {
      // Valid hex digit received. Now place it into the correct field.
      switch(msg_field) {
        case 1: msg.state[0] = in;       break;
        case 2: msg.state[1] = in;       break;
        case 3: msg.left_motor[0] = in;  break;
        case 4: msg.left_motor[1] = in;  break;
        case 5: msg.right_motor[0] = in; break;
        case 6: msg.right_motor[1] = in; break;
      }
      msg_field++;  // Increment the message receiving state
    } else {
      // Invalid data, give up
      msg_field = 0;
    }
  } 
}

/* Utility functions */

// Echo data from one serial to the other, for configuring the BT module and debugging
void serialEcho() {
  if (btSerial.available()) {
    Serial.write(btSerial.read());
  }
  if (Serial.available()) {
    btSerial.write(Serial.read());
  }
}


// Decode 2-digit hex number to integer
// eg. 0f -> 15
uint8_t hexToDec(uint8_t *hexString) {  
  uint8_t decValue = 0;
  uint8_t nextInt;
  
  for (uint8_t i = 0; i < 2; i++) {
    
    nextInt = hexString[i];
    if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
    if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
    if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
    nextInt = constrain(nextInt, 0, 15);
    
    decValue = (decValue * 16) + nextInt;
  }
  
  return decValue;
}

// Encode a integer into two digit hex number
String decToHex(byte decValue) {
  
  String hexString = String(decValue, HEX);
  while (hexString.length() < 2) hexString = "0" + hexString;
  
  return hexString;
}
