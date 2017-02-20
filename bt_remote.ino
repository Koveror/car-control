// BT-car, remote controller

/* Constants */

// Bluetooth module serial pins
#define BT_RX 4
#define BT_TX 5
// Bluetooth carrier detect pin
#define BT_CD 6
#define BT_CD_LED 13

// Use softwareSerial for the bt communication
#include <SoftwareSerial.h>

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

void setup() {
  // Pins
  pinMode(BT_CD, INPUT);
  pinMode(13, OUTPUT);   // Use the built-in led for BT-carrier detect light
  Serial.begin(115200);  // USB serial
  btSerial.begin(38400); // SoftwareSerial, needs to match module configuration
                          // Higher speeds than 38400 result in errors.
}

void loop() {
  // BT carrier detect
  if (digitalRead(BT_CD) == HIGH) {
    // We have a link
    delay(100);
    
    digitalWrite(BT_CD_LED, HIGH); // Turn on link-detect led

    /* Read and process the input from the joysticks */

    //Analog input from A0 and A1
    int sensorValue0 = analogRead(A0);  //value of A0 from 0 to 1023 (left side motors)
    int sensorValue1 = analogRead(A1);  //value of A1 from 0 to 1023 (right side motors)
    int deadZoneStart = 501;            //deadzone around halfway
    int deadZoneEnd = 521;
    bool motor1_dir, motor2_dir;
    uint8_t motor1_speed, motor2_speed;


    //Check on which side of the deadzone the joystick is and control the motors accordingly
    if(sensorValue0 > deadZoneEnd) {
      uint8_t mappedValue0 = map(sensorValue0, deadZoneEnd, 1023, 0, 255); //map 521-1023 to 0-255
      motor1_dir = true;
      motor1_speed = mappedValue0;
    } else if(sensorValue0 < deadZoneStart) {
      uint8_t mappedValue0 = map(sensorValue0, 0, deadZoneStart, 255, 0); //map 0-501 to 0-255
      motor1_dir = false;
      motor1_speed = mappedValue0;
    } else {
      motor1_dir=false;
      motor1_speed=0;
    }

    if(sensorValue1 > deadZoneEnd) {
      int mappedValue1 = map(sensorValue1, deadZoneEnd, 1023, 0, 255);
      motor2_dir=true;
      motor2_speed=mappedValue1;
    } else if(sensorValue1 < deadZoneStart) {
      int mappedValue1 = map(sensorValue1, 0, deadZoneStart, 255, 0);
      motor2_dir=false;
      motor2_speed=mappedValue1;
    } else {
      motor2_dir=true;
      motor2_speed=0;
    }
    
    // Send the control data to the car
    send_msg(motor1_dir,motor2_dir,motor1_speed,motor2_speed);


  } else {
    // No BT link
    digitalWrite(BT_CD_LED, LOW);
  }
}

// Decode a received message
void decode_msg() {
  uint8_t left_direction, right_direction, left_speed, right_speed, state;
  
  /* First the 8bit state variable */
  state = hexToDec(msg.state);
  // First bit controls the left motor direction
  if (state & B00000001) { // bitwise and (&): 11 & 01 = 01
    left_direction = 1;
  } else {
    left_direction = 2;
  }
  
  // Second bit controls the rigth motor direction
  if (state & B00000010) { // bitwise and (&): 11 & 01 = 01
    right_direction = 1;
  } else {
    right_direction = 2;
  }
  
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
