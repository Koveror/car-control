// BT-car, remote controller

// Bluetooth module serial pins
#define BT_RX 4
#define BT_TX 5
// Bluetooth carrier detect pin
#define BT_CD 6
#define BT_CD_LED A5

#define ESTE_LED A2

// Use softwareSerial for the bt communication
#include <SoftwareSerial.h>
SoftwareSerial btSerial(BT_RX, BT_TX); // RX, TX

struct s_control_msg {
  uint8_t start_magic;   // '<'
  uint8_t state[2];      // two byte hex encoded state
  uint8_t left_motor[2]; // two byte hex left motor speed
  uint8_t right_motor[2];// two byte hex right motor speed
  uint8_t end_magic;     // '>'
};

// Message and decoding helper
s_control_msg msg;
uint8_t msg_field;

bool este;

void setup() {
  // Pins
  pinMode(BT_CD, INPUT);
  pinMode(BT_CD_LED, OUTPUT);   // Use the built-in led for BT-carrier detect light
  Serial.begin(115200);  // USB serial
  btSerial.begin(38400); // SoftwareSerial, needs to match module configuration
  // Higher speeds result in errors.
  
  pinMode(ESTE_LED, OUTPUT);
  digitalWrite(ESTE_LED, LOW);
}

void loop() {
  // BT carrier detect
  if (digitalRead(6) == HIGH) {
    delay(10);
    digitalWrite(BT_CD_LED, HIGH);
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
      motor1_dir = false;
      motor1_speed = mappedValue0;
    } else if(sensorValue0 < deadZoneStart) {
      uint8_t mappedValue0 = map(sensorValue0, 0, deadZoneStart, 255, 0); //map 0-501 to 0-255
      motor1_dir = true;
      motor1_speed = mappedValue0;
    } else {
      motor1_dir=false;
      motor1_speed=0;
    }

    if(sensorValue1 > deadZoneEnd) {
      int mappedValue1 = map(sensorValue1, deadZoneEnd, 1023, 0, 255);
      motor2_dir=false;
      motor2_speed=mappedValue1;
    } else if(sensorValue1 < deadZoneStart) {
      int mappedValue1 = map(sensorValue1, 0, deadZoneStart, 255, 0);
      motor2_dir=true;
      motor2_speed=mappedValue1;
    } else {
      motor2_dir=true;
      motor2_speed=0;
    }
    send_msg(motor1_dir,motor2_dir,motor1_speed,motor2_speed);
    if (btSerial.available()) {
      uint8_t in = btSerial.read();
      if (in == 200) {
        digitalWrite(ESTE_LED, HIGH);
      } else if (in == 201) {
        digitalWrite(ESTE_LED, LOW);
      }
    }
  } else {
    digitalWrite(BT_CD_LED, LOW);
  }
}

// Echo data from one serial to the other
void serialEcho() {
  if (btSerial.available()) {
    Serial.write(btSerial.read());
  }
  if (Serial.available()) {
    btSerial.write(Serial.read());
  }
}

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

String decToHex(byte decValue) {
  
  String hexString = String(decValue, HEX);
  while (hexString.length() < 2) hexString = "0" + hexString;
  
  return hexString;
}

void decode_msg() {
  uint8_t left_direction, right_direction, left_speed, right_speed, state;
  state = hexToDec(msg.state);
  if (state & B00000001) {
    left_direction = 1;
  } else {
    left_direction = 2;
  }
  if (state & B00000010) {
    right_direction = 1;
  } else {
    right_direction = 2;
  }
  
  
  
  left_speed = hexToDec(msg.left_motor);
  right_speed = hexToDec(msg.right_motor);
}


// directions are true for forward
void send_msg(bool left_direction, bool right_direction, uint8_t left_speed, uint8_t right_speed) {
  uint8_t state = 0;
  String str;
  
  s_control_msg send_msg;
  send_msg.start_magic = '<';
  send_msg.end_magic   = '>';
  
  if (left_direction) {
    state = state | B00000001;
  }
  if (right_direction) {
    state = state | B00000010;
  }
  str = decToHex(state);
  send_msg.state[0] = str.c_str()[0];
  send_msg.state[1] = str.c_str()[1];
  
  str = decToHex(left_speed);
  send_msg.left_motor[0] = str.c_str()[0];
  send_msg.left_motor[1] = str.c_str()[1];
  
  str = decToHex(right_speed);
  send_msg.right_motor[0] = str.c_str()[0];
  send_msg.right_motor[1] = str.c_str()[1];
  
  // Send the message
  for (uint8_t i = 0; i < 8; i++) {
    btSerial.write(((uint8_t*)&send_msg)[i]);
  }
  
}


void receive_msg() {
  // Read new data until we get a complete message or data is exhausted
  while (btSerial.available()) {
    uint8_t in = (uint8_t)btSerial.read();
    if (in == '<') {
      // Start of message
      msg_field=1;
      continue;
    } else if (msg_field == 7 && in == '>') {
      // Complete message
      msg_field = 0;
      decode_msg();
      continue;
    } else if (in == '>') {
      // Premature end
      msg_field = 0;
      continue;
    }
    // Validate input to be 0-9 A-F
    if ((in >= '0' && in <= '9') || (in >= 'a' && in <= 'f')) {
      switch(msg_field) {
        case 1: msg.state[0] = in;       msg_field++; break;
        case 2: msg.state[1] = in;       msg_field++; break;
        case 3: msg.left_motor[0] = in;  msg_field++; break;
        case 4: msg.left_motor[1] = in;  msg_field++; break;
        case 5: msg.right_motor[0] = in; msg_field++; break;
        case 6: msg.right_motor[1] = in; msg_field++; break;
      }
    } else {
      // Invalid data, give up
      msg_field = 0;
    }
  } 
}
