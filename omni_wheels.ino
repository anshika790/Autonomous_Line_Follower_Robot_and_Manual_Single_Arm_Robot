#include "BluetoothSerial.h"

// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

// --- MOTOR PIN DEFINITIONS ---
const int M1_IN1 = 13;
const int M1_IN2 = 12;
const int M1_EN  = 14; // Speed Control

const int M2_IN1 = 27;
const int M2_IN2 = 26;
const int M2_EN  = 25;

const int M3_IN1 = 33;
const int M3_IN2 = 32;
const int M3_EN  = 15;

const int M4_IN1 = 18;
const int M4_IN2 = 19;
const int M4_EN  = 21;

// PWM Properties
const int freq = 30000;
const int resolution = 8;

int wheelSpeed = 100;
int dataIn, m;

// Helper function to control motor direction and speed
void setMotor(int motorId, int speed, int direction) {
  int in1, in2, enPin;
  
  // Select the correct pins based on motor ID
  if (motorId == 1)      { in1 = M1_IN1; in2 = M1_IN2; enPin = M1_EN; }
  else if (motorId == 2) { in1 = M2_IN1; in2 = M2_IN2; enPin = M2_EN; }
  else if (motorId == 3) { in1 = M3_IN1; in2 = M3_IN2; enPin = M3_EN; }
  else if (motorId == 4) { in1 = M4_IN1; in2 = M4_IN2; enPin = M4_EN; }
  else return;

  // FIXED: Write speed directly to the PIN (New ESP32 v3.0 API)
  ledcWrite(enPin, speed);

  if (direction == 1) { // Forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (direction == -1) { // Backward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else { // Stop/Release
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void setup() {
  Serial.begin(115200);
  
  SerialBT.begin("ESP32_Omni_Robot"); 
  Serial.println("The device started, now you can pair it with bluetooth!");

  // Set up Motor Direction Pins
  pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);
  pinMode(M3_IN1, OUTPUT); pinMode(M3_IN2, OUTPUT);
  pinMode(M4_IN1, OUTPUT); pinMode(M4_IN2, OUTPUT);

  // FIXED: Configure PWM/Speed Control (New ESP32 v3.0 API)
  // This replaces both ledcSetup and ledcAttachPin
  ledcAttach(M1_EN, freq, resolution);
  ledcAttach(M2_EN, freq, resolution);
  ledcAttach(M3_EN, freq, resolution);
  ledcAttach(M4_EN, freq, resolution);
}

// --- MOVEMENT FUNCTIONS ---

void moveForward() {
  setMotor(1, wheelSpeed, 1);
  setMotor(2, wheelSpeed, 1);
  setMotor(3, wheelSpeed, 1);
  setMotor(4, wheelSpeed, 1);
}

void moveBackward() {
  setMotor(1, wheelSpeed, -1);
  setMotor(2, wheelSpeed, -1);
  setMotor(3, wheelSpeed, -1);
  setMotor(4, wheelSpeed, -1);
}

void moveSidewaysRight() {
  setMotor(1, wheelSpeed, -1);
  setMotor(2, wheelSpeed, 1);
  setMotor(3, wheelSpeed, -1);
  setMotor(4, wheelSpeed, 1);
}

void moveSidewaysLeft() {
  setMotor(1, wheelSpeed, 1);
  setMotor(2, wheelSpeed, -1);
  setMotor(3, wheelSpeed, 1);
  setMotor(4, wheelSpeed, -1);
}

void rotateLeft() {
  setMotor(1, wheelSpeed, 1);
  setMotor(2, wheelSpeed, 1);
  setMotor(3, wheelSpeed, -1);
  setMotor(4, wheelSpeed, -1);
}

void rotateRight() {
  setMotor(1, wheelSpeed, -1);
  setMotor(2, wheelSpeed, -1);
  setMotor(3, wheelSpeed, 1);
  setMotor(4, wheelSpeed, 1);
}

void moveRightForward() {
  setMotor(1, wheelSpeed, 0); 
  setMotor(2, wheelSpeed, 1); 
  setMotor(3, wheelSpeed, 0); 
  setMotor(4, wheelSpeed, 1); 
}

void moveRightBackward() {
  setMotor(1, wheelSpeed, -1); 
  setMotor(2, wheelSpeed, 0);  
  setMotor(3, wheelSpeed, -1); 
  setMotor(4, wheelSpeed, 0);  
}

void moveLeftForward() {
  setMotor(1, wheelSpeed, 1); 
  setMotor(2, wheelSpeed, 0); 
  setMotor(3, wheelSpeed, 1); 
  setMotor(4, wheelSpeed, 0); 
}

void moveLeftBackward() {
  setMotor(1, wheelSpeed, 0);  
  setMotor(2, wheelSpeed, -1); 
  setMotor(3, wheelSpeed, 0);  
  setMotor(4, wheelSpeed, -1); 
}

void stopMoving() {
  setMotor(1, 0, 0);
  setMotor(2, 0, 0);
  setMotor(3, 0, 0);
  setMotor(4, 0, 0);
}

void loop() {
  if (SerialBT.available() > 0) {
    dataIn = SerialBT.read();
    Serial.print("value of datain: ");
    Serial.println(dataIn);

    if (dataIn >= 0 && dataIn <= 14) {
      m = dataIn;
      Serial.print("value of m: ");
      Serial.println(m);
    }

    if (dataIn >= 16) {
      wheelSpeed = dataIn;      
      Serial.print("value of speed: ");
      Serial.println(wheelSpeed);
    }
  }

  if (m == 0) stopMoving();
  else if (m == 1) moveRightForward();
  else if (m == 2) moveForward();
  else if (m == 3) moveLeftForward();
  else if (m == 4) moveSidewaysRight();
  else if (m == 5) moveSidewaysLeft();
  else if (m == 6) moveRightBackward();
  else if (m == 7) moveBackward();
  else if (m == 8) moveLeftBackward();
  else if (m == 9) rotateLeft();
  else if (m == 10) rotateRight();
}