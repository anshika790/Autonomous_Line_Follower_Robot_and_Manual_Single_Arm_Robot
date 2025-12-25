#include <BluetoothSerial.h>
#include <ESP32Servo.h>

BluetoothSerial SerialBT;

const int numServos = 5;
Servo servos[numServos];Gripper
const int servoPins[numServos] = {13, 12, 14, 27, 26}; 

int servoAngles[numServos] = {90, 90, 90, 120, 60}; 
int dataIn = 0;
int speedDelay = 15;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("AG_Arm");
  Serial.println("Bluetooth ready. Waiting for commands...");

  
  for (int i = 0; i < numServos; i++) {
    servos[i].attach(servoPins[i], 500, 2400);
    servos[i].write(servoAngles[i]);
  }
}

void loop() {
  if (SerialBT.available() > 0) {
    dataIn = SerialBT.read();
    Serial.println(dataIn); 
    Serial.print("Received: ");
    Serial.println(dataIn);

    handleCommand(dataIn);
  }
}

void handleCommand(int cmd) {
  switch (cmd) {
    case 0: 
      break;

    case 1: adjustServo(0, +5); break; 
    case 2: adjustServo(0, -5); break; 
    case 3: adjustServo(1, +5); break; 
    case 4: adjustServo(1, -5); break; 
    case 5: adjustServo(2, +5); break; 
    case 6: adjustServo(2, -5); break; 
    case 7: adjustServo(3, +5); break; 
    case 8: adjustServo(3, -5); break; 
    case 9: adjustServo(4, +5); break; 
    case 10: adjustServo(4, -5); break; 
    
    default:
      Serial.println("Unknown command");
      break;
  }
}

// Increment or decrement servo angle
void adjustServo(int index, int change) {
  servoAngles[index] += change;
  servoAngles[index] = constrain(servoAngles[index], 0, 180);
  servos[index].write(servoAngles[index]);
  Serial.print("Servo ");
  Serial.print(index + 1);
  Serial.print(" angle: ");
  Serial.println(servoAngles[index]);
  delay(speedDelay); 
}