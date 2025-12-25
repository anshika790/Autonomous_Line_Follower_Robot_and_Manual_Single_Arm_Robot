#define DIGITAL_SENSORS false
#define threshold_value 600

// --- ULTRASONIC SENSOR SETTINGS ---
#define TRIG_PIN 3   // Trigger pin for obstacle detection
#define ECHO_PIN 4   // Echo pin for obstacle detection
#define STOP_DISTANCE 20 // cm (stop if obstacle is closer than this)
// ----------------------------------

const uint8_t sensorPins[8] = {A0,A1,A2,A3,A4,A5,A6,A7};
const float sensorWeight[8] = {3.5, 2.5, 1.5, 0.5, -0.5, -1.5, -2.5, -3.5};

const uint8_t ENA = 10;
const uint8_t IN1 = 9;
const uint8_t IN2 = 8;

const uint8_t ENB = 5;
const uint8_t IN3 = 7;
const uint8_t IN4 = 6;

float Kp = 30.0;
float Ki = 0.0;
float Kd = 15.0;

int baseSpeed = 80;
int maxSpeed  = 150;
int minSpeed  = 60;

float integral = 0.0;
float prevError = 0.0;
unsigned long prevTime = 0;
float alpha = 0.9;
float lastDerivative = 0.0;

int thresholds[8];
int rawMax[8], rawMin[8];
bool calibrated = false;

struct SensorResult {
  float position;
  bool onLine;
  int raw[8];
};

// --- Ultrasonic ---
int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 5000);
  if (duration == 0) return 999; // No echo means no obstacle
  return duration * 0.034 / 2;   // Convert to cm
}

// --- Motor Control ---
void driveMotorLeft(int pwm) {
  if (pwm >= 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, constrain(pwm,0,255));
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, constrain(-pwm,0,255));
  }
}

void driveMotorRight(int pwm) {
  if (pwm >= 0) {
    digitalWrite(IN4, HIGH);
    digitalWrite(IN3, LOW);
    analogWrite(ENB, constrain(pwm,0,255));
  } else {
    digitalWrite(IN4, LOW);
    digitalWrite(IN3, HIGH);
    analogWrite(ENB, constrain(-pwm,0,255));
  }
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

// --- Sensor Reading ---
SensorResult readSensors() {
  SensorResult r;
  float weightedSum = 0.0;
  float total = 0.0;
  r.onLine = false;

  for (int i=0;i<8;i++) {
    int value = analogRead(sensorPins[i]);
    r.raw[i] = value;
    float activated = (!calibrated) ? (value > threshold_value) : (value > thresholds[i]);
    if (activated) {
      r.onLine = true;
      weightedSum += sensorWeight[i];
      total += 1.0;
    }
  }

  if (r.onLine && total > 0) {
    r.position = weightedSum / total;
  } else {
    r.position = (prevError >= 0) ? 3.5 : -3.5;
  }
  return r;
}

// --- Calibration ---
void calibrateSensors(int cycles = 200, int delayMs = 5) {
  for (int i=0;i<8;i++) {
    rawMax[i] = 0;
    rawMin[i] = 1023;
  }
  unsigned long start = millis();
  while (millis() - start < cycles * delayMs) {
    for (int i=0;i<8;i++) {
      int v = analogRead(sensorPins[i]);
      if (v > rawMax[i]) rawMax[i] = v;
      if (v < rawMin[i]) rawMin[i] = v;
    }
    delay(delayMs);
  }
  for (int i=0;i<8;i++) {
    thresholds[i] = (rawMax[i] + rawMin[i]) / 2;
  }
  calibrated = true;
}

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.println("Calibrating...");
  delay(1000);
  calibrateSensors(400, 5);
  Serial.println("Calibration done.");
  prevTime = millis();
}

void loop() {
  // --- Obstacle Detection ---
  int distance = getDistance();
  if (distance > 0 && distance < STOP_DISTANCE) {
    stopMotors();
    Serial.print("OBSTACLE DETECTED: "); Serial.print(distance); Serial.println(" cm");
    
    // Wait until obstacle is removed
    while (getDistance() < STOP_DISTANCE) {
      delay(50);
    }
    Serial.println("Obstacle cleared. Resuming...");
  }

  unsigned long now = millis();
  if (now - prevTime < 3) return;
  float dt = (now - prevTime) / 1000.0;
  if (dt <= 0) dt = 0.001;

  SensorResult s = readSensors();
  float error = s.position;

  // --- Directional Lost Line Recovery ---
  static unsigned long lostLineTime = millis();
  if (!s.onLine) {
    if (millis() - lostLineTime > 200) {
      stopMotors();
      Serial.println("Lost line! Searching...");
      
      // Turn toward last known direction
      if (prevError > 0) {
        driveMotorLeft(80);
        driveMotorRight(-80);
      } else {
        driveMotorLeft(-80);
        driveMotorRight(80);
      }

      // Keep turning until line is found
      while (!readSensors().onLine) {
        delay(10);
      }
      Serial.println("Line found!");
    }
  } else {
    lostLineTime = millis();
  }

  // --- Deadband for straight motion ---
  if (fabs(error) < 0.2) {
    driveMotorLeft(baseSpeed);
    driveMotorRight(baseSpeed);
    prevError = error;
    prevTime = now;
    return;
  }

  // PID Calculation
  integral += error * dt;
  integral = constrain(integral, -200.0, 200.0);
  float derivative = (error - prevError) / dt;
  lastDerivative = alpha * lastDerivative + (1.0 - alpha) * derivative;

  // Dynamic PID tuning
  float dynamicKp = (fabs(error) < 1.0) ? Kp * 0.5 : Kp;
  float dynamicKd = (fabs(error) < 1.0) ? Kd * 0.5 : Kd;

  float output = dynamicKp * error + Ki * integral + dynamicKd * lastDerivative;

  int currentBaseSpeed = baseSpeed;
  if (fabs(error) > 2.5) currentBaseSpeed = baseSpeed / 2;

  float leftSpeedF = currentBaseSpeed + output;
  float rightSpeedF = currentBaseSpeed - output;

  int leftPWM = constrain(abs(leftSpeedF), minSpeed, maxSpeed);
  int rightPWM = constrain(abs(rightSpeedF), minSpeed, maxSpeed);

  int leftSigned = (leftSpeedF >= 0) ? leftPWM : -leftPWM;
  int rightSigned = (rightSpeedF >= 0) ? rightPWM : -rightPWM;

  driveMotorLeft(leftSigned);
  driveMotorRight(rightSigned);

  prevError = error;
  prevTime = now;
}