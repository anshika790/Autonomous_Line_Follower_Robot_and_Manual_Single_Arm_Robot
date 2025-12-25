
#include <Adafruit_GFX.h>

#include <Adafruit_ST7735.h>

#include <SPI.h>

#include "HX711.h"

#include "team_photo.h" 

#define TFT_CS   26

#define TFT_RST  33

#define TFT_DC   32

#define TFT_MOSI 23

#define TFT_SCLK 18

#define TFT_LED  27

#define PIN_IR_GATE_1  16 

#define PIN_IR_GATE_2  14 

#define PIN_IR_GATE_3  17 

#define PIN_PUMP       13 

#define PIN_LAM_LED    4  





#define PIN_LOADCELL_DT  21

#define PIN_LOADCELL_SCK 22

const unsigned long DEFAULT_PUMP_RUN_TIME_MS = 3000UL; 

unsigned long pumpRunTimeMs = DEFAULT_PUMP_RUN_TIME_MS; 


const float LOADCELL_CALIBRATION_FACTOR = 2280.0; 
const float TARGET_WEIGHT_MIN = 120.0; 

const float TARGET_WEIGHT_MAX = 130.0; 

const unsigned long DEBOUNCE_MS = 50;

const uint8_t HX711_SAMPLES = 10;

const unsigned long HX711_READY_TIMEOUT_MS = 1500UL;

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

HX711 scale;


const uint8_t NUM_GATES = 3;

const int gatePins[NUM_GATES] = {PIN_IR_GATE_1, PIN_IR_GATE_2, PIN_IR_GATE_3};

int gateStableState[NUM_GATES];

int gateLastRaw[NUM_GATES];

unsigned long gateLastChangeMs[NUM_GATES];

bool pumpActive = false;

unsigned long pumpStartMs = 0;

unsigned long pumpDurationMs = 0;



bool lamActive = false;

unsigned long lamStartMs = 0;

unsigned long lamDurationMs = 0;

bool gate1Done = false;

bool gate2Done = false;

bool gate3Done = false;

float lastMeasuredWeight = 0.0;

bool weightValid = false;

Enum SysState {S_IDLE, S_PUMPING, S_LAM, S_WEIGHING, S_RESULT};

SysState currentState = S_IDLE;


bool splashActive = false;

unsigned long splashStartMs = 0;

unsigned long splashDurationMs = 0;


bool resultActive = false;

unsigned long resultStartMs = 0;

unsigned long resultDurationMs = 0;

void initLCD();

void startSplash(unsigned long ms);

void handleSplash();

void lcdStatus(const char* line1, const char* line2 = nullptr);

void lcdResult(bool pass, float weight);



void initLoadCell();

float readWeightAverage(uint8_t samples, unsigned long timeoutMs = HX711_READY_TIMEOUT_MS);



void initSensors();

void handleDebounce();



bool gateFallingEdge(uint8_t gateIndex); // returns true when stable HIGH->LOW (active low)

void handleStages();



void startPump(unsigned long durationMs);

void stopPump();

void handlePump();



void startLam(unsigned long durationMs);

void stopLam();

void handleLam();



void resetSystem();

void scheduleResultTimeout(unsigned long ms);

void handleResultTimeout();



void processSerialCommands();

void setup() {

  Serial.begin(115200);

  Serial.println("System Initializing...");



  // Pins

  pinMode(TFT_LED, OUTPUT);

  pinMode(PIN_PUMP, OUTPUT);

  pinMode(PIN_LAM_LED, OUTPUT);



  // IR inputs: use INPUT_PULLUP because sensors pull LOW when active

  for (uint8_t i = 0; i < NUM_GATES; ++i) {

    pinMode(gatePins[i], INPUT_PULLUP);

    gateLastRaw[i] = digitalRead(gatePins[i]);

    gateStableState[i] = gateLastRaw[i];

    gateLastChangeMs[i] = millis();

  }



  // Initial actuator states: since pump & LAM are active-LOW, drive HIGH to keep them OFF

  digitalWrite(TFT_LED, HIGH);

  digitalWrite(PIN_PUMP, HIGH);     // OFF (active-LOW)

  digitalWrite(PIN_LAM_LED, HIGH);  // OFF (active-LOW)



  // Initialize subsystems

  initLCD();

  startSplash(1200); // non-blocking splash

  initLoadCell();



  lcdStatus("System Ready", "Waiting for Car...");

  Serial.println("Setup Complete.");

}



// ==================== MAIN LOOP ====================

void loop() {

  processSerialCommands(); // allow runtime commands



  handleSplash();         // non-blocking splash display

  handleDebounce();       // update gateStableState[] based on debounce

  handlePump();           // pump timing

  handleLam();            // lam timing

  handleStages();         // check gates and advance sequence

  handleResultTimeout();  // non-blocking result timeout -> reset

}



// ==================== SUBSYSTEM IMPLEMENTATIONS ====================



// LCD helpers

void initLCD() {

  tft.initR(INITR_BLACKTAB);

  tft.fillScreen(ST7735_BLACK);

  tft.setRotation(0);

  tft.setTextWrap(true);

}



void startSplash(unsigned long ms) {

  splashActive = true;

  splashStartMs = millis();

  splashDurationMs = ms;

  tft.fillScreen(ST7735_BLACK);

  // draw bitmap now

  tft.drawRGBBitmap(0, 0, lcd_test_pic_1, 128, 160);

  // we keep the bitmap visible until splashDurationMs elapses; no blocking here

}



void handleSplash() {

  if (!splashActive) return;

  if (millis() - splashStartMs >= splashDurationMs) {

    splashActive = false;

    // After splash, show Ready screen (but do not block)

    lcdStatus("Ready for", "Next Car");

  }

}



void lcdStatus(const char* line1, const char* line2) {

  tft.fillScreen(ST7735_BLACK);

  tft.setCursor(8, 10);

  tft.setTextSize(1);

  tft.setTextColor(ST7735_WHITE);

  tft.println(line1);

  if (line2) {

    tft.setCursor(8, 30);

    tft.setTextColor(ST7735_CYAN);

    tft.println(line2);

  }

}



void lcdResult(bool pass, float weight) {

  tft.fillScreen(ST7735_BLACK);

  tft.setTextSize(2);

  if (pass) {

    tft.setTextColor(ST7735_GREEN);

    tft.setCursor(10, 30);

    tft.println("PASS");

  } else {

    tft.setTextColor(ST7735_RED);

    tft.setCursor(10, 30);

    tft.println("FAIL");

  }

  tft.setTextSize(1);

  tft.setCursor(10, 70);

  tft.setTextColor(ST7735_WHITE);

  if (!isnan(weight)) {

    tft.print("Vol: "); tft.print(weight); tft.println(" g");

  } else {

    tft.println("Vol: N/A");

  }

}



// Load cell

void initLoadCell() {

  Serial.println("Initializing Load Cell...");

  scale.begin(PIN_LOADCELL_DT, PIN_LOADCELL_SCK);

  scale.set_scale(LOADCELL_CALIBRATION_FACTOR);

  scale.tare();

}



float readWeightAverage(uint8_t samples, unsigned long timeoutMs) {

  unsigned long start = millis();

  // Wait for HX711 ready with non-blocking timeout; also allow serial processing while waiting

  while (!scale.is_ready()) {

    if (millis() - start >= timeoutMs) {

      return NAN; // timeout

    }

    processSerialCommands(); // keep serial responsive while waiting

    // do NOT call delay() here

  }

  // hx711 get_units may internally block briefly while reading samples; library function used here

  float val = scale.get_units(samples);

  return val;

}



// Debounce handler: updates gateStableState[]

void handleDebounce() {

  for (uint8_t i = 0; i < NUM_GATES; ++i) {

    int raw = digitalRead(gatePins[i]);

    if (raw != gateLastRaw[i]) {

      // raw changed, restart debounce timer

      gateLastChangeMs[i] = millis();

      gateLastRaw[i] = raw;

    } else {

      // raw is stable; update stable state after debounce interval

      if ((millis() - gateLastChangeMs[i]) >= DEBOUNCE_MS) {

        gateStableState[i] = raw; // 0 or 1

      }

    }

  }

}



// Sensor: returns true only when stable edge HIGH -> LOW detected (active-low sensors)

bool gateFallingEdge(uint8_t gateIndex) {

  static int prevStable[NUM_GATES] = {HIGH, HIGH, HIGH}; // assume HIGH at start (no car)

  int stable = gateStableState[gateIndex];

  bool falling = (prevStable[gateIndex] == HIGH) && (stable == LOW);

  prevStable[gateIndex] = stable;

  return falling;

}



// Central stage handler (non-blocking)

void handleStages() {

  // Stage 1: Pump station (waiting for falling edge on gate1)

  if (!gate1Done) {

    if (gateFallingEdge(0)) {

      Serial.println("Gate 1 Triggered (Pump)");

      lcdStatus("PUMP ON:", "Filling...");

      startPump(pumpRunTimeMs);

      gate1Done = true; // ensure sequence advances

      currentState = S_PUMPING;

    }

  }



  // Stage 2: LAM LED station

  if (gate1Done && !gate2Done) {

    if (gateFallingEdge(1)) {

      Serial.println("Gate 2 Triggered (LAM)");

      lcdStatus("LAM LED ON", nullptr);

      startLam(3000); // 3 seconds

      gate2Done = true;

      currentState = S_LAM;

    }

  }



  // Stage 3: Weighing station

  if (gate2Done && !gate3Done) {

    if (gateFallingEdge(2)) {

      Serial.println("Gate 3 Triggered (Weighing)");

      lcdStatus("Weighing...", nullptr);

      currentState = S_WEIGHING;



      // Perform weight read (this does a readiness wait but yields to serial)

      float w = readWeightAverage(HX711_SAMPLES, HX711_READY_TIMEOUT_MS);

      if (isnan(w)) {

        Serial.println("HX711 read failed or timed out.");

        lcdStatus("HX711 Error", "Check wiring/calib");

        lastMeasuredWeight = NAN;

        weightValid = false;

      } else {

        Serial.print("Weight Detected: "); Serial.println(w);

        lastMeasuredWeight = w;

        weightValid = true;

        bool pass = (w >= TARGET_WEIGHT_MIN && w <= TARGET_WEIGHT_MAX);

        lcdResult(pass, w);

      }



      gate3Done = true;

      currentState = S_RESULT;



      // Start non-blocking result timer (10 seconds) and return to loop

      scheduleResultTimeout(10000UL);

    }

  }

}



// Pump control (non-blocking)

// Active-LOW: drive LOW to turn ON, HIGH to turn OFF

void startPump(unsigned long durationMs) {

  if (pumpActive) return; // already running

  digitalWrite(PIN_PUMP, LOW);   // ON (active-LOW)

  pumpActive = true;

  pumpStartMs = millis();

  pumpDurationMs = durationMs;

  Serial.print("Pump started (ACTIVE-LOW) for "); Serial.print(durationMs); Serial.println(" ms");

}



void stopPump() {

  digitalWrite(PIN_PUMP, HIGH);  // OFF

  pumpActive = false;

  pumpStartMs = 0;

  pumpDurationMs = 0;

  Serial.println("Pump stopped (OFF -> HIGH)");

}



void handlePump() {

  if (pumpActive) {

    if (millis() - pumpStartMs >= pumpDurationMs) {

      stopPump();

      // After pump done, update LCD

      lcdStatus("Fill Complete", nullptr);

    }

  }

}



// LAM control (non-blocking)

// Active-LOW: drive LOW to turn ON, HIGH to turn OFF

void startLam(unsigned long durationMs) {

  if (lamActive) return;

  digitalWrite(PIN_LAM_LED, LOW); // ON

  lamActive = true;

  lamStartMs = millis();

  lamDurationMs = durationMs;

  Serial.print("LAM ON (ACTIVE-LOW) for "); Serial.print(durationMs); Serial.println(" ms");

}



void stopLam() {

  digitalWrite(PIN_LAM_LED, HIGH); // OFF

  lamActive = false;

  lamStartMs = 0;

  lamDurationMs = 0;

  Serial.println("LAM OFF (HIGH)");

}



void handleLam() {

  if (lamActive) {

    if (millis() - lamStartMs >= lamDurationMs) {

      stopLam();

    }

  }

}



// Schedule and handle the non-blocking result timeout

void scheduleResultTimeout(unsigned long ms) {

  resultActive = true;

  resultStartMs = millis();

  resultDurationMs = ms;

  Serial.print("Result displayed for "); Serial.print(ms); Serial.println(" ms");

}



void handleResultTimeout() {

  if (!resultActive) return;

  if (millis() - resultStartMs >= resultDurationMs) {

    resultActive = false;

    resetSystem();

    currentState = S_IDLE;

  }

}



// Reset system for next car

void resetSystem() {

  gate1Done = gate2Done = gate3Done = false;

  // ensure actuators are off (active-LOW => HIGH is off)

  if (pumpActive) stopPump();

  if (lamActive) stopLam();



  lastMeasuredWeight = 0.0;

  weightValid = false;



  // re-draw splash and ready message non-blocking

  startSplash(800);

  lcdStatus("Ready for", "Next Car");

  Serial.println("System reset and ready.");

}



// Serial command handling (simple)

// Commands:

//  'p' -> print current weight reading

//  'c' + number -> run pump for given ms for calibration (e.g. "c1500" runs pump 1500ms)

//  'r' -> reset system

//  's' + number -> set default pump runtime

void processSerialCommands() {

  while (Serial.available()) {

    String s = Serial.readStringUntil('\n');

    s.trim();

    if (s.length() == 0) return;

    char cmd = s.charAt(0);

    if (cmd == 'p') {

      float w = readWeightAverage(HX711_SAMPLES, 1000);

      if (isnan(w)) Serial.println("Weight read failed");

      else {

        Serial.print("Weight: "); Serial.print(w); Serial.println(" g");

      }

    } else if (cmd == 'c') {

      // calibrate pump runtime quickly by running for given ms

      String num = s.substring(1);

      num.trim();

      unsigned long msVal = num.toInt();

      if (msVal == 0) {

        Serial.println("Usage: c<ms>  e.g. c1500 to run pump 1500ms");

      } else {

        Serial.print("Calibration run: pump for "); Serial.print(msVal); Serial.println(" ms");

        startPump(msVal);

      }

    } else if (cmd == 'r') {

      resetSystem();

    } else if (cmd == 's') {


      String num = s.substring(1);

      pumpRunTimeMs = num.toInt();

      if (pumpRunTimeMs == 0) pumpRunTimeMs = DEFAULT_PUMP_RUN_TIME_MS;

      Serial.print("Set default pump runtime: "); Serial.print(pumpRunTimeMs); Serial.println(" ms");

    } else {

      Serial.print("Unknown cmd: "); Serial.println(s);

    }

  }

}