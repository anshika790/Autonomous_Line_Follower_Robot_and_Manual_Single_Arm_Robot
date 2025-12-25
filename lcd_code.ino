#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <SPI.h>

// 1. INCLUDE YOUR IMAGE TAB
#include "team_photo.h" 

// --- PIN DEFINITIONS (Updated for your ESP32 Board) ---
#define TFT_CS   5   // Pin G5
#define TFT_RST  4   // Pin G4
#define TFT_DC   2   // Pin G2 (A0)
#define TFT_MOSI 23  // Pin G23 (SDA)
#define TFT_SCLK 18  // Pin G18 (SCK)
#define TFT_LED  27  // Add this line for the backlight pin



// Initialize the screen
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

void setup() {
  Serial.begin(115200);
  pinMode(TFT_LED, OUTPUT);    // Tell ESP32 that G27 is an output
  digitalWrite(TFT_LED, HIGH); // Turn G27 ON (Lights up the screen)
  // --------------------------
  Serial.println("Starting LCD...");

  // 1. Initialize Display
  // Try INITR_BLACKTAB first. If colors are wrong, try INITR_GREENTAB.
  tft.initR(INITR_BLACKTAB); 
  
  // 2. Clear Screen
  tft.fillScreen(ST7735_BLACK);

  // 3. Set Orientation
  // 0 = Portrait (Cables top/bottom)
  // 1 = Landscape (Cables left/right)
  tft.setRotation(0); 

  // 4. Draw the Image
  // Ensure "lcd_test_pic_1" matches the variable name in your team_photo.h file!
  Serial.println("Drawing image...");
  tft.drawRGBBitmap(0, 0, lcd_test_pic_1, 128, 160);
  
  Serial.println("Done!");
}

void loop() {
  // Nothing needed here
}