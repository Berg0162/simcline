/*********************************************************************
This is programming code for ESP32 Espressif Wroom boards or 
                                   Adafruit nRF52840 boards
 
      Tested with Adafruit Feather ESP32 V2 a.k.a. Huzzah
      Tested with Adafruit nRF52840 Feather Express
 
 The code uses heavily the supplied ESP32/nRF52 libraries for different components!          

 Many have invested time and resources providing open source code!
 
        MIT license, check LICENSE for more information
        All text must be included in any redistribution
*********************************************************************/

/*
* Requirements: Setup equiped with VL6180X Time of Flight Sensor and SSD1306 Oled Display,
*               working with an ESP32 or nRF52 board and connected over USB with computer
*
*/

// -------------------------------------------------------------------------------------------
// COMPILER DIRECTIVE to allow/suppress DEBUG messages that help debugging...
// Uncomment "#define DEBUG" to activate
//#define DEBUG

// You need to set the board specification to handle correct pin assigments and power for the Stemma connector!!!
#define ADAFRUIT_FEATHER_ESP32_V2
//#define ARDUINO_NRF52840_FEATHER

// Libraries for use of I2C devices (Oled and VL6180X distance sensor)
#include <SPI.h>
#include <Wire.h>
// Necessary libraries for use of Oled display(s)
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// Additional splash screen bitmap and icon(s) for Oled
#include "Adafruit_SSD1306_Icons.h"  // needs to be in the SAME (!) directory
// Declarations for an SSD1306 128x64 display connected to I2C (SDA, SCL pins)
#define SCREEN_WIDTH 128       // SSD1306-OLED display width, in pixels
#define SCREEN_HEIGHT 64       // SSD1306-OLED display height, in pixels
#define OLED_RESET -1          // No reset pin on this OLED display
#define OLED_I2C_ADDRESS 0x3C  // I2C Address of OLED display
// Declare the display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#include <VL6180X.h>
VL6180X sensor;
// Set scaling of the VL6180X to approriate value 1, 2 or 3
// Only scaling factor #3 will work in a Simcline setup of measuring distance equal or larger than 30 cm!
#define SCALING 3
// setup VL6180X Range Continuous or Range Single Shot, read the manual.... 
#define RANGE_CONTINUOUS 0 // 1 = Range Continuous   0 = Range Single Shot
uint16_t CurrentPosition;

#include <MovingAverageFilter.h>
// Declare the running average filter for VL6180X Range measurements
// Filter is only used in the Lifter Class 
// Sampling is at about 10 Hz --> 10 VL6180X-RANGE-readings per second
#define NUMBER_OF_RANGE_READINGS 10
// Instantiate MovingAverageFilter class
MovingAverageFilter movingAverageFilter_Range(NUMBER_OF_RANGE_READINGS);

int16_t GetVL6180X_Range_Reading(void);
void Fill_Moving_Average_Filter(void);
void InitVL6180X(void);

void setup(void) {

  Serial.begin(115200);
  while (!Serial)
    delay(10);

/* The Feather ESP32 V2 has a NEOPIXEL_I2C_POWER pin that must be pulled HIGH
 * to enable power to the STEMMA QT port. Without it, the QT port will not work!
 */
#ifdef ADAFRUIT_FEATHER_ESP32_V2
  // Turn on the I2C power on Stemma connector by pulling pin HIGH.
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
#endif

  Serial.println("nRF52/ESP32 VL6180X and SSd1306 Diagnostics Test");
  Serial.println("---------------- Version 010 -------------------");

  // Set the correct I2C Address and start the show for the Oled display
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) {
    Serial.println(F("SSD1306 OLED display allocation failed!"));
  } else {
    Serial.println(F("SSD1306 OLED display is running..."));
    // Load Oled with initial display buffer contents on the screen,
    // the SSD1306 library initializes with a Adafruit splash screen,
    // (respect or edit the splash.h in the library).
    display.display();  // Acknowledge Adafruit rights, license and efforts
    delay(500);         // show some time
  }
 // Ready to show our own SIMCLINE splash screen
  display.clearDisplay();  // clean the oled screen
  display.setTextColor(SSD1306_WHITE);
  display.drawBitmap(24, 0, Mountain_bw_79x64, 79, 64, 1);
  display.display();
  delay(1000);  // Take somewhat more time.....
// Clear and set Oled display for further use
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);  // Large characters
  display.setCursor(16,22);
  display.print("SIMCLINE");
  display.setCursor(16,44);
  display.print("  2.0");
  display.display(); 
  delay(1000);                    // Pause some time
  
  InitVL6180X();
  // fill the movingAverageFilter with actual values instead of default zero's....
  // that blur operation in the early stages (of testing..)
  Fill_Moving_Average_Filter();
  // Clear and set Oled display for further use
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);  // Large characters
  display.setCursor(16,22);
  display.print("SIMCLINE"); 
  display.display(); 
  delay(2000);  
}

void Fill_Moving_Average_Filter(void) {
  // fill the movingAverageFilter with current values
  // these blur operation when movement changes of direction
  for (int i = 0; i < NUMBER_OF_RANGE_READINGS; i++) {
    delay(100); // Respect sample rate of 10 Hz
    CurrentPosition = GetVL6180X_Range_Reading();
    } 
}

void InitVL6180X(void) {  
  Wire.begin();
// setup VL6180X settings and operating mode
  sensor.init();
  // setup wire communication and default settings for the VL6180X
  Serial.println("Wire I2C and ToF VL6180X Initialized");
  sensor.configureDefault();
  sensor.setScaling(SCALING);
// Single shot operating mode of VL6180X is simplest and default
// The following is extra code critical for using Continuous mode !!!
#if RANGE_CONTINUOUS
  // Reduce range max convergence time and the inter-measurement
  // -time to 30 ms and 50 ms, respectively, to allow 10 Hz
  // operation. Somewhat more power consumption but higher accuracy!
  sensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  sensor.writeReg(VL6180X::SYSRANGE__INTERMEASUREMENT_PERIOD, 50);
  // stop continuous mode if already active
  sensor.stopContinuous();
  // in case stopContinuous() triggered a single-shot
  // measurement, wait for it to complete
  delay(300);
  // start range continuous mode with a period of 100 ms
  sensor.startRangeContinuous(100);
  Serial.print("VL6180X Range Continuous Mode Selected"); Serial.println();
#else 
  Serial.print("VL6180X Range Single Shot Mode Selected"); Serial.println();
#endif
  sensor.setTimeout(500);
}

int16_t GetVL6180X_Range_Reading(void) {
#if RANGE_CONTINUOUS
    int16_t temp = sensor.readRangeContinuousMillimeters();
#else
    int16_t temp = sensor.readRangeSingleMillimeters();
#endif
    if (sensor.timeoutOccurred()) 
        {
        Serial.print(">> ERROR << --> VL6180X reports TIMEOUT --> VL6180X is reset --> continue"); Serial.println();
        // Handle timeout error state VL6180X
        InitVL6180X();           // Initialize ToF VL6180X again --> this resets timeout error state!
        return CurrentPosition; // Do NOT use latest (temp) reading, it is not valid due to the timeout!!
        }
  return movingAverageFilter_Range.process(temp);  
} 

void loop(void) {
  // Sampling rate is at about 10 Hz  
  delay(100);
  CurrentPosition = GetVL6180X_Range_Reading();  
  // Show on OLED
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);  // Large characters
  display.setCursor(54,4);
  display.print(sensor.getScaling());
  display.print("x");
  display.setCursor(45,30);
  display.print(CurrentPosition); 
  display.display();   
}
