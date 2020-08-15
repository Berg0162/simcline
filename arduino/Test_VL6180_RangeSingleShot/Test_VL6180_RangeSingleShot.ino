/* This minimal example shows how to get single-shot range
measurements from the VL6180X.

The range readings are in units of mm. */

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 64     // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1         // No reset pin on this OLED display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#include <VL6180X.h>
VL6180X sensor;
// To try different scaling factors, change the following define.
// Valid scaling factors are 1, 2, or 3.
#define SCALING 3

void setup() 
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for feather nrf52832 with native usb

// Init OLED Display
   if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) 
   { // Address 0x3C for 128x64
   Serial.println("SSD1306 allocation failed");
   }
  else
  {
  // Show Oled with initial display buffer contents on the screen --
  // the library initializes this with a Adafruit splash screen (edit the splash.h in the library).
  display.display();
  delay(2000);                    // Pause some time
  }

  // setup wire communication and defaults for the VL6180X
  Wire.begin();
  sensor.init();
  sensor.configureDefault();
  // Set scaling (after configureDefault = 1) of the VL6180X to approriate value 1, 2 or 3
  sensor.setScaling(SCALING);
  sensor.setTimeout(500);

  // Clear and set Oled display for further use
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);  // Large characters
  display.setCursor(16,22);
  display.print("SIMCLINE"); 
  display.display(); 
  delay(1000);
}

void loop() 
{ 
  Serial.print("(Scaling = ");
  Serial.print(sensor.getScaling());
  Serial.print("x) ");

  Serial.print(sensor.readRangeSingleMillimeters());
  
  // Show on OLED
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);  // Large characters
  display.setCursor(36,2);
  display.print(sensor.getScaling());
  display.print("x");
  display.setCursor(32,22);
  display.print(sensor.readRangeSingleMillimeters()); 
  display.display(); 

  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  
  Serial.println();
}
