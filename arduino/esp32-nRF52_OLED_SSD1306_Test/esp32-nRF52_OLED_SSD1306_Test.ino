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

 
// -------------------------------------------------------------------------------------------
// You need to set the board specification to handle power for the I2C Stemma connector!!!
// COMPILER DIRECTIVE to help debugging...
// Uncomment "#define ADAFRUIT_FEATHER_ESP32_V2" to activate: ADAFRUIT_FEATHER_ESP32_V2!
//#define ADAFRUIT_FEATHER_ESP32_V2

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

const unsigned char bluetooth_icon16x16[] PROGMEM = {
  0b00000000, 0b00000000, //                 
  0b00000001, 0b10000000, //        ##       
  0b00000001, 0b11000000, //        ###      
  0b00000001, 0b01100000, //        # ##     
  0b00001001, 0b00110000, //     #  #  ##    
  0b00001101, 0b00110000, //     ## #  ##    
  0b00000111, 0b01100000, //      ### ##     
  0b00000011, 0b11000000, //       ####      
  0b00000001, 0b10000000, //        ##       
  0b00000011, 0b11000000, //       ####      
  0b00000111, 0b01100000, //      ### ##     
  0b00001101, 0b00110000, //     ## #  ##    
  0b00001001, 0b00110000, //     #  #  ##    
  0b00000001, 0b01100000, //        # ##     
  0b00000001, 0b11000000, //        ###      
  0b00000001, 0b10000000, //        ##       
};

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//Global variables
double gradePercentValue = -20.3 ;
long PowerValue = 320;
uint8_t InstantaneousCadence = 114;
float SpeedValue = 24.1;
bool ConnectedToMobilePhone = true;

void setup() {
/* The Feather ESP32 V2 has a NEOPIXEL_I2C_POWER pin that must be pulled HIGH
 * to enable power to the STEMMA QT port. Without it, the QT port will not work!
 */
#ifdef ADAFRUIT_FEATHER_ESP32_V2
  // Turn on the I2C power on Stemma connector by pulling pin HIGH.
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
#endif
 
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for feather nrf52832 with native usb
   
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
      Serial.println("SSD1306 allocation failed");
    } else  {
      // Show initial display buffer contents with the screen ADAFRUIT Splash screen
      // the library initializes this (edit the splash.h in the library).
      display.display();
      delay(400);                    // Pause for some time
    }

  ShowOnOledLarge("", "Oled test!", "", 2000);

  ShowValuesOnOled();
  delay(4000);
}

void ShowOnOledLarge(char const *Line1, char const *Line2, char const *Line3, uint16_t Pause) {
  // Clear and set Oled to display 3 line info -> centered
  int pos = 1;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  if (ConnectedToMobilePhone) { // show BLE icon
    display.drawBitmap(116, 0, bluetooth_icon16x16, 16, 16, 1);
  } // shift icon to the right as much as possible
  display.setTextSize(2);  // Large characters 11 pixels wide
  if(Line1) {
    pos = round( (127-(12 * strlen(Line1)))/2 );
    display.setCursor(pos,2); // 16
    display.print(Line1);  
  }
  if(Line2){
    pos = round( (127-(12 * strlen(Line2)))/2 );
    display.setCursor(pos,22); // 16
    display.print(Line2);  
  }
  if(Line3){
    pos = round( (127-(12 * strlen(Line3)))/2 );
    display.setCursor(pos,44); // 16
    display.print(Line3);  
  }
  display.display();
  delay(Pause);  // Pause indicated time in ms
} // --------------------------------------

// Create basic Oled screen for measurement data
void BuildBasicOledScreen() {
  display.clearDisplay(); // clean the oled screen
  display.setTextColor(SSD1306_WHITE);
  if (ConnectedToMobilePhone) { // show BLE icon
    display.drawBitmap(116, 0, bluetooth_icon16x16, 16, 16, 1);
  } // shift icon to the right as much as possible
  display.setTextSize(1);
  display.setCursor(14, 32);
  display.print(F("Watt"));
  display.setCursor(62, 32);
  display.print(F("Rpm"));
  display.setCursor(99, 32);
  display.print(F("Kph"));
  display.setCursor(102, 10);
  display.setTextSize(2); 
  display.print(F("%"));
} // -------------------------

// Funtion to show measurement data: Grade, Power, Cadence and Speed on Oled screen
void ShowValuesOnOled(void) {
   BuildBasicOledScreen();
   display.setTextColor(SSD1306_WHITE);
   char tmp[12];   // Value was 10!!! --> fatal error @ sprintf() !!!!
   dtostrf(gradePercentValue, 5, 1, tmp); // show sign only if negative
   display.setCursor(10, 6);  
   display.setTextSize(3);
   display.print(tmp);
   sprintf(tmp, "%03d %03d %02d", PowerValue, InstantaneousCadence, int(SpeedValue+0.5)); 
   display.setCursor(4, 44);
   display.setTextSize(2);
   display.print(tmp);       
   display.display(); 
} // ------------------------
 
void ShowSlopeTriangleOnOled(void) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  if (ConnectedToMobilePhone) { // show BLE icon
    display.drawBitmap(116, 0, bluetooth_icon16x16, 16, 16, 1);
  } // shift icon to the right as much as possible
  display.setCursor(102, 10); //26
  display.setTextSize(2); 
  display.print(F("%"));
  char tmp[7];
  dtostrf(gradePercentValue, 5, 1, tmp); // show sign only if negative
  display.setCursor(10, 6); // 22 
  display.setTextSize(3);
  display.print(tmp);
  // The following calculations give more "weight" to lower grade values 
  // (like: 1.2% or 0.5%), these will occur more often in practice and are not well 
  // displayable at 128*64! --> 64 * 64 = 4096 and this value should not be 
  // exceeded (4096/20) = 204.8 
  //double temp = abs(double(204*gradePercentValue)); // max 4080 at 20% grade, NO SIGN!
  int pos = 64 - int(sqrt(abs(204*gradePercentValue))); // cast to int to get rid of decimals only now!
  if (gradePercentValue > 0) {
  display.fillTriangle( 1, 67, 127, 63, 127, pos, SSD1306_INVERSE);
  } else {
  display.fillTriangle( 127, 63, 1, 63, 1, pos, SSD1306_INVERSE);
  }
  // Draw the baseline to smooth small decimal values and show flat road case
  display.drawFastHLine(1, 63, 127, SSD1306_WHITE);
  display.display();
} // ---------------------------------------------------------------

void loop() {
  // some test values...
  gradePercentValue = -0.1;
  ShowSlopeTriangleOnOled();
  delay(1000);
  gradePercentValue = -0.3;
  ShowSlopeTriangleOnOled(); 
  delay(1000); 
  gradePercentValue = -0.4;
  ShowSlopeTriangleOnOled();
  delay(1000);
  gradePercentValue = -0.6;
  ShowSlopeTriangleOnOled();
  delay(1000);
  gradePercentValue = -0.8;
  ShowSlopeTriangleOnOled();
  delay(1000);
  gradePercentValue = -1.0;
  ShowSlopeTriangleOnOled();
  delay(1000);
  gradePercentValue = -1.3;
  ShowSlopeTriangleOnOled();
  delay(1000);
  gradePercentValue = -3.3;
  ShowSlopeTriangleOnOled();
  delay(1000);
  gradePercentValue = -6.3;
  ShowSlopeTriangleOnOled();
  delay(1000);
  gradePercentValue = 0.2;
  ShowSlopeTriangleOnOled();
  delay(1000);
  gradePercentValue = 0.5;
  ShowSlopeTriangleOnOled();
  delay(1000);
  gradePercentValue = 0.8;
  ShowSlopeTriangleOnOled();
  delay(1000);
  gradePercentValue = 1.0;
  ShowSlopeTriangleOnOled();
  delay(1000);
  gradePercentValue = 1.3;
  ShowSlopeTriangleOnOled();
  delay(1000);
  gradePercentValue = 2.3;
  ShowSlopeTriangleOnOled();
  delay(1000);
  gradePercentValue = 10.3;
  ShowSlopeTriangleOnOled();
  delay(1000);
  gradePercentValue = 15.3;
  ShowSlopeTriangleOnOled();
  delay(1000);
  gradePercentValue = 19.0;
  ShowSlopeTriangleOnOled();
  delay(1000);
}
