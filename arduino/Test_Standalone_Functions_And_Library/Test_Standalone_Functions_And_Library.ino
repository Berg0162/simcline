/* 

Testing motor control with driver MDD3A and feedback position with VL6180x

*/
//#include <Arduino.h>
//#include <SPI.h>
//#include <Wire.h>
//#include <Adafruit_GFX.h>

#include <Adafruit_SSD1306.h>

#include "Lifter.h"

  #define SCREEN_WIDTH 128     // OLED display width, in pixels
  #define SCREEN_HEIGHT 64     // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
  #define OLED_RESET     -1         // No reset pin on this OLED display
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//Global variables for measurement data
  bool ConnectedToTACX = false;
// RawGradeValue (RGV) varies between 0 (-200% grade) and 40000 (+200% grade)
// SIMCLINE is between -10% and +20% --> 19000 and 22000
  #define RGVMIN 19000 // 18750 // 19000 is equiv. of -10% correct (250 = 2,5 cm) for measuring plane difference and wheel axis position
  #define RGVMAX 22000 // 21750 // 22000 is equiv. of +20% correct (250 = 2,5 cm) for measuring plane difference and wheel axis position
// set value for a flat road = 0% grade --> 20000
  long RawgradeValue = (RGVMIN + 1000); // 0% grade
  float gradePercentValue = 0;

// MDD3A pin declarations
  #define actuatorOutPin1 2    // To the MDD2A
  #define actuatorOutPin2 3    // To the MDD2A
// Boundaries defined by SCALING = 3 of the VL6180X used in class Lifter
// Safe Range of at least 30 cm
  #define BANDWIDTH 4
  #define MINPOSITION 265 
  #define MAXPOSITION 535 

// Decalaration of Lifter Class for the actual movement 
  Lifter lift;
// Global variables for LIFTER position control
  int16_t TargetPosition = map(RawgradeValue, RGVMIN, RGVMAX, MAXPOSITION, MINPOSITION); 
// set value default for a flat road at start (RawgradeValue should be set to about 20000 before)!

// For testing purpose feed with some data ------------------------------------------------
  uint8_t TargetPositionCount = 0;
  int16_t TargetValuesArray[8] = { (RGVMIN+1000), 18800, 21600, 19800, 19000, 22000, 19000, 19800 }; 
// For testing purpose       --------------------------------------------------------------


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
  delay(500);                    // Pause some time
  }
 

  // Initialize Lifter Class data, variables and set to work
  lift.Init(actuatorOutPin1, actuatorOutPin2, MINPOSITION, MAXPOSITION, BANDWIDTH);
  
// Clear and set Oled display for further use
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);  // Large characters
  display.setCursor(16,22);
  display.print("SIMCLINE");
  display.setCursor(16,44);
  display.print("  2.0");
  display.display(); 
  delay(500);                    // Pause some time

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);  // Large characters
  display.setCursor(16,2);
  display.print("TESTING");
  display.setCursor(16,22);
  display.print("Function");
  display.display();
  
  display.setCursor(16,44);
  
  Serial.println("---------- Start ----------");
  Serial.println("--------- Testing ---------");
  
  if ( lift.TestBasicMotorFunctions() == false )
  {
  display.print("ERROR"); 
  }
  else display.print("  OK!");
  display.display(); 
  delay(500);                    // Pause some time

  Serial.printf("ms: %8d", millis()); Serial.printf("  Raw: %05d ", RawgradeValue, DEC); 
  Serial.printf(" Tpos: %03d", TargetPosition, DEC); Serial.println();
}

// Show testing values....
void UpdateOledDisplay()
{
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);  // Large characters
  display.setCursor(16,1);
  display.print(RawgradeValue); 
  display.setCursor(16,20);
  display.print(TargetPosition);
  display.setCursor(16,40);
  display.print(millis());
  display.display(); 
}
// ------------------------------------------------

void loop() 
{ 
    // handle LIFTER Movement
    // Map RawgradeValue ranging from 0 to 40.000 on the 
    // TargetPosition (between 265 and 532) of the Lifter
    // Notice 22000 is equivalent to +20% incline and 19000 to -10% incline
    RawgradeValue = constrain(RawgradeValue, RGVMIN, RGVMAX);
    TargetPosition = map(RawgradeValue, RGVMIN, RGVMAX, MAXPOSITION, MINPOSITION);
    lift.SetTargetPosition(TargetPosition);
    //  
    int OnOffsetAction = lift.GetOffsetPosition();
    switch (OnOffsetAction) 
      {
      case 0 :
        {
      lift.brakeActuator();

// For testing only ------------------------------------------------
    if ( (++TargetPositionCount)>7) {TargetPositionCount = 0;}  
    RawgradeValue = TargetValuesArray[TargetPositionCount];
    lift.SetTargetPosition(TargetPosition);
// ----------------------------------------------------------------
      Serial.printf("ms: %8d", millis()); Serial.printf("  Raw: %05d ", RawgradeValue, DEC); 
      Serial.printf(" Tpos: %03d", TargetPosition, DEC); Serial.println();
 
      break;
        }
      case 1 :
      lift.moveActuatorUp();
      break;
      case 2 :
      lift.moveActuatorDown();
      break;
      case 3 :
      // Timeout --> OffsetPosition is undetermined --> do nothing
      break;
      }
// for testing feedback

UpdateOledDisplay();
      
} // end loop
