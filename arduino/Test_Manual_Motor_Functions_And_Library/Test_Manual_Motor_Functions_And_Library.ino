/* 

Testing motor control with driver MDD3A and feedback position with VL6180x

*/
#include <Adafruit_SSD1306.h>
#include "Lifter.h"

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 64     // OLED display height, in pixels
#define OLED_RESET    -1     // No reset pin on this OLED display
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Global variables for measurement data
// RawGradeValue (RGV) varies between 0 (-200% grade) and 40000 (+200% grade)
// SIMCLINE is between -10% and +20% --> 19000 and 22000
#define RGVMIN 19000 // -10%    // 19000 // 19000 is equiv. of -10% correct (250 = 2,5 cm) for measuring plane difference and wheel axis position
#define RGVMAX 22000 // +20%    // 22000 // 22000 is equiv. of +20% correct (250 = 2,5 cm) for measuring plane difference and wheel axis position
// set default value for a flat road = 0% grade --> 20000
#define MEASUREOFFSET 120 // Distance between reflection plate and central position of the axle
  long RawgradeValue = 20000 - MEASUREOFFSET; // Flatroad 0% inclination

/* ------------------------------------------------------------------------------------------------
 * Warning I/O Pins have identical position but different naming depending on the processor board
 * I/O Pin declarations for connection to Motor driver board MDD3A
*/
#if defined(ARDUINO_NRF52840_FEATHER) 
  #define actuatorOutPin1 A0   // --> A0/P0.02 connected to pin M1A of the MDD3A Motor Driver board
  #define actuatorOutPin2 A1   // --> A1/P0.03 connected to pin M1B of the MDD3A Motor Driver board
#endif
#if defined(ARDUINO_NRF52832_FEATHER) 
  #define actuatorOutPin1 2    // --> A0/P0.02 connected to pin M1A of the MDD3A Motor Driver board
  #define actuatorOutPin2 3    // --> A1/P0.03 connected to pin M1B of the MDD3A Motor Driver board
#endif

// Boundaries defined by SCALING = 3 of the VL6180X used in class Lifter
// Safe Range of at least 30 cm
#define BANDWIDTH 4
#define MINPOSITION 265 //  Simcline safe test value // 255 // 265
#define MAXPOSITION 530 //  Simcline safe test value // 530 // 525

// Decalaration of Lifter Class for the actual movement 
  Lifter lift;
// Global variables for LIFTER position control default flat road 20000 and 0% inclination
// RawgradeValue should be set to 20000 before!
  int16_t TargetPosition = map(RawgradeValue, RGVMIN, RGVMAX, MAXPOSITION, MINPOSITION);
  float gradePercentValue = 0.0; 
  bool IsBasicMotorFunctions = false; // Mechanical motor functions

// For testing purpose feed with some data -------------------------------------------------------
  char input; // Type 'u' for up movement or 'd' for down movement or '0' (zero) for neutral position
// For testing purpose       ----------------------------------------------------------------------


void setup() 
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for feather nrf52 with native usb

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
  
  Serial.println("----------------- Start ------------------");
  
  if ( lift.TestBasicMotorFunctions() == false ) {
    display.print("ERROR"); 
    Serial.println("Basic Motor Funtions are NOT working!!");
    IsBasicMotorFunctions = false; // Not working properly
  }
  else {
    display.print("  OK!");
    Serial.println("Basic Motor Funtions are working!!");
    IsBasicMotorFunctions = true;
  }

  display.display(); 
  delay(500);                    // Pause some time
  Serial.println("Type 'u' for up or 'd' for down movement or '0' (zero) for neutral position, and close with <ENTER> to confirm!");
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
    if (!IsBasicMotorFunctions) return; // DO NOTHING WHEN MOTOR FUNCTIONS ARE FAILING !!!!!
    
    // handle LIFTER Movement
    // Map RawgradeValue ranging from 0 to 40.000 on the 
    // TargetPosition (between MIN and MAX) of the Lifter
    // Notice 22000 is equivalent to +20% incline and 19000 to -10% incline
// Get input and set testing target values -----------------------------------
    if(Serial.available()){
      input = Serial.read();
      switch (input)
      {
      case 117 : // u
        RawgradeValue += 100;
        Serial.print("Up    ");
        break;
      case 100 : // d
        RawgradeValue -= 100;
        Serial.print("Down  ");
        break;
      case 48 : // 0
        RawgradeValue = 20000 - MEASUREOFFSET;
        Serial.print("Level ");
        break;
      default :
        return;
      }
    } else return;
// ---------------------------------------------------------------- 
    RawgradeValue = constrain(RawgradeValue, RGVMIN, RGVMAX);
    TargetPosition = map(RawgradeValue, RGVMIN, RGVMAX, MAXPOSITION, MINPOSITION);
    gradePercentValue = float( ( RawgradeValue - 20000 + MEASUREOFFSET ) ) / 100;
    lift.SetTargetPosition(TargetPosition);
    
    Serial.printf("Raw grade: %05d Grade perc.: %2.1f %%", RawgradeValue, gradePercentValue); 
    Serial.printf(" Target pos: %03d", TargetPosition, DEC); Serial.println();
    
    int OnOffsetAction = 0;
// internal movement control loop  
 do {
    OnOffsetAction = lift.GetOffsetPosition();
    switch (OnOffsetAction) 
      {
      case 0 :
        lift.brakeActuator();
        break;
      case 1 :
        lift.moveActuatorUp();
        break;
      case 2 :
        lift.moveActuatorDown();
        break;
      case 3 :
        // --> OffsetPosition is undetermined --> do nothing
        lift.brakeActuator();
        break;
      }
  } while ( (OnOffsetAction == 1) || (OnOffsetAction == 2) ); // Run the loop until target position is reached!
// end internal movement control loop

// for testing feedback
    UpdateOledDisplay();
} // end loop
