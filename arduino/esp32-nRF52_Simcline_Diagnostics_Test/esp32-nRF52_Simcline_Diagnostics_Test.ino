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
* Requirements: Fully equiped and operational Simcline (sensor/display/driver/Actuator),
*               working with an ESP32 or nRF52 board and connected over USB with computer
* Diagnostics can be run with manual input of data for Up/Down movement of Actuator
* or with the continuous input of data from a predetermined array of 8 test values! 
*
*
* NOTICE that you have to set several Simcline operation values in accordance with your specific setup!
*
*/

// -------------------------------------------------------------------------------------------
// COMPILER DIRECTIVE to allow/suppress DEBUG messages that help debugging...
// Comment out "#define MANUAL_INPUT_DATA" to activate: input of predetermined test values!
#define MANUAL_INPUT_DATA

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

//----------- Global variable definitions for high level movement control -----------------------------------------------
// In theory the RawgradeValue varies between 0 (equals -200% grade) and 40000 (equals +200% grade)
// SIMCLINE is mechanically working between -10% and +20% --> 19000 and 22000

//------------------------------------------------- WARNING --------------------------------------------------------------
//------------ SET THESE TWO VALUES IN ACCORDANCE WITH THE MECHANICAL RANGE LIMITATIONS OF YOUR SIMCLINE !!! -------------
// Raw Grade Value Minimally (Mechanically: the lowest position of wheel axis)  19000 is equiv. of 10% downhill road grade
#define RGVMIN 19500  // -5%  // Always is RGVMIN < 20000 (flat road level)
// Raw Grade Value Maximally (Mechanically: the highest position of wheel axis) 22000 is equiv. of 20% uphill road grade
#define RGVMAX 22000  // 20%  // +20% // Always is RGVMAX > 20000 (flat road level)
//------------------------------------------------- WARNING --------------------------------------------------------------

// Correction for measuring plane difference and midth wheel axis position (1 cm offset is an MEASUREOFFSET of about 40)
#define MEASUREOFFSET 50  // about 1.25 cm
// Raw Grade Value Minimally (Mechanically: the lowest position of wheel axis)  19000 is equiv. of 10% downhill road grade
// These values are derived from the above RGVMIN and RGVMAX settings
#define RGVMIN_GRADE (20000 - RGVMIN) / 100  // Notice: positive value of the Minimal downhill grade!
#define RGVMAX_GRADE (RGVMAX - 20000) / 100  // Notice: positive value of the Maximal uphill grade!
// Besides what is mechanically possible there are also limits in what is physically pleasant
// Keep the following aRGVMin and aRGVMax values within the limits of the mechanically feasible values of above !!!
// DEFAULT Minimally Allowed Raw Grade Value that should not be exceeded: -5%! -> Descent grade Limit
int aRGVmin = 19500;
// DEFAULT Maximally Allowed Raw Grade Value that should not be exceeded: 15%! -> Ascent grade limit
int aRGVmax = 21500;
// Value for a flat road equals 0% grade or a RGV of 20000; result needs to be corrected for the measure offset
long RawgradeValue = (20000 - MEASUREOFFSET);
int GradeChangeFactor = 100;  // 100% means no effect, 50% means only halved up/down steps --> Road Grade Change Factor
// The Grade Percentage of a road is defined as a measure of the road's steepness as it rises and falls along its route
float gradePercentValue = 0;
//-------------------------------------------------------------------------------------------------

/* ------------------------------------------------------------------------------------------------
 * Warning I/O Pins can have identical board position but different I/O Pin declarations for 
 * connection with the pins of the Motor driver board
 * ADAFRUIT_FEATHER_ESP32_V2 is nearly pin compatible with ARDUINO_NRF52840_FEATHER
*/
#if defined(ARDUINO_NRF52840_FEATHER) || defined(ADAFRUIT_FEATHER_ESP32_V2)
  #define actuatorOutPin1 A0   // --> A0/P0.02 connected to pin M1A of the MDD3A Motor Driver board
  #define actuatorOutPin2 A1   // --> A1/P0.03 connected to pin M1B of the MDD3A Motor Driver board
#endif
#if defined(ARDUINO_NRF52832_FEATHER) 
  #define actuatorOutPin1 2    // --> A0/P0.02 connected to pin M1A of the MDD3A Motor Driver board
  #define actuatorOutPin2 3    // --> A1/P0.03 connected to pin M1B of the MDD3A Motor Driver board
#endif

// -------------------------- WARNING ------------------------------------------------------------
// The following VL6180X sensor values are a 100% construction specific and
// should be experimentally determined, when the Actuator AND the VL6180X sensor are mounted!
// ------>>>> Test manually first and also use sketches that go with the VL6180X sensor! <<<<------
// Microswitches should limit physically/mechanically the upper and lower position of the Actuator!
// The microswitches are mechanically controlled, and NOT by the software --> should be fail safe!
// Notice that unrestricted movement at the boundaries can damage the Actuator and/or construction!
// The following values are respected by the software and will (in normal cases!) never be exceeded!
#define MINPOSITION 270 // 265 // VL6180X highest value top microswitch activated to mechanically stop operation
#define MAXPOSITION 460 // 535 // VL6180X lowest value bottom microswitch activated to mechanically stop operation

// -------------------------- WARNING ------------------------------------------------------------
// Operational boundaries of the VL6180X sensor are used/set in class Lifter after calling its "init".
// A safe measuring range of maximal 30 cm of total movement is recommended for the VL6180X sensor setting!
//
// Bandwidth is used in the code to take measuring errors and a safe margin into account when reaching
// the above defined max or min positions of the construction! The software does painstakingly respect
// these and is independent of the appropriate working of the microswitches when reaching the boundaries!
// These microswitches are a SECOND line of defence against out of range and potentially damaging movement!
#define BANDWIDTH 4

// Library code for low level measuring (VL6180X) and controlling UP and down movement
#include <Lifter.h>
// Decalaration of Lifter Class for control of the low level up/down movement
Lifter lift;
// Global variables for Lifter position control --> RawGradeValue has been defined/set previously to flat road level!!
int16_t TargetPosition = map(RawgradeValue, RGVMIN, RGVMAX, MAXPOSITION, MINPOSITION);
bool IsBasicMotorFunctions = false;  // Mechanical motor functions
 
// For testing purpose feed with some data manually -----------------------------------------------
#ifdef MANUAL_INPUT_DATA
char input; // Type 'u' for up movement or 'd' for down movement or '0' (zero) for neutral position
#else
uint8_t TargetPositionCount = 0;
//int16_t TargetValuesArray[8] = { 20000, 19500, 19000, 20000, 21000, 22000, 21500, 21000 };
// Alternative with smaller values
int16_t TargetValuesArray[8] = { 20000, 21000, 21500, 20500, 20000, 19500, 20000, 20500 };
#endif
// For testing purpose only -----------------------------------------------------------------------

void setup() 
{

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

  Serial.println("nRF52/ESP32 SIMCLINE Functioning Diagnostics");
  Serial.println("--------------- Version 010 ----------------");

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
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);  // Large characters
  display.setCursor(16,2);
  display.print(" TESTING");
  display.setCursor(16,22);
  display.print(" Function");
  display.display();
  display.setCursor(16,44);
  // Initialize Lifter Class data, variables and set to work
  lift.Init(actuatorOutPin1, actuatorOutPin2, MINPOSITION, MAXPOSITION, BANDWIDTH);
  delay(100);  
  if ( lift.TestBasicMotorFunctions() == false ) {
    display.print(" ERROR!"); 
    Serial.println("Basic Motor Funtions are NOT working!!");
    IsBasicMotorFunctions = false; // Not working properly
  } else {
    display.print("   OK!");
    Serial.println("Basic Motor Funtions are working!!");
    IsBasicMotorFunctions = true;
  }
  display.display(); 
  delay(500); // Pause some time
#ifdef MANUAL_INPUT_DATA 
  Serial.println("Type 'u' for UP or 'd' for DOWN movement or '0' (zero) for neutral position, and close with <ENTER> to confirm!");
#endif
}

// Show testing values....
void UpdateOledDisplay() {
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

void loop() 
{ 
    if (!IsBasicMotorFunctions) return; // DO NOTHING WHEN MOTOR FUNCTIONS ARE FAILING !!!!!
    
    // handle LIFTER Movement
    // Map RawgradeValue ranging from 0 to 40.000 on the 
    // TargetPosition (between MIN and MAX) of the Lifter
    // Notice 22000 is equivalent to +20% incline and 19000 to -10% incline
// Get input and set testing target values -----------------------------------
#ifdef MANUAL_INPUT_DATA
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
        RawgradeValue = 20000;
        Serial.print("Level ");
        break;
      default :
        return;
      }
    } else return; 
#else
  if ( (++TargetPositionCount)>7) {TargetPositionCount = 0;}  
    RawgradeValue = TargetValuesArray[TargetPositionCount];   
#endif
// ---------------  Process the test data ------------------------------------------------- 
    gradePercentValue = float(RawgradeValue-20000) / 100; 
    // Give feedback and print values to account for
#ifdef MANUAL_INPUT_DATA    
    Serial.printf("Move ms: %8d", millis()); 
#else
    Serial.printf("Step: [%1d] ms: %8d", TargetPositionCount, millis()); 
#endif
    Serial.printf("  Raw grade: %05d Grade perc.: %5.1f %%", RawgradeValue, gradePercentValue); 
    Serial.printf(" Target pos: %03d", TargetPosition, DEC); Serial.println();
    // Show target values on Oled
    UpdateOledDisplay();    
    // Take into account the measuring offset and comply to settings of Min and Max values
    RawgradeValue = RawgradeValue - MEASUREOFFSET;
    // Comply to Maximally en Minimally Allowed Raw Grade Values -----------------------------
    if (RawgradeValue < aRGVmin) {
      RawgradeValue = aRGVmin;  // Do not allow lower values than aRGVmin !!
    }
    if (RawgradeValue > aRGVmax) {
      RawgradeValue = aRGVmax;  // Do not allow values to exceed aRGVmax !!
    }
    RawgradeValue = constrain(RawgradeValue, RGVMIN, RGVMAX);
    // Finally determine the Target Position complying to all boundary settings!
    TargetPosition = map(RawgradeValue, RGVMIN, RGVMAX, MAXPOSITION, MINPOSITION);
    lift.SetTargetPosition(TargetPosition);
    int OnOffsetAction = 0;
  // internal movement control loop  ----------------------------------------------------------
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
  // end internal movement control loop --------------------------------------------------------
#ifdef MANUAL_INPUT_DATA 
  // no delay
#else
  delay(3000);
#endif
} // end loop
