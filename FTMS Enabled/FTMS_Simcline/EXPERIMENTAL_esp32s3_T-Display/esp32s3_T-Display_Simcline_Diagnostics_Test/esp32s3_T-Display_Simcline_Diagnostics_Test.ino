/*********************************************************************
This is programming code for LilyGo ESP32 T-Display S3 (170x320)
 
  The code uses heavily on the supplied ESP32, TFT_eSPI and sensor libraries!          

  Many have invested time and resources providing open source code!
 
        MIT license, check LICENSE for more information
        All text must be included in any redistribution
*********************************************************************/

/*
* Requirements: Fully equiped and operational Simcline (sensor/display/driver/Actuator),
*               working with an ESP32 T-Display-S3 connected over USB with computer
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

// You need to set the board specification to handle correct pin assigments and power !!!
#define LILYGO_T_DISPLAY_ESP32_S3

// Libraries for use of devices (TFT display and VL6180X distance sensor)
#include <SPI.h>
#include <Wire.h>

// Necessary libraries for use of TFT display
#include <TFT_eSPI.h>
#include "Orbitron_Medium_20.h"
#include "XBM_Icons.h"
#include "pin_config.h"

TFT_eSPI TFT = TFT_eSPI();

#define winX    0
#define winY   69 // centered in Gauge sprite window
#define winW  170
#define winH   84 //3*20 + padding of 6*4 (4 pixels extra space at above Line1, 8 between 2 lines (16) and 4 below Line3
void ShowTextWindow(const String Line1, const String Line2, const String Line3, uint16_t Pause) {
  // Clear and set window to display 3 lines of info -> centered
  int posX = 0;
  //TFT.fillScreen(TFT_BLACK);
  TFT.fillSmoothRoundRect(winX, winY, winW, winH, 4, TFT_NAVY);
  TFT.drawSmoothRoundRect(winX, winY, 5, 3, winW-1, winH, TFT_LIGHTGREY, TFT_NAVY);
  TFT.setFreeFont(&Orbitron_Medium_20);
  TFT.setTextColor(TFT_YELLOW, TFT_NAVY, true);
  if (Line1) {
    posX = round( (winW - TFT.textWidth(Line1)) / 2 );
    TFT.drawString(Line1, winX+posX, (winY+4) );
  }
  if (Line2) {
    posX = round( (winW - TFT.textWidth(Line2)) / 2 );
    TFT.drawString(Line2, winX+posX, (winY+8)+TFT.fontHeight() );
  }
  if (Line3) {
    posX = round( (winW - TFT.textWidth(Line3)) / 2 );
    TFT.drawString(Line3, winX+posX, (winY+12)+2*TFT.fontHeight() );
  }
  delay(Pause);  // Pause indicated time in ms
}

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
*/
#if defined(LILYGO_T_DISPLAY_ESP32_S3) 
  #define actuatorOutPin1 43   // -> pin 43 connected to pin IN2 of the Adafruit DRV8871 Motor Driver board
  #define actuatorOutPin2 44   // -> pin 44 connected to pin IN1 of the Adafruit DRV8871 Motor Driver board
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
#define BANDWIDTH 3

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

#if defined(LILYGO_T_DISPLAY_ESP32_S3) 
  // Pin 15 needs to be set to HIGH in order to boot without USB connection
  pinMode(PIN_POWER_ON, OUTPUT); // to boot with battery...
  digitalWrite(PIN_POWER_ON, HIGH);  // and/or power from 5v rail instead of USB
  Wire.begin(PIN_IIC_SDA, PIN_IIC_SCL); // Set I2C SDA and SCL for TOF sensor
#endif

  Serial.begin(115200);
  while (!Serial)
    delay(10);
  Serial.println("ESP32S3 T-Display SIMCLINE Functioning Diagnostics");
  Serial.println("------------------ Version 010 -------------------");

  // Setup T-Display 170x320
  TFT.init();
  TFT.fillScreen(TFT_BLACK);
  TFT.setRotation(0);  // 0 == USB downward 2 == USB Upward facing
  TFT.setSwapBytes(true); 
  TFT.drawXBitmap(0, 50, mountain170x136, 170, 136, TFT_LIGHTGREY, TFT_BLACK);
  delay(500);
  TFT.setTextColor(TFT_YELLOW, TFT_DARKGREY, true);
  TFT.setFreeFont(&Orbitron_Light_24);
  TFT.fillSmoothRoundRect(0, 200, 170, 32, 4, TFT_DARKGREY); 
  TFT.drawString("SIMCLINE", 19, 200);
  delay(1000);                    // Pause some time

  ShowTextWindow("Testing", "Motor", "Function", 100); 
  // Initialize Lifter Class data, variables and set to work
  lift.Init(actuatorOutPin1, actuatorOutPin2, MINPOSITION, MAXPOSITION, BANDWIDTH);
  delay(100);  
  if ( lift.TestBasicMotorFunctions() == false ) {
    ShowTextWindow("Motor", "Function", "FAILED!!", 500); 
    Serial.println("Basic Motor Funtions are FAILING!!");
    IsBasicMotorFunctions = false; // Not working properly
  } else {
    ShowTextWindow("Motor", "Function", "OK!!", 100);
    Serial.println("Basic Motor Funtions are working!!");
    IsBasicMotorFunctions = true;
  }
  delay(500); // Pause some time

#ifdef MANUAL_INPUT_DATA 
  Serial.println("Type 'u' for UP or 'd' for DOWN movement or '0' (zero) for neutral position, and close with <ENTER> to confirm!");
#endif
}

// Show testing values....
#define PADDING 4
#define CHAR2   2*16
#define CHAR3   3*16
void UpdateTFT(void) {
  TFT.setTextColor(TFT_YELLOW, TFT_DARKGREY, true);
  TFT.fillSmoothRoundRect(0, 200, 170, 32, 4, TFT_DARKGREY); 
  TFT.drawNumber(RawgradeValue, 19+CHAR2, 200+PADDING); 
  TFT.fillSmoothRoundRect(0, 240, 170, 32, 4, TFT_DARKGREY); 
  TFT.drawNumber(TargetPosition, 19+CHAR3, 240+PADDING);
  TFT.fillSmoothRoundRect(0, 280, 170, 32, 4, TFT_DARKGREY); 
  TFT.drawFloat(gradePercentValue, 1, 19+CHAR3, 280+PADDING);
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
        RawgradeValue = (20000 - MEASUREOFFSET);
        Serial.print("Level ");
        break;
      default :
        return;
      }
    } else return; 
#else
    if ( (++TargetPositionCount)>7) { TargetPositionCount = 0; }  
    RawgradeValue = TargetValuesArray[TargetPositionCount];  
    // Take into account the measuring offset
    RawgradeValue = RawgradeValue - MEASUREOFFSET;
#endif
// ---------------  Process and comply the test data -----------------------------------------
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
    // Give feedback and print values to account for
    gradePercentValue = float(RawgradeValue-(20000-MEASUREOFFSET)) / 100; 
#ifdef MANUAL_INPUT_DATA    
    Serial.printf("Move ms: %8d", millis()); 
#else
    Serial.printf("Step: [%1d] ms: %8d", TargetPositionCount, millis()); 
#endif
    Serial.printf("  Raw grade: %05d Grade perc.: %5.1f %%", RawgradeValue, gradePercentValue); 
    Serial.printf(" Target pos: %03d", TargetPosition, DEC); Serial.println();
    UpdateTFT(); // Show target values on TFT
    // Finally position actuator now!
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
