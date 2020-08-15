#include "Lifter.h"
#include "SPI.h"
#include "Wire.h"

//#include "Adafruit_GFX.h"
//#include "Adafruit_SSD1306.h"

#include "MovingAverageFilter.h"
// Declare the running average filter for VL6180X Range measurements
// Filter is only used in the Lifter Class 
// Sampling is at about 10 Hz --> 10 VL6180X-RANGE-readings per second
#define NUMBER_OF_RANGE_READINGS 10
// Instantiate MovingAverageFilter class
  MovingAverageFilter movingAverageFilter_Range(NUMBER_OF_RANGE_READINGS);
  
// Instantiate Lifter class
  Lifter::Lifter() {
  }

void Lifter::Init(int OutPin1, int OutPin2, int MINPOS, int MAXPOS, int BANDWTH)
{
  _actuatorOutPin1 = OutPin1;
  _actuatorOutPin2 = OutPin2;
  //  setup control pins and set to BRAKE by default
  pinMode(_actuatorOutPin1, OUTPUT);
  pinMode(_actuatorOutPin2, OUTPUT);
  // set brake
  digitalWrite(_actuatorOutPin1, LOW);
  digitalWrite(_actuatorOutPin2, LOW);
  // 
  _BANDWIDTH = BANDWTH;
  _MINPOSITION = MINPOS;
  _MAXPOSITION = MAXPOS;

  // NOTICE: COMPILER DIRECTIVE !!!!
  // that leaves out almost all print statements!
//#define DEBUG is off !!
  
#ifdef DEBUG
  Serial.print("Pin1: "); Serial.print(_actuatorOutPin1); Serial.print(" Pin2: "); Serial.print(_actuatorOutPin2); 
  Serial.print(" BandWidth: "); Serial.print(_BANDWIDTH); Serial.print(" MinPosition: "); Serial.print(_MINPOSITION);
  Serial.print(" MaxPosition: "); Serial.print(_MAXPOSITION); Serial.println();  
#endif
// Private variables for position control
  _IsBrakeOn = true;
  _IsMovingUp = false;
  _IsMovingDown = false;
  _TargetPosition = 400; // Choose the safe value of a flat road
  
// setup wire communication and defaults for the VL6180X
  Wire.begin();
  
// setup VL6180X settings and operating mode
// Range Continuous or Single Shot, read the manual.... 
// NOTICE: COMPILER DIRECTIVE !!!!
  #define RANGE_CONTINUOUS
// Set scaling (after configureDefault = 1) of the VL6180X to approriate value 1, 2 or 3
//  Only scaling factors  #3 will work in our situation of 30+ cm range !!!!
  #define _SCALING 3
  sensor.init();
  sensor.configureDefault();
  sensor.setScaling(_SCALING);
// Single shot operating mode of VL6180X is simplest and default
// The following is extra code critical for using Continuous mode !!!
  #ifdef RANGE_CONTINUOUS
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
#ifdef DEBUG
  Serial.print(" ---------- VL6180X Range Continuous Mode Selected ---------"); Serial.println();
#endif
  #endif
  sensor.setTimeout(500);
  
// fill the movingAverageFilter with actual values instead of default zero's....
// that blur operation in the early stages (of testing..)
  for (int i = 0; i < 10; i++) {
    _CurrentPosition = GetVL6180X_Range_Reading();
    } 
}

int16_t Lifter::GetVL6180X_Range_Reading()
{
 #ifdef RANGE_CONTINUOUS
    int16_t temp = sensor.readRangeContinuousMillimeters();
 #else
    int16_t temp = sensor.readRangeSingleMillimeters();
 #endif
    if (sensor.timeoutOccurred()) 
        { 
#ifdef DEBUG
        Serial.print(" TIMEOUT"); Serial.println();
#endif
        }
  return movingAverageFilter_Range.process(temp);  
}

bool Lifter::TestBasicMotorFunctions()
{
#ifdef DEBUG 
  Serial.print("Testing VL6180X and motor functioning..."); Serial.println();
#endif  
  int16_t PresentPosition01 = GetVL6180X_Range_Reading();
#ifdef DEBUG  
  Serial.print("Start at position: "); Serial.print(PresentPosition01);Serial.println();
#endif
  if (PresentPosition01 != (constrain(PresentPosition01, _MINPOSITION, _MAXPOSITION)) )
  { // VL6108X is out of Range ... ?
#ifdef DEBUG
    Serial.print(">> ERROR << -> VL6108X Out of Range at start !!"); Serial.println();
#endif
    return false; 
  }
#ifdef DEBUG
  Serial.print("Moving UP ..."); Serial.println();
#endif
  moveActuatorUp();
  delay(800); // Wait for some time
  brakeActuator();
  
  for (int i = 0; i < 10; i++) {  // Determine precise position after 800 ms of movement !
     _CurrentPosition = GetVL6180X_Range_Reading();
     } 
 
  int16_t PresentPosition02 = (_CurrentPosition + _BANDWIDTH);
  if (PresentPosition02 != (constrain(PresentPosition02, _MINPOSITION, _MAXPOSITION)) )
  { // VL6108X is out of Range ... ?
#ifdef DEBUG
    Serial.print(">> ERROR << -> VL6108X Out of Range"); Serial.println();
#endif
    return false; 
  }
  if (!(PresentPosition02 < PresentPosition01))
    { 
#ifdef DEBUG
    Serial.print(">> ERROR << -> VL6108X did not detect an UP movement"); Serial.println();
#endif
    return false;
    }
  // VL6108X is properly working moving UP! ------------------------------------
#ifdef DEBUG  
  Serial.print("Moving Down ..."); Serial.println();
#endif
  moveActuatorDown();
  delay(1600); // Wait some time (extra to "undo" the previous Up movement!!)
  brakeActuator();
  
  for (int i = 0; i < 10; i++) {  // Determine precise position after 2400 ms of movement !
     _CurrentPosition = GetVL6180X_Range_Reading();
     } 
 
  PresentPosition01 = (_CurrentPosition - _BANDWIDTH);
  if (PresentPosition01 != (constrain(PresentPosition01, _MINPOSITION, _MAXPOSITION)) )
  { // VL6108X is out of Range ... ?
#ifdef DEBUG
    Serial.print(">> ERROR << -> VL6108X Out of Range"); Serial.println();
#endif
    return false; 
  }
  if (!(PresentPosition01 > PresentPosition02))
    { 
#ifdef DEBUG
    Serial.print(">> ERROR << -> VL6108X did not detect a DOWN movement"); Serial.println();
#endif
    return false;
    }
  // AND VL6108X is properly moving DOWN ! --------------------------------------
#ifdef DEBUG  
  Serial.print("VL6180X and motor properly working ..."); Serial.println();
#endif
  return true;
}


int Lifter::GetOffsetPosition()
{
  _CurrentPosition = GetVL6180X_Range_Reading();
  int16_t _PositionOffset = _TargetPosition - _CurrentPosition;
  if (sensor.timeoutOccurred()) 
    {
#ifdef DEBUG 
      Serial.print(" TIMEOUT"); Serial.println();
#endif
    return 3;
    }
#ifdef DEBUG
  Serial.print("Target: "); Serial.print(_TargetPosition);
  Serial.print("  Current: "); Serial.print(_CurrentPosition);
  Serial.print("  Offset: "); Serial.print(_PositionOffset); 
#endif
  /* Show position on OLED
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);  // Large characters
  display.setCursor(36,2);
  display.print(_PositionOffset);
  display.setTextSize(2);  // Large characters
  display.setCursor(32,22);
  display.print(_CurrentPosition); 
  display.display(); 
  */
  
  if ( (_PositionOffset >= -_BANDWIDTH) && (_PositionOffset <= _BANDWIDTH) )
    { // postion = 0 + or - BANDWIDTH so don't move anymore!
#ifdef DEBUG
    Serial.print(" offset = 0 (within bandwidth) "); Serial.println();
#endif
    return 0; 
    }
  if ( _PositionOffset < 0 )
  {
#ifdef DEBUG
   Serial.print(" offset < 0 "); Serial.println();
#endif
   return 1;    
  }
  else 
  {
#ifdef DEBUG
   Serial.print(" offset > 0 "); Serial.println();
#endif
   return 2;     
  }
  // default --> error... stop!
#ifdef DEBUG
  Serial.print(" BRAKE --> Offset comparison error!"); Serial.println();
#endif
  return 0; 
}

void Lifter::SetTargetPosition(int16_t Tpos)
{
  _TargetPosition = Tpos;
}

void Lifter::moveActuatorUp()
  { 
  // FORWARD
  if (_CurrentPosition <= (_MINPOSITION + _BANDWIDTH) )
    { // Stop further movement to avoid destruction...
    _IsMovingUp = false;
#ifdef DEBUG
    Serial.print(" Stop MovingUp ");
#endif
    brakeActuator();
    return;
    }
    // DO NOT REPEATEDLY set the same motor direction 
    if (_IsMovingUp) {return;}
    
    digitalWrite(_actuatorOutPin1, LOW);                           
    digitalWrite(_actuatorOutPin2, HIGH);
    delay(200);
    _IsMovingUp = true;
    _IsMovingDown = false;
    _IsBrakeOn = false;
#ifdef DEBUG
    Serial.println(" Set MovingUp ");
#endif 
  }

void Lifter::moveActuatorDown()
  { 
  // REVERSE
  if (_CurrentPosition >= (_MAXPOSITION - _BANDWIDTH) )
    { // Stop further movement to avoid destruction...
    _IsMovingDown = false;
#ifdef DEBUG
    Serial.println(" Stop MovingDown ");
#endif
    brakeActuator();
    return;
    }
    // DO NOT REPEATEDLY set the same motor direction
    if (_IsMovingDown) {return;}
    
    // moving in the wrong direction or not moving at all
    digitalWrite(_actuatorOutPin1, HIGH);                           
    digitalWrite(_actuatorOutPin2, LOW);
    delay(200);
    _IsMovingDown = true;
    _IsMovingUp = false;
    _IsBrakeOn = false;
#ifdef DEBUG
    Serial.println(" Set MovingDown ");
#endif
  }

void Lifter::brakeActuator()
  { 
    // BRAKE
    // DO NOT REPEATEDLY stop the motor
    if (_IsBrakeOn) {return;}
    digitalWrite(_actuatorOutPin1, LOW);
    digitalWrite(_actuatorOutPin2, LOW);
    delay(200);
    _IsBrakeOn = true;
    _IsMovingDown = false;
    _IsMovingUp = false; 
#ifdef DEBUG   
    Serial.println(" Set Brake On ");
#endif
  }
