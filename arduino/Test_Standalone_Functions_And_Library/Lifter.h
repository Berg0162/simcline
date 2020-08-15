/*
 * 
 * 
 * 
 */
#ifndef Lifter_h
#define Lifter_h

#include "Arduino.h"
#include "VL6180X.h"

class Lifter {
 
  VL6180X sensor;
  bool _IsBrakeOn;
  bool _IsMovingUp;
  bool _IsMovingDown;
  int _actuatorOutPin1;
  int _actuatorOutPin2;
  int16_t _TargetPosition;
  int16_t _CurrentPosition;
  int _BANDWIDTH;
  int _MINPOSITION;
  int _MAXPOSITION;
  
public:
 
  Lifter();
  void Init(int OutPin1, int OutPin2, int MINPOS, int MAXPOS, int BANDWTH);
  int16_t GetVL6180X_Range_Reading();
  void SetTargetPosition(int16_t Tpos);
  bool TestBasicMotorFunctions();
  int GetOffsetPosition();
  void moveActuatorUp();
  void moveActuatorDown();
  void brakeActuator();
};

#endif
