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
*
* Requirements: DRV8871 Motor Driver Board connected to (a) functional Actuator and 
*               (b) an ESP32 or nRF52 board, with USB connection to the computer
* Diagnostics can be run with manual input of data for Up/Down movement of Actuator
* or with the continuous input of data from a predetermined set of test values! 
*
*/

#include <Arduino.h>

// -------------------------------------------------------------------------------------------
// COMPILER DIRECTIVE to help debugging...
// Comment out "#define MANUAL_INPUT_DATA" to activate: input of predetermined test values!
#define MANUAL_INPUT_DATA

// You need to set the board specification to handle correct pin assigments and power for the Stemma connector!!!
#define ADAFRUIT_FEATHER_ESP32_V2
//#define ARDUINO_NRF52840_FEATHER

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

#ifdef MANUAL_INPUT_DATA
char input; // Type 'u' for up movement or 'd' for down movement or '0' (zero) for neutral position
#endif

void setup() {

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

  Serial.println("nRF52/ESP32 DRV8871 + Actuator Functioning Diagnostics");
  Serial.println("--------------------- Version 010 --------------------");
     
  //  setup control pins and set brake by default
  pinMode(actuatorOutPin1, OUTPUT);
  pinMode(actuatorOutPin2, OUTPUT);
  brakeActuator();
#ifdef MANUAL_INPUT_DATA 
  Serial.println("Type 'u' for UP or 'd' for DOWN movement, and close with <ENTER> to confirm!");
#endif
}

void loop() {
#ifdef MANUAL_INPUT_DATA
    if(Serial.available()){
      input = Serial.read();
      switch (input)
      {
      case 117 : // u
        moveActuatorUp();
        delay(1000);
        brakeActuator();
        break;
      case 100 : // d
        moveActuatorDown();
        delay(1000);
        brakeActuator();
        break;
      case 48 : // 0
        brakeActuator();
        break;
      default :
        return;
      }
    } else return; 
#else
    moveActuator();
#endif
}

void moveActuatorUp()
  { // FORWARD
  digitalWrite(actuatorOutPin1, LOW);                           
  digitalWrite(actuatorOutPin2, HIGH); 
  Serial.print("Move Up  ");
  }

void moveActuatorDown()
  { // REVERSE
  digitalWrite(actuatorOutPin1, HIGH);                           
  digitalWrite(actuatorOutPin2, LOW);
  Serial.print("Move Down");
  }
  
void brakeActuator()
  { // BRAKE
  digitalWrite(actuatorOutPin1, LOW);
  digitalWrite(actuatorOutPin2, LOW);
  Serial.println(" > Brakes On");
  }

void moveActuator(void) 
  { // TEST FUNCTION 
  moveActuatorUp();
  delay(1000);
  brakeActuator();
  delay(1000);
  moveActuatorDown();
  delay(1000);
  brakeActuator();
  delay(2000);
  }
