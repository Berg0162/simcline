// Basic code for testing function of Motor Driver Board
// Actuator should be connected to Motor Driver Board


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

void setup() 
   {
    //  setup control pins and set to lower trainer by default
    pinMode(actuatorOutPin1, OUTPUT);
    pinMode(actuatorOutPin2, OUTPUT);
    brakeActuator();
    }

void loop() 
  {
   moveActuator();
   delay(4000);
  }

void moveActuatorUp()
  { // FORWARD
  digitalWrite(actuatorOutPin1, HIGH);                           
  digitalWrite(actuatorOutPin2, LOW);  
  }

void moveActuatorDown()
  { // REVERSE
  digitalWrite(actuatorOutPin1, LOW);                           
  digitalWrite(actuatorOutPin2, HIGH);
  }
  
void brakeActuator()
  { // BRAKE
  digitalWrite(actuatorOutPin1, LOW);
  digitalWrite(actuatorOutPin2, LOW);
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
