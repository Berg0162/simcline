// Basic test code for MDD3A board
//
#define actuatorOutPin1 2    // To the level shifter and then to the H Bridge
#define actuatorOutPin2 3    // To the level shifter and then to the H Bridge

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
  delay(1000);
  }
