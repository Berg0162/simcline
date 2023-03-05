# <img src="https://github.com/Berg0162/simcline/blob/master/images/SC_logo.png" width="64" height="64" alt="SIMCLINE Icon"> &nbsp; SIMCLINE for Wahoo KICKR
# Simulation of Changing Road Inclination for Indoor Cycling<br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/Simcline_And_Wheel.jpg" width="300" height="300" ALIGN="left" alt="SIMCLINE">
<img src="https://github.com/Berg0162/simcline/blob/master/images/Simcline_2_0.jpg" width="300" height="300" ALIGN="left" alt="SIMCLINE 2">
The SIMCLINE physically adjusts the bike position to mimic hilly roads, climbing and descending. This allows the rider to naturally change position on the bike, engage climbing muscles, and improve pedaling technique to become a more efficient and powerful climber.<br>
Without user intervention the SIMCLINE will replicate inclines and declines depicted in (online & offline) training programs (like Zwift, Rouvy, VeloReality and many others) that adjust accordingly the resistance of the indoor trainer.<br>
The SIMCLINE auto connects at power up with a smart Wahoo KICKR trainer and let's relive the ascents and descents from favorite rides or routes while training indoors.<br>
The physical reach is: 20% maximum incline and -10% maximum decline. However, the reach that the rider is comfortable with can be adjusted!<br>
The SIMCLINE pairs directly to the Wahoo KICKR smart trainer and with your PC/Laptop with (Zwift) training App for a connection that notifies the SIMCLINE to simulate autonomous the (change in) physical grade of the road during an indoor ride.<br>
During operation an OLED display shows the road grade in digits and in graphics.<br>
The SIMCLINE Companion App (for Android smartphones) can be paired, only when the training App is disconnected, for adjusting operational settings, like Ascent Grade Limit (between 0-20%), Descent Grade Limit (between 0-10%), Road Grade Change Factor (between 0-100%) and manual Up and Down control. Notice that the Companion App has a slightly different functionality depending of what brand of trainer (TACX or Wahoo) is connected, due to specific connectivity differences. <br clear="left"> 

# Credits to Christian B. from Canada
Christian was one of those that have a Wahoo KICKR trainer and built the SIMCLINE design for personal use. However, he could not apply the SIMCLINE code as it was! We have collaborated very closely during a teamwise journey to explore all possible scenarios to realize the SIMCLINE design and code to operate with a Wahoo KICKR trainer, in addition to a TACX trainer! Christian's expertise and effort was focussed on testing with Wahoo KICKR and figure out how (Zwift-KICKR) communication is implemented. He collected tons of valuable information that helped to find out how things work, pinpoint causes of failures and at the end to construct the present working solution. Without his valuable effort this project would never have taken place!<br>
If you need to modify your Wahoo Kickr trainer, see Christian's effort: [Free Axle for Wahoo KICKR Gen2](https://www.instructables.com/Free-Axle-for-Wahoo-Kickr-Gen2-2018/)<br>

# Man-In-The-Middle (MITM) software pattern<br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/Man_in_the_Middle.png" align="left" width="1000" height="500" alt="Man in the Middle"><br>
TACX published in 2015 a document [TACX, FE-C and Bluetooth](https://github.com/Berg0162/simcline/blob/master/docs/How_to_FE_C_over_BLE_v1_0_0.pdf) that explains how to use the FE-C ANT+ protocol over BLE feature implemented on all(?) TACX Smart Trainers. TACX designed this feature because at that time an open standard (on BLE) for trainers was lacking. However Wahoo did not officially publish any document describing their proprietary protocol to control a Wahoo KICKR over BLE. The major training App developers were invited (by Wahoo) to design their Apps for use with Wahoo products. No doubt they had to sign a non-disclosure agreement. While TACX has been transparent, Wahoo was definitely NOT! However, the internet is a great source for information about not so transparent companies. At first the SIMCLINE was designed to successfully work with smart TACX trainers and after publication in the public domain several people applied the design to built their own. However most remarks/questions I received were about the application of a Wahoo KICKR trainer. Early in the year 2022, we decided to put a jont effort in modifying the Simcline code in such a way that it would operate with a Wahoo KICKR. After other scenarios had failed, I have chosen to apply the Man-in-the-middle software pattern, since that should work with all smart Wahoo KICKR trainers, young and relatively older!<br>

<b>Man-In-The-Middle</b> is a powerful software engineering pattern that is applied in many software designs. Unfortunately it is also known for a negative application in communication traffic: MITM is a common type of cybersecurity attack that allows attackers to eavesdrop on the communication between two targets.
We have applied the very principle: the Simcline is strategicly positioned in between the BLE communication of the Wahoo KICKR Trainer and the training App (like Zwift) running on the PC/Laptop, all communication traffic can be inspected in that MITM position, when it is passed on from one to the other, in both directions. When Zwift sends resistance information (like the road inclination) to the Wahoo KICKR, this information can be intercepted and applied to determine the up/down positioning of the Simcline. <br>

Meanwhile there is well documented (<b>FTMS</b>) FiTness Machine Service protocol to control fitness equipment over Bluetooth. According to the smart trainer recommendations guide winter 2019-2020 of [DCRainmaker](https://www.dcrainmaker.com/2019/10/the-smart-trainer-recommendations-guide-winter-2019-2020.html) the situation evolved:
> Meanwhile, for Bluetooth Smart, there’s FTMS, which is basically the same thing as FE-C when it comes to trainers. It’s not quite as widely adopted yet by trainer companies, but is by app companies. On the trainer company side only Elite, Saris, and Kinetic support it across the board. With Tacx having it on some but not all units, and Wahoo having it on no units (but all Wahoo and Tacx trainers support a <b>proprietary</b> Bluetooth Smart with all major apps anyway).<br>

A next generation of Simclines will hopefully be based on this FTMS, when all companies embrace its open standard, however, today we still have to open our box of tricks! <br>

# How to start?<br>
+ Install the [Arduino IDE](https://www.arduino.cc/en/Main/Software) and all the libraries on a PC/Mac.
+ Download all the code from [Github](https://github.com/Berg0162/simcline) and install.
<br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/Simcline_circuitry_02.jpg" align="left" width="200" height="200" alt="Cardboard">
When I started the project in 2020 I did not have any practical experience with any of the components. So I had to setup the circuitry step by step adding components and did a lot of time consuming but instructive testing first. My advice is to setup (<b>some of</b>) the electronic components first in a similar way as shown on the photo with the cardboard base. Use double sided adhesive tape but only attach it on sections that have no pcb-wiring or soldering, to avoid possible electrical interference. You will find in this Github repository all the code that controls the Simcline and the Arduino test programs (modified for this project) that focus on components separately and in conjunction. <br clear="left">

# To see is to believe!<br>
I can understand and respect that you have some reserve: Is this really working in my situation? Better test if it is working, before buying all components and start building.
In the Github repository (see above) you will find the appropriate test code named: <b>Wahoo_Client</b>, <b>Wahoo_Server</b> and <b>Wahoo_Zwift_Bridge</b>. It is coded with the only intention to check if the MITM solution is delivering in your specific situation.<br>

<b>What it does in short:</b><br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/Wahoo_Feather_Zwift_BLE.png" align="left" width="1000" height="500" alt="Simcline in the Middle"><br>
The <b>Wahoo_Zwift_Bridge</b> code links a bike trainer (BLE Server Wahoo KICKR) and a PC/Laptop (BLE Client running Zwift) with the Feather nRF52840/832, like a bridge in between. The MITM bridge can pass on, control, filter and alter the interchanged trafic data! This test code is fully ignorant of the mechanical components that drive the Simcline. It simply estabishes a virtual BLE bridge and allows you to ride the bike on the Wahoo trainer and feel the resistance that comes with it, thanks to Zwift. The experience should not differ from a normal direct one-to-one connection, Zwift - Wahoo KICKR!<br>
+ The client-side scans and connects with the Wahoo relevant Cycling Power Service (<b>CPS</b>) plus the additional Wahoo proprietary CPS characteristic and collects cyling power data like Zwift would do! The code with the name: <b>Wahoo_Client</b> is doing just that one side of the "bridge"!
+ The Server-side advertises and enables connection with Cycling apps like Zwift and collects relevant resistance data, it simulates as if an active Wahoo trainer is connected to Zwift or alike! The code with the name: <b>Wahoo_Server</b> is doing just the other side of the "bridge"!
+ The <b>Wahoo_Zwift_Bridge</b> code is connecting both sides at the same time: the full-blown bridge!<br clear="left">

<b>How to make it work?</b><br>
The requirements are simple: 
+ running Zwift app or alike, 
+ working Feather nRF52840/52832 board <u>plus</u> SSD1306 Oled display and 
+ a Wahoo KICKR trainer.<br>

<b>Use the test code for reconnaissance!</b><br>
Please follow the instructions at the first part of the program code!
+ Start your reconnaissance with running <b>Wahoo_Client</b> and experience how the Feather is controlling the resistance of your Wahoo trainer. 
+ Next step is running <b>Wahoo_Server</b>, pairing with Zwift and then notice how your avatar is moving effortless in the Zwift world controlled by the Feather.<br>

<i>The 2 test programs (Client and Server) are NOT using a SSD1306 display, only Serial Monitor to show what is happening!</i><br>
Please write down the MAC or Device Addresses of a) your Wahoo trainer and b) your Desktop/Laptop with Zwift. These are presented in the Serial Monitor log file when running the Client and Server test code.<br>

<b>Now it is time to test the bridge!</b><br>
The <b>Wahoo_Zwift_Bridge</b> code needs these "hardware" addresses to unmistakingly establish a BLE connection with the right device. I know it can be implemented differently but this is to avoid unwanted BLE connection(s) with an additional power meter, another fitness device or a second computer/laptop, etcetera. 
```C++
.
// -----------------------------------------------------------------
// Your hardware MAC/DEVICE ADDRESSES
// Laptop/Desktop Device Address that runs Zwift: [00:01:02:03:04:05]
// Little Endian: in reversed order !!!!
#define LAPTOPADDRESS {0x05,0x04,0x03,0x02,0x01,0x00}
// Trainer Wahoo KICKR Device Address [00:01:02:03:04:05]
// Little Endian: in reversed order !!!!
#define TRAINERADDRESS {0x05,0x04,0x03,0x02,0x01,0x00}
// -----------------------------------------------------------------
.
```
The two precise device addresses are critical to assure a reliable test! You have to insert the values in the program code!<br> 

1) First insert in the <b>Wahoo_Zwift_Bridge</b> code the two precise BLE MAC Addresses it has to connect with
2) Upload and Run this code on the Feather nRF52840
2) Start the Serial Monitor to catch debugging info
3) Start/Power-On the Wahoo trainer  
4) Feather and Trainer will pair as reported on the Serial Monitor
5) Start Zwift on your computer or tablet
6) Search on Zwift pairing screen "<b>Power</b>" for the Feather nRF52 a.k.a. "<b>Sim Wahoo</b>"
7) Pair <b>Power</b> and <b>Controllable</b> with "<b>Sim Wahoo</b>"
8) Notice Wahoo does NOT support Speed nor Cadence, optionally pair with alternative
9) After successful pairing start the default Zwift ride or any ride you wish
10) Make Serial Monitor visible on top of the Zwift window 
11) Hop on the bike and make it happen..
12) Inspect the info presented by Serial Monitor and check the SSD1306 for the Zwift road inclination values.....
<br clear="left">

# Overview of Arduino Program Code Flow and Snippets<br>
+ Include headers of libraries and declare classes
```C++
.
#include <bluefruit.h>
// Compile toggle that determines Serial Monitor is switched ON (1) or OFF (0)
#define Serial_Monitor 1
// Libraries for use of I2C devices (Oled and VL6180X distance sensor)
#include <SPI.h>
#include <Wire.h>
// Necessary libraries for use of Oled display(s)
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// Additional splash screen bitmap and icon(s) for Oled
#include "Adafruit_SSD1306_Icons.h" // needs to be in directory of main code
// Declarations for an SSD1306 128x64 display connected to I2C (SDA, SCL pins)
#define SCREEN_WIDTH 128            // SSD1306-OLED display width, in pixels
#define SCREEN_HEIGHT 64            // SSD1306-OLED display height, in pixels
#define OLED_RESET -1               // No reset pin on this OLED display
#define OLED_I2C_ADDRESS 0x3C       // I2C Address of OLED display
// Declare the display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#include <avr/dtostrf.h>
.
```
+ Declare crucial BLE services, variables and charateristics 
```C++
.
/* Cycling Power Service
 * CP Service: 0x1818  
 * CP Characteristic: 0x2A63 (Measurement)
 * CP Characteristic: 0x2A65 (Feature)
 * CP Characteristic: 0x2A5D (Location) NOT supported in Wahoo legacy trainers
 * CP Characteristic: 0x2A66 (CYCLING_POWER_CONTROL_POINT)
 */
 // --------------------------------------------------------------------------------------------------------------------------------------------
BLEService        server_cps = BLEService(UUID16_SVC_CYCLING_POWER);
// Define how Server CPS is advertised
const uint16_t Server_appearance = 0x0480;  // 1152 -> Cycling
BLECharacteristic server_cpmc = BLECharacteristic(UUID16_CHR_CYCLING_POWER_MEASUREMENT);
BLECharacteristic server_cpfc = BLECharacteristic(UUID16_CHR_CYCLING_POWER_FEATURE);
BLECharacteristic server_cplc = BLECharacteristic(UUID16_CHR_SENSOR_LOCATION);
BLECharacteristic server_cpcp = BLECharacteristic(UUID16_CHR_CYCLING_POWER_CONTROL_POINT); // Indicate, write

// Hidden Wahoo Trainer characteristic to the Cycling Power Service !
// Unknown Characteristic is 128 bit:            A0 26 E0 05 - 0A 7D - 4A B3 - 97 FA - F1 50 0F 9F EB 8B
// Declare in Reversed order !!!
uint8_t CYCLING_POWER_WAHOO_TRAINER_Uuid[16] = {0x8B, 0xEB, 0x9F, 0x0F, 0x50, 0xF1, 0xFA, 0x97, 0xB3, 0x4A, 0x7D, 0x0A, 0x05, 0xE0, 0x26, 0xA0};
BLECharacteristic server_cpwt = BLECharacteristic(CYCLING_POWER_WAHOO_TRAINER_Uuid);

// Server helper class instance for Device Information Service
BLEDis bledis;

//-----------------------------------------------------------------------------------------------------------------------------------------------
BLEClientService        client_cps(UUID16_SVC_CYCLING_POWER);
BLEClientCharacteristic client_cpmc(UUID16_CHR_CYCLING_POWER_MEASUREMENT);
BLEClientCharacteristic client_cpfc(UUID16_CHR_CYCLING_POWER_FEATURE);
uint32_t client_cpfcDef = 0;
BLEClientCharacteristic client_cplc(UUID16_CHR_SENSOR_LOCATION);
uint16_t client_cplc_loc_value = 0;
BLEClientCharacteristic client_cpcp(UUID16_CHR_CYCLING_POWER_CONTROL_POINT); // Indicate, write
BLEClientCharacteristic client_cpwt(CYCLING_POWER_WAHOO_TRAINER_Uuid);
.
```
```C++
.
// CPS Wahoo Trainer Operation Codes in Decimal
const uint8_t unlock                     = 32;
const uint8_t setResistanceMode          = 64;
const uint8_t setStandardMode            = 65;
const uint8_t setErgMode                 = 66;
const uint8_t setSimMode                 = 67;
const uint8_t setSimCRR                  = 68;
const uint8_t setSimWindResistance       = 69;
const uint8_t setSimGrade                = 70;
const uint8_t setSimWindSpeed            = 71;
const uint8_t setWheelCircumference      = 72;
const uint8_t UnlockCommandBuf[3]        = {unlock, 0xEE, 0xFC}; // Unlock codes
.
```
```C++
.
#define SERVICE_DEVICE_INFORMATION                  0x180A
#define CHARACTERISTIC_MANUFACTURER_NAME_STRING     0x2A29
#define CHARACTERISTIC_MODEL_NUMBER_STRING          0x2A24
#define CHARACTERISTIC_SERIAL_NUMBER_STRING         0x2A25

// Client Service Device Information
BLEClientService        client_diss(SERVICE_DEVICE_INFORMATION);
BLEClientCharacteristic client_disma(CHARACTERISTIC_MANUFACTURER_NAME_STRING);
BLEClientCharacteristic client_dismo(CHARACTERISTIC_MODEL_NUMBER_STRING);
BLEClientCharacteristic client_dissn(CHARACTERISTIC_SERIAL_NUMBER_STRING);
.
```
```C++
.
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
.
// Declaration of Function prototypes
int16_t EMA_TargetPositionFilter(int16_t current_value); // EMA filter function for smoothing Target Position values
void ShowIconsOnTopBar(void);
void ShowOnOledLarge(const char* Line1, const char* Line2, const char* Line3, uint16_t Pause);
void ShowSlopeTriangleOnOled(void);

void SetNewRawGradeValue(float RoadGrade);
void SetManualGradePercentValue(void);
void SetNewActuatorPosition(void);
void ControlUpDownMovement(void);

bool getPRSdata(void);
void setPRSdata(void);

void StartAdvBLEuart(void);
void prph_bleuart_rx_callback(uint16_t conn_handle);

void Setup_Client_CPS(void);
void Client_Start_Scanning(void);
void client_scan_callback(ble_gap_evt_adv_report_t* report);
void Client_Enable_Notify_Indicate(void);
void Client_connect_callback(uint16_t conn_handle);
void Get_client_Diss(uint16_t conn_handle);
void Client_disconnect_callback(uint16_t conn_handle, uint8_t reason);
void client_cpcp_indicate_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len);
void client_cpmc_notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len);
void client_cpwt_indicate_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len);

void Setup_Server_CPS(void);
void Start_Server_Advertising(void);
void server_cpwt_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len);
void server_cpcp_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len);
void Periph_adv_stop_callback(void);
void Periph_connect_callback(uint16_t conn_handle); 
void Periph_disconnect_callback(uint16_t conn_handle, uint8_t reason);
void server_cccd_callback(uint16_t conn_handle, BLECharacteristic* chr, uint16_t cccd_value);
// End Function Definitions
```
<b>Begin of the Arduino Setup() Function</b><br>
+ Get or set (first time only) the values of relevant and crucial variables to persistence, whith the Companion App the user can set these on the fly!
+ Start the show for the SSD1306 Oled display.
+ Initialize Lifter Class data, variables, test and set to work!
```C++
.
  //Show Name and SW version on Oled
  ShowOnOledLarge("SIMCLINE", "Wahoo", "3.4.2", 500);
  // Initialize Lifter Class data, variables, test and set to work !
  lift.Init(actuatorOutPin1, actuatorOutPin2, MINPOSITION, MAXPOSITION, BANDWIDTH);
.
```
```C++
.
  // Test Actuator and VL8106X for proper functioning
  ShowOnOledLarge("Testing", "Up & Down", "Functions", 100);
  if (!lift.TestBasicMotorFunctions()) {
    ShowOnOledLarge("Testing", "Functions", "Failed!", 500);
    IsBasicMotorFunctions = false; // Not working properly
#if Serial_Monitor  
    Serial.println("Basic Motor Funtions are NOT working!!");
#endif
    }
  else {
    ShowOnOledLarge("Testing", "Functions", "Done!", 500);
    // Is working properly
    IsBasicMotorFunctions = true;
#if Serial_Monitor  
    Serial.println("Basic Motor Funtions are working!!");
#endif
    // Put Simcline in neutral: flat road position
    // Init EMA filter at first call with flat road position as reference
    TargetPosition = EMA_TargetPositionFilter(TargetPosition); 
    SetNewActuatorPosition();
    ControlUpDownMovement();
  }
.    
```
+ Initialize BLE with maximum connections as Peripheral = 1, Central = 1 and start...
```C++
.
  // begin (Peripheral = 1, Central = 1)
  Bluefruit.begin(1, 1);
  // Set the device name (keep it short!) 
#if Serial_Monitor
  Serial.println("Setting Device Name to 'Wahoo Sim'");
#endif
  // Supported tx_power values depending on mcu:
  // - nRF52832: -40dBm, -20dBm, -16dBm, -12dBm, -8dBm, -4dBm, 0dBm, +3dBm and +4dBm.
  // - nRF52840: -40dBm, -20dBm, -16dBm, -12dBm, -8dBm, -4dBm, 0dBm, +2dBm, +3dBm, +4dBm, +5dBm, +6dBm, +7dBm and +8dBm.
  Bluefruit.setTxPower(4); // See above for supported values: +4dBm
  Bluefruit.setName("Wahoo Sim");

  Setup_Client_CPS();
  Client_Start_Scanning();
  while (Bluefruit.Scanner.isRunning()) { // do nothing else but scanning....
  }
#if Serial_Monitor 
  Serial.println("Scanning for Wahoo Cycle Power Service is stopped!");
#endif  
  TimeInterval = millis() + TIME_SPAN; // ADD just enough delay
  // wait enough time or go on when client is connected and set!
  while ( (TimeInterval > millis()) || (!Trainer.IsConnected) ) {
    }

  // Setup the Server Cycle Power Service
  // BLEService and BLECharacteristic classes initialized
  Setup_Server_CPS();
  Start_Server_Advertising();
  
  while (Bluefruit.Advertising.isRunning()) { // ONLY advertise!
  }
#if Serial_Monitor 
  Serial.println("Wahoo Simulated Advertising stopped! Paired to Zwift?");
#endif  
  TimeInterval = millis() + TIME_SPAN; // ADD just enough DELAY
  // wait enough time or go on when server is connected and set!
  while ( (TimeInterval > millis()) || (!Laptop.IsConnected) ) { 
    }
  // Only now enable Client (Wahoo) data streams...
  Client_Enable_Notify_Indicate(); 
#if Serial_Monitor   
  Serial.println("Up and running!");
#endif
  // Initialize BLE Uart functionality for connecting to smartphone No advertising!!
  bleuart.begin();
  // End Setup
.
```
+ Actuator movement is controlled in this function
```C++
.
void ControlUpDownMovement(void) // Move fully controlled to the right position
{
  // Check Position and move Up/Down until target position is reached, 
  // BLE channels can interrupt and change target position on-the-fly !!
 
  if (!IsBasicMotorFunctions) { return; } // do nothing that can damage construction!!
  int OnOffsetAction = 0;
  InControlUpDownMovementLoop = true;
do {
  OnOffsetAction = lift.GetOffsetPosition(); // calculate offset to target and determine action
  switch (OnOffsetAction)
  {
    case 0 :
        lift.brakeActuator();
#if Serial_Monitor_Movement
      Serial.println(F(" -> Brake"));
#endif
      break;
    case 1 :
      lift.moveActuatorUp();
#if Serial_Monitor_Movement
      Serial.println(F(" -> Upward"));
#endif
      break;
    case 2 :
      lift.moveActuatorDown();
#if Serial_Monitor_Movement
      Serial.println(F(" -> Downward"));
#endif
      break;
    case 3 :
      // Timeout --> OffsetPosition is undetermined --> do nothing and brake
      lift.brakeActuator();
#if Serial_Monitor_Movement
      Serial.println(F(" -> Timeout"));
#endif
      break;
  }
} while ( (OnOffsetAction == 1) || (OnOffsetAction == 2) ); // Run the loop until target position is reached!
  InControlUpDownMovementLoop = false;
} // end 
.    
```
+ Road inclination data are extracted from the Server CPWT Cycling Power Wahoo Trainer data field
```C++
.
void server_cpwt_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{    
  // Server CPWT Cycling Power Wahoo Trainer data field
  // Transfer CPWT data from the Server (Zwift) to the Client (Wahoo)
  client_cpwt.write_resp(data, len); // Just pass on and process later!
  // Process to extract critical data values
  uint8_t cpwtDataLen = (uint8_t)len;    // Get the actual length of data bytes and type cast to (uint8_t)
  uint8_t cpwtData[cpwtDataLen];
  memset(cpwtData, 0, cpwtDataLen); // set to zero
  // Display the raw request packet actual length
#if Serial_Monitor
  Serial.printf("Server CPWT Data [Len: %d] [Data:  ", len);
#endif
  // Transfer the contents of data to cpwtData
  for (int i = 0; i < cpwtDataLen; i++) {
    if ( i <= sizeof(cpwtData)) {
      cpwtData[i] = *data++;
#if Serial_Monitor
      // Display the raw request packet byte by byte in HEX
      Serial.printf("%02X ", cpwtData[i], HEX);
#endif
    }
  }
.
.
  if (cpwtData[0] == setSimGrade) {
    uint16_t gr = ( cpwtData[1] + (cpwtData[2]*256) ); // This works perfect !!!
    float SimGrade = 100 * float( ((gr * 2.0 / 65535) - 1.0) ); // Percentage of road grade --> range: between +1 and -1 (!)
    SetNewRawGradeValue(SimGrade);
    SetNewActuatorPosition();
    ShowSlopeTriangleOnOled();
  }
  // Check position with the settings
  if (!InControlUpDownMovementLoop) {
    ControlUpDownMovement();
  } 
} // end
.    
```
+ The callback functions are dominating completely
```C++
.
void loop()
{ // Do not use ... !!!
  // -------------------------------------------------------
  // The callback/interrupt functions are dominating completely the
  // processing and loop() is never ever (!) called,
  // since there is a constant stream of BLE packets, handled with
  // high priority interrupts that are coming in!
  // -------------------------------------------------------
 }
// END of program
.
```

# SIMCLINE Companion App<br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/App_screens.jpg" width="600" height="600" alt="Companion App"><br clear="left">
After the project was more or less accomplished and running, practical experience was gathered during many months. It became clear to me that a Companion App with some basic features would be very welcome.<br> You need some easy possibility to change settings that in the beginning were supposed to be set at compile time only. Insights change with time! Reprogramming the Arduino code on the Feather over USB becomes cumbersome when the SIMCLINE has to be dismantled every time! Therefore it was decided to develop a Companion App that would allow at minimal a feature for changing settings.<br clear="left">
After some exploring of the field (I had no experience with App development), the outcome was to build one (for Android) in the accessible environment of [MIT App Inventor 2](https://appinventor.mit.edu).<br>
+ Download the <b>MIT App Inventor</b> SIMCLINE Companion App code with extension *file*<b>.aia</b>
+ [Visit at AppInventor](https://appinventor.mit.edu), You can get started by clicking the orange "Create Apps!" button from any page on the website.
+ Get started and upload the SIMCLINE Companion App code.
+ Or upload the SIMCLINE Companion App <b>APK</b> to your Android device directly and install the APK. Android will call this a security vulnerability!

# Flow and Some Code Snippets<br>
+ The Simcline Companion app is differently functioning with this implementation of the Simcline Wahoo code... First of all and most important the "Bridge-Function" is a very heavy burden on the processing capacity of the Feather nRF52. This simply does NOT (!) allow the smartphone to be connected over BLE at the same time the Laptop/Desktop (with Zwift) is connected, as is the case when you have a TACX trainer connected!
+ After startup of the Simcline is established and it has paired with trainer and laptop (with Zwift), you have to disconnect (unpair) first the Simcline (a.k.a. Wahoo Sim) on the Zwift pairing screen. Only then it starts (BLE) advertising for the smartphone to connect! The Companion App (on your smartphone) establishes a connection over BLE with the Nordic UART service (a.k.a. BLEUART) for exchange of information. A simple dedicated protocol was implemented that allows for bidirectional exchange of short strings (<= 20 bytes) containing diagnostic messages or cyling variables.<br>
+ At first the SIMCLINE sends the latest (persistent) settings data to allow the App user to assess the current values.
+ From this moment on the App user can change the current settings or control data, the Companion App sends these to the SIMCLINE to make use of.
+ The SIMCLINE receives asynchronously settings and sets the appropriate operational variables in accordance. This determines only at a later stage the working of the equipment!
+ The settings are persistently stored for future use.
+ The SIMCLINE is <b>NOT</b> capable of sending on-the-fly cyling data (like Speed, Power, Cadence or Grade), however, other functionality like manual movement control is available.
+ After you have disconnected the Companion app, you can renew a BLE connection (pair) with the Laptop/Desktop (with Zwift) and continue or start riding... New settings will then be active!
<img src="https://github.com/Berg0162/simcline/blob/master/images/ButtonSendCache.jpg" alt="Companion App"><br clear="left">
```C++
.
void prph_bleuart_rx_callback(uint16_t conn_handle) {
  (void) conn_handle;
// Read data received over BLE Uart from Mobile Phone
  char RXpacketBuffer[20+1] = { 0 };
  bleuart.read(RXpacketBuffer, 20);
// The following routines parse and process the incoming commands
// Every RXpacket starts with a '!' otherwise corrupt/invalid
  if (RXpacketBuffer[0] != '!'){
    return; // invalid RXpacket: do not further parse and process
  }
.
.
 // New Settings values have arrived --> parse, set values and store persistently
   uint8_t iMax = 0, iMin = 0, iPerc = 0, iDispl = 0;
   sscanf(RXpacketBuffer, "!S%d;%d;%d;%d;", &iMax, &iMin, &iPerc, &iDispl);
 // set aRGVmax
   aRGVmax = map(iMax, 0, 20, 20000, 22000);
 // set aRGVmin
   aRGVmin = map(iMin, 10, 0, 19000, 20000);
 // set PercIncreaseInclination
   PercentageIncreaseValue = iPerc;
 // set OledDisplaySelection
   OledDisplaySelection = iDispl;
 // LittleFS for persistent storage of these values
 setPRSdata();
.
.
```

# Simcline in TTS4 controlled operation<br>
In the following images is shown how the Simcline is operating when different inclination values are applied by the training software. In the present situation the Simcline is controlled by the TACX Training System (TTS4 is no longer supported by Tacx). The software is still running on my laptop and is operating in Workout Mode: <b>Slope-Time</b>. After setting the slope (in workout mode by the user) the Tacx trainer is instructed to set resistance exactly with that inclination. The Simcline polls regularly for this value with the Tacx trainer and sets the height of the front wheel axle in accordance. Notice the position and inclination with respect to the measuring tape. The measuring tape starts (zero centimeter position) at the lowest possible position of -10%. 10 Centimeter height is flat road level (0% inclination)....<br> Notice that TTS4 in this mode is <b>WhatYouSeeIsWhatYouGet</b>, the inclination that is shown on the app screen is exactly sent to the trainer for resistance setting!<br>
Please notice that today many training apps are concerned more about "<i>the optimal user experience</i>" rather than with <b>WYSIWYG</b>, with respect to the road inclination. Most apps manipulate in some way the inclination that is sent to the trainer. When it is made clear <b>HOW</b> that inclination is manipulated (like the Difficulty Setting with Zwift) one can choose to allow for that <i>optimal user experience</i> or undo the setting and go for real road feel!<br>
So when the Simcline seems not to follow the road inclination values on the training app screen, be aware that the training app is <b>NOT WYSIWYG</b>!
<br>

<img src="https://github.com/Berg0162/simcline/blob/master/images/Simcline_in_operation.jpg" width="1000" height="600" alt="Simcline at work"><br clear="left">
<br>


# [Code support for smart TACX trainers](https://github.com/Berg0162/simcline/edit/master/Tacx%20Smart/)<br>

# Mechanical Construction of SIMCLINE<br>
There are elaborated <b>Instructables</b> available with all the nitty gritty of how to create, construct and install the various parts and components of the SIMCLINE.<br> 

# SIMCLINE 1.0<br>
See: <img src="https://www.instructables.com/assets/img/instructables-logo-v2.png" width="32" height="48" align="left" alt="Instructables"> [SIMCLINE 1.0 Instructables](https://www.instructables.com/id/SIMCLINE-Simulation-of-Changing-Road-Incline-for-I/)<br clear="left">

# SIMCLINE 2.0<br>
See: <img src="https://www.instructables.com/assets/img/instructables-logo-v2.png" width="32" height="48" align="left" alt="Instructables"> [SIMCLINE 2.0 Instructables](https://www.instructables.com/SIMCLINE-20-Easy-Simulation-of-Road-Incline/)<br clear="left">



