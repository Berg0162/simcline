# <img src="https://github.com/Berg0162/simcline/blob/master/images/SC_logo.png" width="64" height="64" alt="SIMCLINE Icon"> &nbsp; SIMCLINE for TACX
# Simulation of Changing Road Inclination for Indoor Cycling <br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/Simcline_And_Wheel.jpg" width="300" height="300" ALIGN="left" alt="SIMCLINE">
<img src="https://github.com/Berg0162/simcline/blob/master/images/Simcline_2_0.jpg" width="300" height="300" ALIGN="left" alt="SIMCLINE 2">
The SIMCLINE physically adjusts the bike position to mimic hilly roads, climbing and descending. This allows the rider to naturally change position on the bike, engage climbing muscles, and improve pedaling technique to become a more efficient and powerful climber.<br>
Without user intervention the SIMCLINE will replicate inclines and declines depicted in (online & offline) training programs (like Zwift, Rouvy, VeloReality and many others) that adjust accordingly the resistance of the indoor trainer.<br>
The SIMCLINE auto connects at power up with a smart TACX trainer and let's relive the ascents and descents from favorite rides or routes while training indoors.<br>
The physical reach is: 20% maximum incline and -10% maximum decline. However, the reach that the rider is comfortable with can be adjusted even during a ride!<br>
The SIMCLINE pairs directly to the TACX smart trainer for a connection that notifies the SIMCLINE to simulate autonomous the (change in) physical grade of the road during an indoor ride.<br>
During operation an OLED display shows the road grade in digits and in graphics or cycling data from the trainer (speed, power, cadence, elapsed time and distance).<br>
The SIMCLINE Companion App (for Android smartphones) can be paired simultaneously for adjusting operational settings, like Ascent Grade Limit (between 0-20%) and Descent Grade Limit (between 0-10%), Road Grade Change Factor (between 0-100%) and the type of OLED display format.<br clear="left"> 

# Choose a Development board: nRF52840 or ESP32?<br>
Until early 2023 the Simcline project (a.o.) solely worked with the <b>Feather nRF52840 Express</b> development board and the Bluefruit/Adafruit libraries. This is a very stable platform and gave reliable results during development and what's more during many indoor seasons of 5 hours per week operation! However, the MITM application and actuator control (a.k.a. Simcline) is at the capacity limits of the nRF52840 processor. 
See for a reliable and <b>proven</b> solution: [Adafruit Feather nRF52840 Express](https://learn.adafruit.com/introducing-the-adafruit-nrf52840-feather) <br>

So the question was raised why not use the <b>ESP32</b>, a series of low-cost and low-power System on a Chip (SoC) microcontrollers developed by Espressif that include Wi-Fi and Bluetooth wireless capabilities and dual-core processor? See for an introduction: [Random Nerds Tutorials](https://randomnerdtutorials.com/getting-started-with-esp32/). Particularly the multiprocessing capabilities of the dual-core processor make the ESP32 a very tempting option in this project!
To benefit of the same formfactor (fit with the Simcline 2.0 component box!), I decided to (re)produce the Simcline code with the [Adafruit Feather ESP32 V2](https://learn.adafruit.com/adafruit-esp32-feather-v2) for the <b>ESP32</b> platform. Just comparing the overal specs (on paper) of both processor platforms is not sufficient... one needs to take into account the (quality of the) available libraries as well to reach success. The 'standard' ESP32 Arduino <b>Bluedroid</b> library (for BLE support) turned out to be buggy! Fortunately [H2Zero](https://github.com/h2zero/NimBLE-Arduino) has created a more or less compatible replacement for Bluedroid, called <b>NimBLE-Arduino</b>! The application of <b>NimBLE</b> saved the porting of the software to the ESP32 platform! 
In week 10 of 2023 the ESP32 code reached a stable level and became publicly available for testing with Tacx-trainers.

# How to start?<br>
+ Install the [Arduino IDE](https://www.arduino.cc/en/Main/Software) and all the libraries on a PC/Mac.
+ If you are using an ESP32 board then also download the ESP32 NimBLE library, see [Arduino Installation NimBLE](https://github.com/h2zero/NimBLE-Arduino#arduino-installation)
+ Download all the code from [Github](https://github.com/Berg0162/simcline/tree/master/Tacx%20Smart) and install.
<br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/Simcline_circuitry_02.jpg" align="left" width="200" height="200" alt="Cardboard">
When I started the project in 2020 I did not have any practical experience with any of the components. So I had to setup the circuitry step by step adding components and did a lot of time consuming but instructive testing first. My advice is to setup the electronic components first in a similar way as shown on the photo with the cardboard base. Use double sided adhesive tape but only attach it on sections that have no pcb-wiring or soldering, to avoid possible electrical interference. You will find in this Github repository all the code that controls the Simcline and the Arduino test programs (modified for this project) that focus on components seperately and in conjunction.<br clear="left">

# ANT+, FE-C protocol and BLE<br>
It took me a lot of time exploring the relevant techniques, protocols and software tools involved with ANT+, BLE and FE-C to acquire the approriate knowledge. Many projects at Github address ANT+ and/or BLE with a different point of departure, however I have learnt a lot of looking closely at the many program codes, explanations and descriptions. Finally, I tracked down how a smart up/down lift of my front wheel axle is optimally benefitting from the (ANT+) FE-C protocol when I am indoor riding with the TACX Neo in the hilly Zwift game world or with any other virtual cycling program that simulates road grade. After a succesfull proof of concept with ANT+ and BLE tools (see [ThisisANT Tools](https://www.thisisant.com/developer/resources/software-tools/) and [Nordic NRF Connect](https://www.nordicsemi.com/Software-and-tools/Development-Tools/nRF-Connect-for-mobile). I was convinced to bring the project to a successful conclusion!
<img src="https://github.com/Berg0162/simcline/blob/master/images/Devices_Interacting_FEC_ANT.jpg" width="800" height="300" alt="FE-C"><br clear="left">
<b>ANT+ Trainer Control (FE-C)</b><br>
The ANT+ FE-C protocol details the bi-directional message communication of trainer data (speed, power, etcetera) from trainer to the controller and from the controller to the trainer commands or settings of targeted power, trainer resistance (grade) and calibration controls. See [ThisisAnt](http://www.thisisant.com/) for all conceivable documentation, tools, implementations, ANT+ based products, etcetera! See at Github for a detailed description of the [ANT Message Protocol and Usage](https://github.com/Berg0162/simcline/blob/master/docs/ANT_Message_Protocol_and_Usage_Rev_5.1.pdf).
The majority of FE-C trainers on the market is dual ANT+ & Bluetooth Smart in some capacity. The Tacx Neo generation has full capacity: ANT+ Trainer Control (FE-C) and Bluetooth Smart.<br>
Tacx published in 2015 a document [TACX, FE-C and Bluetooth](https://github.com/Berg0162/simcline/blob/master/docs/How_to_FE_C_over_BLE_v1_0_0.pdf) that explains how to use the FE-C ANT+ protocol over BLE feature implemented on all Tacx Smart Trainers then: `Genius Smart, Bushido Smart, Vortex Smart, Flow Smart, Satori Smart and NEO`. Tacx designed this feature because at that time an open standard (on BLE) for trainers was lacking. <b>Always check if your trainer has support for the FE-C ANT+ protocol over BLE feature, since NOT all TACX Smart Trainers that are labelled smart are equally smart!</b>
Now there is (<b>FTMS</b>) FiTness Machine Service protocol to control fitness equipment over Bluetooth. According to the smart trainer recommendations guide winter 2019-2020 of [DCRainmaker](https://www.dcrainmaker.com/2019/10/the-smart-trainer-recommendations-guide-winter-2019-2020.html) the situation evolved:
> Meanwhile, for Bluetooth Smart, there’s FTMS, which is basically the same thing as FE-C when it comes to trainers. It’s not quite as widely adopted yet by trainer companies, but is by app companies. On the trainer company side only Elite, Saris, and Kinetic support it across the board. With Tacx having it on some but not all units, and Wahoo having it on no units (but all Wahoo and Tacx trainers support private Bluetooth Smart with all major apps anyway).<br>

<b>FE-C over Bluetooth</b><br>
In the document Tacx described how Tacx Smart trainers use an ANT+ FE-C definition for the data content, but transports this data over a BLE serial port service. The prerequisite that remains for this technique is thourough knowledge of ANT+ <b>AND</b> the ANT+ FE-C protocol. A complete description of FE-C can be found here at [Github](https://github.com/Berg0162/simcline/blob/master/docs/ANT+_Device_Profile_Fitness_Equipment_Rev_5.0.pdf).<br>
<b>Nordic Serial Port Service</b><br>
The transport method that is used is derived from the Nordic Uart Service. This serial port service can basically send and receive arrays of data ranging from 1..20 bytes. The only difference is the service UUID and the two characteristics UUID’s. These are renamed conforming the internal Tacx standard UUID with the specific part for the FE-C service starting with 0xFEC.<br>
<b>ANT+ and BLE Roles</b><br>
The Adafruit Feather nRF52 Express is equiped with a native-Bluetooth chip, the nRF52840 of Nordic Semiconductor. This chip can be loaded with "SoftDevices" for different applications. Nordic nRF52 and nRF51 Series support ANT+ and ANT+ / Bluetooth LE SoftDevices. However, the adafruit version is loaded with SoftDevice S132: a high-performance Bluetooth Low Energy protocol stack for the nRF52840 System-on-Chips (SoCs). It supports up to 20 concurrent links in all roles (Broadcaster, <b>Central</b>, Observer, <b>Peripheral</b>). It is Bluetooth 5.1 qualified and supports the following Bluetooth features: high-throughput 2 Mbps, Advertising Extensions and channel selection algorithms. <br>
> Recently we have shared Simcline code that is targeted for another of the Adafruit star Feathers: the <b>Adafruit HUZZAH32 ESP32 Feather V2</b> - with the fabulous ESP32 WROOM module. The ESP32 has both WiFi and Bluetooth Classic/LE support. The new HUZZAH32 V2 is Adafruit's redesigned ESP32-based Feather V2. Compared to the original Feather with only 4 MB Flash and no PSRAM, the V2 has 8 MB Flash and 2 MB PSRAM. Packed with everything people love about Feathers: built in USB-to-Serial converter, automatic bootloader reset, Lithium Ion/Polymer charger, and just about all of the GPIOs brought out.<br>

In our design the SIMCLINE has a <b>Central</b> and as well as a <b>Peripheral</b> role simultaneously. The communication SIMCLINE to Trainer is Central to Peripheral respectively and the communication SIMCLINE to Smartphone is Peripheral to Central. The peripheral BLE Device is advertising its name, services and characteristics and the central device is deciding to connect and control the communication. <br>
The communication between the trainer (<b>controllable</b>) and the PC/MAC/Tablet-application (<b>controller</b>, like Zwift) is full blown ANT+ and complies the FE-C protocol. The SIMCLINE is not interfering with this setup in any way! The application on PC/MAC/Tablet is <b>controller</b> and remains in full and only charge of setting the trainers resistance.<br>
<b>How to detect the grade of the simulated track?</b><br>
The SIMCLINE is paired with the trainer over a different channel: Bluetooth! In that configuration it is complying to the ANT+ FE-C protocol as well but over Bluetooth LE. The trainer is not only broadcasting FE-C messages with cycling data (speed, cadence, power, etcetera) over ANT+ to the <b>controller</b>-application (like Zwift), but also over the BLE connection to the paired Feather nRF52/ESP32. The program of the Feather nRF52/ESP32 is dealing with these data in its own way, independent of the <b>ANT+ controller</b>-application.
At regular intervals the Feather nRF52/ESP32 is programmed to send a socalled Common Page 70 (0x46) (Request Data Page) with the request data page field set to Data Page <b>51</b>.
```C++
.
//Define the FE-C ANT+ Common Page 70 with Request for Page #51
const unsigned char Page51Bytes[13] = {
    0xA4, //Sync
    0x09, //Length
    0x4F, //Acknowledge message type
    0x05, //Channel 
          //Data
    0x46, //Common Page 70
    0xFF,
    0xFF,
    0xFF, //Descriptor byte 1 (0xFF for no value)
    0xFF, //Descriptor byte 2 (0xFF for no value)
    0x80, //Requested transmission response
    0x33, //Requested Page number 51 
    0x01, //Command type (0x01 for request data page, 0x02 for request ANT-FS session)
    0x47}; //Checksum;
.
```
Sending Common Page 70 allows a connected device to request specific data pages from the trainer. The trainer replies with Common Page 71 (0x47) (Command Status) to the requester, the Feather nRF52/ESP32. The purpose of the command status page is to confirm the status of commands (and settings) sent from a controller to the controllable trainer. The last <b>settings</b> of the Data Page 51 (0x33) (Track Resistance) are included in the data of Common Page 71. The Track Resistance Page itself is sent by the controller (like Zwift) to command the trainer to use simulation mode, and to set the desired track resistance factors. It provides the simulation parameters for the trainer, the rolling resistance and gravitational resistance applied to the rider. <br>
```C++
.
#endif
  // Standard FE-C Data Message Format
  // buffer[0]  ->  Sync
  // buffer[1]  ->  Msg Length
  // buffer[2]  ->  Msg ID
  // buffer[3]  ->  Channel Number
  // buffer[4]-buffer[12] Payload of 8 bytes --> buffer[4] is byte(0) of Payload bytes 
  // buffer[13] ->  Checksum
  // --------------------------------
  uint8_t DataPageNumber = buffer[4]; // Get Data Page Number from ANT FE-C packet
  // process only the data pages we are interested in, ignore others !
  switch(DataPageNumber) {
    ///////////////////////////////////////////////////////////////
    //////////////////// Handle PAGE 71 ///////////////////////////
    ////////////// Requested PAGE 51 for grade info ///////////////
    //At a regular rate Page 51 is requested for, so process here//
    ///////////////////////////////////////////////////////////////
    case 0x47 :
    // buffer[4] -> Contains 71 (0x47) -> Common Page 71 -> Command status
    // buffer[5] -> Last Received Command ID
    // buffer[6] -> Sequence #
    // buffer[7] -> Command Status
    // buffer[8]-buffer[12] -> Response data 5 bytes
    // buffer[13] -> Checksum
    if ( buffer[5] == 0x33 ) { // check for Requested Page 51
      // We are interested in the Requested Page 51 (0x33) --> Track Resistance
      // in that case the packet contains:
      // buffer[5] -> Last Received Command ID --> Data Page Number 51 (0x33)
      // buffer[6] -> Reserved and set to 0xFF
      // buffer[7] -> Reserved and set to 0xFF
      // buffer[8] -> Reserved and set to 0xFF
      // buffer[9]-buffer[12] -> Response data 4 bytes
      // buffer[13] -> Checksum
      uint8_t lsb_GradeValue = buffer[9]; // Grade (Slope) LSB
      uint8_t msb_GradeValue = buffer[10];// Grade (Slope) MSB
      RawgradeValue = lsb_GradeValue + msb_GradeValue*256;
      // buffer[11] -> Coefficient of Rolling Resistance
.
```
By sending regularly a request for Data Page 51 (0x33) (Track Resistance) the SIMCLINE is always informed about the settings of the current grade of the simulated track and the coefficient of rolling resistance. These values are both set by the <b>ANT+ controller</b>. For proper functioning of the SIMCLINE only the current road grade is critical.</br>

# Dual Processor use with ESP32
One of the advantages of the ESP32 platform is the fact that the ESP32 WROOM processor has two cores. This makes it possible to precisely balance the load of a program over 2 processor cores. <br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/Arduino_IDE_2_Tools_Menu.jpg" align="left" width="440" height="310" alt="Arduino IDE 2.0 Tools Menu">
With the Simcline this is particular usefull for the motor control of the actuator. During operation Zwift sends from time to time new settings, and one of these is the grade value (road inclination in degrees). The program translates the grade to a level that should be reached by the actuator to simulate exactly the road grade that was received from Zwift. However, the actuator can only be switched to <b>move up</b>, <b>move down</b> or <b>stop</b>. After  having set the actuator to move (up or down), the program has to check continuously if the actuator has reached the desired level by reading its position with the help of the Time-Of-Flight sensor and act accordingly. Meanwhile the trainer sends your cycling data and the Zwift app has to confirm the receipt of these data. The data sent by Zwift has also to be tranferred to the trainer and also the trainer has to confirm the receipt. Being a MITM means handling a lot of BLE traffic and it does not allow for mistakes!
The load of the Simcline program itself, the BLE handling and the critical control of the actuator is balanced over 2 processor cores on the ESP32 platform.
The following code snippets show how this is achieved for controlling the actuator motor. To avoid conflicts during variable updates (i.c. TargetPosition) a Binary Semaphore scheme is applied to protect <b>task shared variables</b> during an update.<br clear="left">

At the start the major players are defined
```C++
.
// ----------------------xControlUpDownMovement task definitions ------------------
SemaphoreHandle_t xSemaphore = NULL;
TaskHandle_t ControlTaskHandle = NULL;
// Set Arduino IDE Tools Menu --> Events Run On: "Core 1"
// Set Arduino IDE Tools Menu --> Arduino Runs On: "Core 1"
// Run xControlUpDownMovement on "Core 0"
const BaseType_t xControlCoreID = 0;
void xControlUpDownMovement(void* arg); 
// --------------------------------------------------------------------------------
.
```
In the setup() routine the variables are instantiated (after checking the mechanics of the motor function) and the <b>xControlUpDownMovement</b> task is pinned to processor <b>core 0</b>, with a priority of 10. Most of the Simcline program and Events are running on <b>core 1</b>.
```C++
.
  } else {
    ShowOnOledLarge("Testing", "Functions", "Done!", 500);
    // Is working properly --> Start Motor Control Task
    xSemaphore = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(xControlUpDownMovement, "xControlUpDownMovement", 4096, NULL, 10, &ControlTaskHandle, xControlCoreID);
    xSemaphoreGive(xSemaphore);
    DEBUG_PRINTLN("Motor Control Task Created and Active!");        
    IsBasicMotorFunctions = true;
    DEBUG_PRINTLN("Simcline Basic Motor Funtions are working!!");
    // Put Simcline in neutral: flat road position
#ifdef EMA_ALPHA
    // Init EMA filter at first call with flat road position as reference
    TargetPosition = EMA_TargetPositionFilter(TargetPosition); 
#endif
    SetNewActuatorPosition();
  }
.
```
Whenever new values for the road grade are received these are translated to a physical actuator position (level above ground) and the <b>TargetPosition</b> is set during Semaphore protection. When the new position has been set, the protection is cancelled, and the motor control task can access the new setting.
```C++
.
void SetNewActuatorPosition(void) {
  // Handle mechanical movement i.e. wheel position in accordance with Road Inclination
  // Map RawgradeValue ranging from 0 to 40.000 on the
  // TargetPosition (between MINPOSITION and MAXPOSITION) of the Lifter
  // Notice 22000 is equivalent to +20% incline and 19000 to -10% incline
  RawgradeValue = constrain(RawgradeValue, RGVMIN, RGVMAX); // Keep values within the safe range
  TargetPosition = map(RawgradeValue, RGVMIN, RGVMAX, MAXPOSITION, MINPOSITION);
  // EMA filter for smoothing quickly fluctuating Target Position values see: Zwift Titan Grove
#ifdef EMA_ALPHA
  TargetPosition = EMA_TargetPositionFilter(TargetPosition);
#endif
  if(IsBasicMotorFunctions) {  
    xSemaphoreTake(xSemaphore, portMAX_DELAY); 
    lift.SetTargetPosition(TargetPosition);
    xSemaphoreGive(xSemaphore);
#ifdef MOVEMENTDEBUG
    DEBUG_PRINTF("RawgradeValue: %05d Grade percent: %03.1f%% ", RawgradeValue, gradePercentValue);
    DEBUG_PRINTF("TargetPosition: %03d\n", TargetPosition, DEC);
#endif
  }  
}
.
```
The motor control task regularly checks how far off the actuator position is from its target position and if it should be braked yet. However, it happens all the time that the road grade changed from upward to flat or to downward. The actuator should follow these changes and therefore the motor is switched many times to brake or to reverse its movement. When the motor control task is accessing the relevant variables the semaphore is protecting these against updates!
```C++
void xControlUpDownMovement(void *arg) {
  // Check "continuously" the Actuator Position and move Motor Up/Down until target position is reached
  int OnOffsetAction = 0;
  const TickType_t xDelay = 110 / portTICK_PERIOD_MS; // Block for 110ms < 10Hz sample rate of VL6180X
  while(1) {
    if(xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
        // BLE channels can interrupt and consequently target position changes on-the-fly !!
        // We do not want changes in TargetPosition during one of the following actions!!!
        OnOffsetAction = lift.GetOffsetPosition(); // calculate offset to target and determine action
        switch (OnOffsetAction)
            {
              case 0 :
                lift.brakeActuator();
                #ifdef MOVEMENTDEBUG
                DEBUG_PRINTLN(F(" -> Brake"));
                #endif
                break;
              case 1 :
                lift.moveActuatorUp();
                #ifdef MOVEMENTDEBUG
                DEBUG_PRINTLN(F(" -> Upward"));
                #endif
                break;
              case 2 :
                lift.moveActuatorDown();
                #ifdef MOVEMENTDEBUG
                DEBUG_PRINTLN(F(" -> Downward"));
                #endif
                break;
              case 3 :
                // Timeout --> OffsetPosition is undetermined --> do nothing and brake
                lift.brakeActuator();
                #ifdef MOVEMENTDEBUG
                DEBUG_PRINTLN(F(" -> Timeout"));
                #endif
                break;
            } // switch 
        xSemaphoreGive(xSemaphore);    
    }      
    vTaskDelay(xDelay);
  } // while
} // end
```

# Overview of Arduino Feather nRF52840 Express Program Code Flow and Snippets<br>
+ Include headers of libraries and declare classes
```C++
.
// Library Adafruit Feather nRF52 Bluefruit (Bluetooth Low Energy)
#include <bluefruit.h>
// Libraries for use of I2C devices (Oled and VL6180X distance sensor)
#include <SPI.h>
#include <Wire.h>
// Necessary libraries for use of Oled display(s)
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// Additional splash screen bitmap and icon(s) for Oled
#include "Adafruit_SSD1306_Icons.h" // needs to be in directory of main code
// Declare the SSD1306 Class
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#include <avr/dtostrf.h>
// LittleFS for internal storage of persistent data on the Feather nRF52
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;
// Managing persistence of some data with LittleFile system
// PeRSistent Data  --> PRS data
#define PRS_FILENAME "/prsdata.txt"
// Declare the File Class
File file(InternalFS);
// LittleFS--------------------------------------------------------------
// Library code for low level measuring (VL6180X) and controlling UP and down movement
#include <Lifter.h>
.
```
+ Define variables, set to default values and initialize classes.
```C++
.
// Declare in Reversed order !!!
uint8_t TACX_FEC_PRIMARY_SERVICE_Uuid[16]=     {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0xC1, 0xFE, 0x40, 0x6E,};
uint8_t TACX_FEC_READ_CHARACTERISTIC_Uuid[16]= {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0xC2, 0xFE, 0x40, 0x6E,};
uint8_t TACX_FEC_WRITE_CHARACTERISTIC_Uuid[16]={0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0xC3, 0xFE, 0x40, 0x6E,};
.
```
```C++
.
//Declare crucial services and charateristics for TACX FE-C trainer
BLEClientService        fecps(TACX_FEC_PRIMARY_SERVICE_Uuid);
BLEClientCharacteristic fecrd(TACX_FEC_READ_CHARACTERISTIC_Uuid);
BLEClientCharacteristic fecwr(TACX_FEC_WRITE_CHARACTERISTIC_Uuid);
.
```
```C++
// Declaration of Function prototypes
bool getPRSdata(void);
void setPRSdata(void);
void prph_connect_callback(uint16_t conn_handle);
void prph_disconnect_callback(uint16_t conn_handle, uint8_t reason);
void prph_bleuart_rx_callback(uint16_t conn_handle);
void prph_bleuart_TX_Grade(void);
void prph_bleuart_TX_PWR_CAD(void);
void prph_bleuart_TX_ADT_SPD_AET(void);
void scan_stop_callback(void);
void adv_stop_callback(void);
void scan_callback(ble_gap_evt_adv_report_t* report);
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void SendRequestPage51(void);
void fecrd_notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len);
void SetNeutralValues(void);
bool ControlUpDownMovement(void);
void ShowOnOledLarge(char *Line1, char *Line2, char *Line3, uint16_t Pause);
void BuildBasicOledScreen(void);
void ShowValuesOnOled(void);
void ShowSlopeTriangleOnOled(void);
```
<b>Begin of the Arduino Setup() Function</b><br>
+ Get or set (first time only) the values of relevant and crucial variables to persistence, whith the Companion App the user can set these on the fly!
+ Start the show for the SSD1306 Oled display.
+ Initialize Lifter Class data, variables, test and set to work!
```C++
.
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
    }
else {
    ShowOnOledLarge("Testing", "Functions", "Succes!", 500);
    // Is working properly
    IsBasicMotorFunctions = true;
    // Put Simcline in neutral: flat road position
    SetNeutralValues(); // set relevant flat road values
    while (ControlUpDownMovement()) { // wait until flat road position is reached
    }
    }
.    
```
+ Initialize Bluefruit with maximum connections as Peripheral = 1, Central = 1.
```C++
.
  Bluefruit.begin(1, 1);
  Bluefruit.setTxPower(4); // Check bluefruit.h for supported values
  Bluefruit.setName("Bluefruit-nRF52");
.
```
+ Setup Central Scanning for an advertising TACX trainer...
```C++
.
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.filterRssi(-70);      // original value of -80 , we want to scan only nearby peripherals, so get close to your TACX trainer !!
  Bluefruit.Scanner.setInterval(160, 80); // in units of 0.625 ms
// We are only interested in the services of the TACX Trainer
  Bluefruit.Scanner.filterUuid(TACX_FEC_PRIMARY_SERVICE_Uuid);
  Bluefruit.Scanner.useActiveScan(true);
.    
```
+ Initialize TACX FE-C trainer services and characteristics.
+ Declare Callbacks for Peripheral (smartphone connection) and Callbacks for Central (trainer connection).
```C++
.
// Declare Callbacks for Peripheral (smartphone connection)
  Bluefruit.Periph.setConnectCallback(prph_connect_callback);
  Bluefruit.Periph.setDisconnectCallback(prph_disconnect_callback);

// Callbacks for Central (trainer connection)
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.setStopCallback(scan_stop_callback);

// set up callback for receiving ANT+ FE-C packets; this is the main work horse!
  fecrd.setNotifyCallback(fecrd_notify_callback);
.    
```
+ Initialize some characteristics of the Device Information Service.
+ All initialized --> Start the actual scanning!
```C++
.
// Show Scanning message on the Oled
  ShowOnOledLarge("Scanning", "for", "Trainer", 500);
  Bluefruit.Scanner.start(300); // 0 = Don't stop scanning or after n, in units of hundredth of a second (n/100)
  while (Bluefruit.Scanner.isRunning()) { // do nothing else but scanning....
  }
.    
```
+ Initialize and setup BLE Uart functionality for connecting to smartphone --> Start the advertising!
```C++
   bleuart.begin();
   bleuart.setRxCallback(prph_bleuart_rx_callback);
// Advertising packet construction
   Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
   Bluefruit.Advertising.addTxPower();
// Include the BLE UART (AKA 'NUS') 128-bit UUID
   Bluefruit.Advertising.addService(bleuart);
   Bluefruit.Advertising.setStopCallback(adv_stop_callback);
// Start advertising: to be picked up by a Smartphone with the Companion App!
   Bluefruit.Advertising.start(60); // 0 = Don't stop advertising or after n (!) seconds -> 1 minuut
.    
```
<b>End of the Arduino Setup() Function</b><br>

+ The callback functions are dominating completely the processing and <b>loop()</b> would never have been called, since there is a constant stream of FE-C packets that are coming in!
+ The function <b>fecrd_notify_callback</b> does the bulk of the work!
```C++
.
    void loop()
    { // Do not use ... !!!
    }
.
```
+ <b>fecrd_notify_callback</b> is a callback that is triggered when a ANT+ message is sent from TACX Trainer.
```C++
.
  void fecrd_notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len) {
// The FE-C Read charateristic of ANT+ packets
// In TACX context receive or send arrays of data ranging from 1--20 bytes so FE-C
// will not exceed the 20 byte maximum...
// Data pages are broadcast (by the trainer) at 4Hz message rate
  uint8_t buffer[20 + 1];
  memset(buffer, 0, sizeof(buffer)); // fill with zero
// Transfer first the contents of data to buffer (array of chars)
  for (int i = 0; i < len; i++) {
    if ( i <= sizeof(buffer)) {
      buffer[i] = *data++;
    }
   }
.   
```
+ All ANT+ FE-C message pages are handled, parsed and relevant variables set to new values.
```C++
// Show the actual values of the trainer on the Oled
  if (OledDisplaySelection == 1) {
    ShowValuesOnOled();
  } else {
    ShowSlopeTriangleOnOled();
  }
// Check and control motor up/down movement within settings!
  if (IsBasicMotorFunctions) {
    while (ControlUpDownMovement()) {
    }
  }
.  
```
+ Send a request for Page 51 about every 4 seconds.

# SIMCLINE Companion App<br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/App_screens.jpg" width="600" height="600" alt="Companion App"><br clear="left">
After the project was more or less accomplished and running, practical experience was gathered during many months. It became clear to me that a Companion App with some basic features would be very welcome.<br> You need some easy possibility to change settings that in the beginning were supposed to be set at compile time only. Insights change with time! Reprogramming the Arduino code on the Feather over USB becomes cumbersome when the SIMCLINE has to be dismantled every time! Therefore it was decided to develop a Companion App that would allow at minimal a feature for changing settings.<br clear="left">
After some exploring of the field (I had no experience with App development), the outcome was to build one (for Android) in the accessible environment of [MIT App Inventor 2](https://appinventor.mit.edu).<br>
+ Download the <b>MIT App Inventor</b> SIMCLINE Companion App code with extension *file*<b>.aia</b>
+ [Visit at AppInventor](https://appinventor.mit.edu), You can get started by clicking the orange "Create Apps!" button from any page on the website.
+ Get started and upload the SIMCLINE Companion App code.
+ Or upload the SIMCLINE Companion App <b>APK</b> to your Android device directly and install the APK. Android will call this a security vulnerability!

# Flow and Some Code Snippets<br>
+ At startup SIMCLINE starts (BLE) advertising, independent of whether a trainer connection is realized before or not! The Companion App establishes a connection over BLE and the Nordic UART service (a.k.a. BLEUART) for exchange of information is applied. A simple dedicated protocol was implemented that allows for bidirectional exchange of short strings (<= 20 bytes) containing diagnostic messages or cyling variables.<br>
+ At first the SIMCLINE sends the latest (persistent) settings data to allow the App user to assess the current values.
+ The SIMCLINE sends regularly cyling data (like Speed, Power, Cadence, Grade etcetera) that were received from the trainer (in ANT+ FE-C packets) and processed.
+ At any time the App user changes the current settings or control data, the Companion App sends these to the SIMCLINE to make use of.
<img src="https://github.com/Berg0162/simcline/blob/master/images/ButtonSendCache.jpg" alt="Companion App"><br clear="left">
+ The SIMCLINE receives asynchronously settings and sets the appropriate operational variables in accordance. This determines instantly the working of the equipment.
+ The settings are persistently stored for future use.
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
+ In addition to the OLED display the Companion App can serve as an enhanced screen for road grade and cycling data.
+ Until the BLE connection is disconnected, manually or by quitting the App, both devices remain connected.

# Simcline in TTS4 controlled operation<br>
In the following images is shown how the Simcline is operating when different inclination values are applied by the training software. In the present situation the Simcline is controlled by the TACX Training System (TTS4 is no longer supported by Tacx). The software is still running on my laptop and is operating in Workout Mode: <b>Slope-Time</b>. After setting the slope (in workout mode by the user) the Tacx trainer is instructed to set resistance exactly with that inclination. The Simcline polls regularly for this value with the Tacx trainer and sets the height of the front wheel axle in accordance. Notice the position and inclination with respect to the measuring tape. The measuring tape starts (zero centimeter position) at the lowest possible position of -10%. 10 Centimeter height is flat road level (0% inclination)....<br> Notice that TTS4 in this mode is <b>WhatYouSeeIsWhatYouGet</b>, the inclination that is shown on the app screen is exactly sent to the trainer for resistance setting!<br>
Please notice that today many training apps are concerned more about "<i>the optimal user experience</i>" rather than with <b>WYSIWYG</b>, with respect to the road inclination. Most apps manipulate in some way the inclination that is sent to the trainer. When it is made clear <b>HOW</b> that inclination is manipulated (like the Difficulty Setting with Zwift) one can choose to allow for that <i>optimal user experience</i> or undo the setting and go for real road feel!<br>
So when the Simcline seems not to follow the road inclination values on the training app screen, be aware that the training app is <b>NOT WYSIWYG</b>!
<br>

<img src="https://github.com/Berg0162/simcline/blob/master/images/Simcline_in_operation.jpg" width="1000" height="600" alt="Simcline at work"><br clear="left">
<br>

# [Code support for Wahoo KICKR trainers](https://github.com/Berg0162/simcline/tree/master/Wahoo%20Kickr/)<br>

# [Code support for FTMS-enabled trainers](https://github.com/Berg0162/simcline/tree/master/FTMS%20Enabled)<br>

# Mechanical Construction of SIMCLINE<br>
There are elaborated <b>Instructables</b> available with all the nitty gritty of how to create, construct and install the various parts and components of the SIMCLINE.<br> 

# SIMCLINE 1.0<br>
See: <img src="https://www.instructables.com/assets/img/instructables-logo-v2.png" width="32" height="48" align="left" alt="Instructables"> [SIMCLINE 1.0 Instructables](https://www.instructables.com/id/SIMCLINE-Simulation-of-Changing-Road-Incline-for-I/)<br clear="left">

# SIMCLINE 2.0<br>
See: <img src="https://www.instructables.com/assets/img/instructables-logo-v2.png" width="32" height="48" align="left" alt="Instructables"> [SIMCLINE 2.0 Instructables](https://www.instructables.com/SIMCLINE-20-Easy-Simulation-of-Road-Incline/)<br clear="left">


