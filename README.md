# SIMCLINE 

<img src="https://github.com/Berg0162/simcline/blob/master/images/SC_logo.png" width="64" height="64" alt="SIMCLINE Icon"> <b>Simulation of Changing Road Inclination for Indoor Cycling</b></br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/Simcline_And_Wheel.jpg" ALIGN="left" width="400" height="500" alt="SIMCLINE"></br>
The SIMCLINE physically adjusts the bike position to mimic hilly roads, climbing and descending. This allows the rider to naturally change position on the bike, engage climbing muscles, and improve pedaling technique to become a more efficient and powerful climber.</br>
Without user intervention the SIMCLINE will replicate inclines and declines depicted in (online & offline) training programs (like Zwift, Rouvy, VeloReality and many others) that adjust accordingly the resistance of the indoor trainer.</br>
The SIMCLINE auto connects at power up with a smart TACX trainer and let's relive the ascents and descents from favorite rides or routes while training indoors.</br>
The physical reach is: 20% maximum incline and -10% maximum decline. However, the reach that the rider is comfortable with can be adjusted even during a ride!</br>
The SIMCLINE pairs directly to the TACX smart trainer for a connection that notifies the SIMCLINE to simulate autonomous the (change in) physical grade of the road during an indoor ride.</br>
During operation an OLED display shows the road grade in digits and in graphics or cycling data from the trainer (speed, power, cadence, elapsed time and distance).</br>
The SIMCLINE Companion App (for Android smartphones) can be paired simultaneously for adjusting operational settings, like Ascent Grade Limit (between 0-20%) and Descent Grade Limit (between 0-10%), Road Grade Change Factor (between 0-100%) and the type of OLED display format.<br clear="left"/>

# Electronic Components and Circuitry</br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/Simcline_circuitry.jpg" width="1100" height="500" alt="Circuitry">
This project could have been elaborated with many different electronic parts that would lead to more or less the same succesfull end product. I have choosen for the following 4 active components: </br>
<b>Cytron Motor Driver MDD3A.</b> Two channel motor driver for 12 V and 3 Amperes with buttons to test manually the working of the attached DC motor. This board enables the processor to set the Actuator motor in up or down movement. It transforms logical digital levels (Go Up, Go Down and Stop) from the Feather nRF52 to switching of 12 Volt at 3 Amperes max., the levels at which the Actuator works.</br>
<b>Adafruit Feather nRF52 Bluefruit</b></br>
Is another easy-to-use all-in-one Bluetooth Low Energy board, with a native-Bluetooth chip, the nRF52832! It's Adafruits's take on an 'all-in-one' Arduino-compatible + Bluetooth Low Energy with built in USB and battery charging. It is a low power, handsome and fast processor board with lots of memory and I/O pins. Can easily be programmed over the USB connection. The Feather nRF52 is communicating with (a) the trainer to collect resistance information (like grade) and (b) with your mobile phone to transfer information that is shown in the Companion App. The Feather nRF52 is in full control of the Simcline operation.</br>
<b>OLED display blue 128x64 pixels</b></br>
Small display with screen of: 26.6 mm x 19 mm. Shows cycling data and diagnostic info that is gathered during operation by the Feather nRF52 to inform the Simcline user about relevant information. NOTICE: Install Right Angle Through Hole Male PCB Header Pins on the board; these will allow later flat mounting of the board on top of the frame!</br>
<b>Time-of-Flight-Distance sensor VL6180X</b></br>
The sensor contains a very tiny laser source, and a matching sensor. The VL6180X can detect the "time of flight", or how long the laser light has taken to bounce back to the sensor. Since it uses a very narrow light source, it is perfect for determining distance of only the surface directly in front of it. The sensor registers quite accurately the (change in) position of the wheel axle during operation, by measuring the distance between the top of the inner frame and the reflection plate that is mounted on the carriage. The distance feedback of the sensor is crucial for determining how to set the position of the carriage and axle in accordance with the grade information that for example Zwift is using to set the resistance of the trainer.NOTICE: Install Right Angle Through Hole Male PCB Header Pins on the board.</br>
In retrospect I do not regret the component choices made. All components are documented very well. There are lots of examples for use in an Arduino enviroment. They have turned out to be very reliable.</br>

# How to start?</br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/Simcline_circuitry_02.jpg" ALIGN="left" width="250" height="250" alt="Cardboard">
When I started the project I did not have any practical experience with any of the components. So I had to setup the circuitry step by step adding components and did a lot of time consuming but instructive testing first.<br>
My advice is to setup the electronic components first in a similar way as shown on the photo with the cardboard base. Use double sided adhesive tape but only attach it on sections that have no pcb-wiring or soldering, to avoid possible electrical interference. Install the Arduino IDE and all the libraries on a PC/Mac. You will find in this Github repository all the code that controls the Simcline and the Arduino test programs (modified for this project) that focus on components seperately and in conjunction. Download all the code from Github and install.<br clear="left"/>

# ANT+, FE-C protocol and BLE</br>
It took me a lot of time studying the relevant techniques, protocols and software tools involved with ANT+, BLE and FE-C to acquire the approriate knowledge. Many projects at Github address ANT+ and/or BLE with a different point of departure, however I have learnt a lot of looking closely at the many program codes, explanations and descriptions. Finally, I tracked down how a smart up/down lift of my front wheel axle is optimally benefitting from the (ANT+) FE-C protocol when I am indoor riding with the TACX Neo in the hilly Zwift game world or with any other virtual cycling program that simulates road grade. After a succesfull proof of concept with ANT+ and BLE tools (see [ThisisANT Tools](https://www.thisisant.com/developer/resources/software-tools/) and [Nordic NRF Connect](https://www.nordicsemi.com/Software-and-tools/Development-Tools/nRF-Connect-for-mobile)) I was convinced to bring the project to a successful conclusion!
<center><img src="https://github.com/Berg0162/simcline/blob/master/images/Devices_Interacting_FEC_ANT.jpg" width="800" height="300" alt="FE-C"></center>
<b>ANT+ Trainer Control (FE-C)</b></br>
The ANT+ FE-C protocol details the bi-directional message communication of trainer data (speed, power, etcetera) from trainer to the controller and from the controller to the trainer commands or settings of targeted power, trainer resistance (grade) and calibration controls. See [ThisisAnt]( http://www.thisisant.com/) for all conceivable documentation, tools, implementations, ANT+ based products, etcetera! See at Github for a detailed description of the [ANT Message Protocol and Usage](https://github.com/Berg0162/simcline/blob/master/docs/ANT_Message_Protocol_and_Usage_Rev_5.1.pdf).
The majority of FE-C trainers on the market is dual ANT+ & Bluetooth Smart in some capacity. The Tacx Neo generation has full capacity: ANT+ Trainer Control (FE-C) and Bluetooth Smart.</br>
Tacx published in 2015 a document [TACX, FE-C and Bluetooth](https://github.com/Berg0162/simcline/blob/master/docs/How_to_FE_C_over_BLE_v1_0_0.pdf) that explains how to use the FE-C ANT+ protocol over BLE feature implemented on all Tacx Smart Trainers. Tacx designed this feature because at that time an open standard (on BLE) for trainers was lacking. Now there is (<b>FTMS</b>) FiTness Machine Service protocol to control fitness equipment over Bluetooth. According to the smart trainer recommendations guide winter 2019-2020 of [DCRainmaker](https://www.dcrainmaker.com/2019/10/the-smart-trainer-recommendations-guide-winter-2019-2020.html) the situation evolved:
> Meanwhile, for Bluetooth Smart, there’s FTMS, which is basically the same thing as FE-C when it comes to trainers. It’s not quite as widely adopted yet by trainer companies, but is by app companies. On the trainer company side only Elite, Saris, and Kinetic support it across the board. With Tacx having it on some but not all units, and Wahoo having it on no units (but all Wahoo and Tacx trainers support private Bluetooth Smart with all major apps anyway).</br>

<b>FE-C over Bluetooth</b></br>
In the document Tacx described how Tacx Smart trainers use an ANT+ FE-C definition for the data content, but transports this data over a BLE serial port service. The prerequisite that remains for this technique is thourough knowledge of ANT+ <b>AND</b> the ANT+ FE-C protocol. A complete description of FE-C can be found here at [Github](https://github.com/Berg0162/simcline/blob/master/docs/ANT+_Device_Profile_Fitness_Equipment_Rev_5.0.pdf).</br>
<b>Nordic Serial Port Service</b></br>
The transport method that is used is derived from the Nordic Uart Service. This serial port service can basically send and receive arrays of data ranging from 1..20 bytes. The only difference is the service UUID and the two characteristics UUID’s. These are renamed conforming the internal Tacx standard UUID with the specific part for the FE-C service starting with 0xFEC.</br>
<b>ANT+ and BLE Roles</b></br>
The Adafruit Feather is equiped with a native-Bluetooth chip, the nRF52832 of Nordic Semiconductor. This chip can be loaded with "SoftDevices" for different applications. Nordic nRF52 and nRF51 Series support ANT+ and ANT+ / Bluetooth LE SoftDevices. However, the adafruit version is loaded with SoftDevice S132: a high-performance Bluetooth Low Energy protocol stack for the nRF52810 and nRF52832 System-on-Chips (SoCs). It supports up to 20 concurrent links in all roles (Broadcaster, <b>Central</b>, Observer, <b>Peripheral</b>). It is Bluetooth 5.1 qualified and supports the following Bluetooth features: high-throughput 2 Mbps, Advertising Extensions and channel selection algorithms. 
In our design the SIMCLINE has a <b>Central</b> and as well as a <b>Peripheral</b> role simultaneously. The communication SIMCLINE to Trainer is Central to Peripheral respectively and the communication SIMCLINE to Smartphone is Peripheral to Central. The peripheral BLE Device is advertising its name, services and characteristics and the central device is deciding to connect and control the communication. </br>
The communication between the trainer (<b>controllable</b>) and the PC/MAC/Tablet-application (<b>controller</b>, like Zwift) is full blown ANT+ and complies the FE-C protocol. The SIMCLINE is not interfering with this setup in any way! The application on PC/MAC/Tablet is <b>controller</b> and remains in full and only charge of setting the trainers resistance.</br>
<b>How to detect the grade of the simulated track?</b></br>
The SIMCLINE is paired with the trainer over a different channel: Bluetooth! In that configuration it is complying to the ANT+ FE-C protocol as well but over Bluetooth LE. The trainer is not only broadcasting FE-C messages with cycling data (speed, cadence, power, etcetera) over ANT+ to the <b>controller</b>-application (like Zwift), but also over the BLE connection to the paired Feather nRF52. The Feather nRF52 is dealing with these data in its own way, independent of the <b>ANT+ controller</b>-application.
At regular intervals the Feather nRF52 is programmed to send a socalled Common Page 70 (0x46) (Request Data Page) with the request data page field set to Data Page <b>51</b>.
```C++
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
```
Sending Common Page 70 allows a connected device to request specific data pages from the trainer. The trainer replies with Common Page 71 (0x47) (Command Status) to the requester: the Feather nRF52. The purpose of the command status page is to confirm the status of commands (and settings) sent from a controller to the controllable trainer. The last <b>settings</b> of the Data Page 51 (0x33) (Track Resistance) are included in the data of Common Page 71. The Track Resistance Page itself is sent by the controller (like Zwift) to command the trainer to use simulation mode, and to set the desired track resistance factors. It provides the simulation parameters for the trainer, the rolling resistance and gravitational resistance applied to the rider. </br>
```C++
.
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
.
```
By sending regularly a request for Data Page 51 (0x33) (Track Resistance) the SIMCLINE is always informed about the settings of the current grade of the simulated track and the coefficient of rolling resistance. These values are both set by the <b>ANT+ controller</b>. For proper functioning of the SIMCLINE only the current road grade is critical.</br>
# Overview of Arduino Program Code Flow and Snippets</br>
Include headers of libraries and declare classes
```C++
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
```
Define variables, set to default values and initialize classes
Setup()
Get or set (first time only) the values of relevant and crucial variables to persistence, whith the Companion App the user can set these on the fly!
Start the show for the SSD1306 Oled display 
Initialize Lifter Class data, variables, test and set to work !
```C++
lift.Init(actuatorOutPin1, actuatorOutPin2, MINPOSITION, MAXPOSITION, BANDWIDTH);

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
```
Initialize Bluefruit with maximum connections as Peripheral = 1, Central = 1
Declare Callbacks for Peripheral (smartphone connection) and Callbacks for Central (trainer connection)
Setup Central Scanning for an advertising TACX trainer...
```C++
.
Bluefruit.Scanner.filterUuid(TACX_FEC_PRIMARY_SERVICE_Uuid);
.
// Initialize TACX FE-C trainer services and characteristics
// set up callback for receiving ANT+ FE-C packets; this is the main work horse!
fecrd.setNotifyCallback(fecrd_notify_callback);
// Initialize some characteristics of the Device Information Service.
// ---------------  All initialized --> Start the actual scanning   -------------
// Show Scanning message on the Oled
ShowOnOledLarge("Scanning", "for", "Trainer", 500);
Bluefruit.Scanner.start(300); // 0 = Don't stop scanning or after n, in units of hundredth of a second (n/100)
while (Bluefruit.Scanner.isRunning()) { // do nothing else but scanning....
}
// Initialize and setup BLE Uart functionality for connecting to smartphone
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
```
The callback functions are dominating completely the processing and loop() would never have been called, since there is a constant stream of FE-C packets that are coming in! <b>fecrd_notify_callback</b> does the bulk of the work!
```C++
void loop()
{ // Do not use ... !!!
}
```
fecrd_notify_callback is hooked callback that is triggered when a ANT+ message is sent from TACX Trainer
```C++
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
```
All ANT+ pages are handled, parsed and relevant variables set
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
```
Send a request for Page 51 about every 4 seconds
done------------------------------------------



# Physical Construction of SIMCLINE </br>

some text....</br>

# Construction of SIMCLINE </br>
<img src="https://www.instructables.com/assets/img/instructables-logo-v2.png" width="32" height="48" alt="Instructables"> [See Instructables](https://www.instructables.com)
