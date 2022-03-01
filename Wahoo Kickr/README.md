# <img src="https://github.com/Berg0162/simcline/blob/master/images/SC_logo.png" width="64" height="64" alt="SIMCLINE Icon"> &nbsp; SIMCLINE for Wahoo KICKR
# Simulation of Changing Road Inclination for Indoor Cycling<br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/Simcline_And_Wheel.jpg" width="300" height="300" ALIGN="left" alt="SIMCLINE">
The SIMCLINE physically adjusts the bike position to mimic hilly roads, climbing and descending. This allows the rider to naturally change position on the bike, engage climbing muscles, and improve pedaling technique to become a more efficient and powerful climber.<br>
Without user intervention the SIMCLINE will replicate inclines and declines depicted in (online & offline) training programs (like Zwift, Rouvy, VeloReality and many others) that adjust accordingly the resistance of the indoor trainer.<br>
The SIMCLINE auto connects at power up with a smart Wahoo KICKR trainer and let's relive the ascents and descents from favorite rides or routes while training indoors.<br>
The physical reach is: 20% maximum incline and -10% maximum decline. However, the reach that the rider is comfortable with can be adjusted!<br>
The SIMCLINE pairs directly to the Wahoo KICKR smart trainer and with your PC/Laptop with (Zwift) training App for a connection that notifies the SIMCLINE to simulate autonomous the (change in) physical grade of the road during an indoor ride.<br>
During operation an OLED display shows the road grade in digits and in graphics.<br>
The SIMCLINE Companion App (for Android smartphones) can be paired, only when the training App is disconnected, for adjusting operational settings, like Ascent Grade Limit (between 0-20%), Descent Grade Limit (between 0-10%), Road Grade Change Factor (between 0-100%) and manual Up and Down control. Notice that the Companion App has a slightly different functionality depending of what brand of trainer (TACX or Wahoo) is connected, due to specific connectivity differences. <br clear="left"> 

# Man-In-The-Middle (MITM) software pattern<br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/Man_in_the_Middle.png" align="left" width="1000" height="500" alt="Man in the Middle"><br>
TACX published in 2015 a document [TACX, FE-C and Bluetooth](https://github.com/Berg0162/simcline/blob/master/docs/How_to_FE_C_over_BLE_v1_0_0.pdf) that explains how to use the FE-C ANT+ protocol over BLE feature implemented on all(?) TACX Smart Trainers. TACX designed this feature because at that time an open standard (on BLE) for trainers was lacking. However Wahoo did not officially publish any document describing their proprietary protocol to control a Wahoo KICKR over BLE. The major training App developers were invited (by Wahoo) to design their Apps for use with Wahoo products. No doubt they had to sign a non-disclosure agreement. While TACX has been transparent, Wahoo was definitely NOT! However, the internet is a great source for information about not so transparent companies. At first the Simcline was designed to successfully work with smart TACX trainers and after publication in the public domain several people applied the design to built their own. However most remarks/questions I received were about the application of a Wahoo KICKR trainer. Early in the year 2022, I decided to put an effort in modifying the Simcline code in such a way that it would operate with a Wahoo KICKR and I have choosen to apply the Man-in-the-middle software pattern, since that will work with all smart Wahoo KICKR trainers, young and relatively older!<br>

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
In the Github repository (see above) you will find the appropriate test code named: <b>Test_Wahoo_Client_v03</b>, <b>Test_Wahoo_Server_v03</b> and <b>Test_Wahoo_Zwift_Bridge_v03</b>. It is coded with the only intention to check if the MITM solution is delivering in your specific situation.<br>

<b>What it does in short:</b><br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/Wahoo_Feather_Zwift_BLE.png" align="left" width="1000" height="500" alt="Simcline in the Middle"><br>
The <b>Test_Wahoo_Zwift_Bridge</b> code links a bike trainer (BLE Server Wahoo KICKR) and a PC/Laptop (BLE Client running Zwift) with the Feather nRF52840/832, like a bridge in between. The MITM bridge can pass on, control, filter and alter the interchanged trafic data! This test code is fully ignorant of the mechanical components that drive the Simcline. It simply estabishes a virtual BLE bridge and allows you to ride the bike on the Wahoo trainer and feel the resistance that comes with it, thanks to Zwift. The experience should not differ from a normal direct one-to-one connection, Zwift - Wahoo KICKR!<br>
+ The client-side scans and connects with the Wahoo relevant Cycling Power Service (<b>CPS</b>) plus the additional Wahoo proprietary CPS characteristic and collects cyling power data like Zwift would do! The code with the name: <b>Test_Wahoo_Client_v03</b> is doing just that one side of the "bridge"!
+ The Server-side advertises and enables connection with Cycling apps like Zwift and collects relevant resistance data, it simulates as if an active Wahoo trainer is connected to Zwift or alike! The code with the name: <b>Test_Wahoo_Server_v03</b> is doing just the other side of the "bridge"!
+ The <b>Test_Wahoo_Zwift_Bridge_v03</b> code is connecting both sides at the same time: the full-blown bridge!<br clear="left">

<b>How to make it work?</b><br>
The requirements are simple: 
+ running Zwift app or alike, 
+ working Feather nRF52840/52832 board <u>plus</u> SSD1306 Oled display and 
+ a Wahoo KICKR trainer.<br>

<b>Use the test code for reconnaissance!</b><br>
Please follow the instructions at the first part of the program code!
+ Start your reconnaissance with running <b>Test_Wahoo_Client_v03</b> and experience how the Feather is controlling the resistance of your Wahoo trainer. 
+ Next step is running <b>Test_Wahoo_Server_v03</b>, pairing with Zwift and then notice how your avatar is moving effortless in the Zwift world controlled by the Feather.<br>

<i>The 2 test programs (Client and Server) are NOT using a SSD1306 display, only Serial Monitor to show what is happening!</i><br>
Please write down the MAC or Device Addresses of a) your Wahoo trainer and b) your Desktop/Laptop with Zwift. These are presented in the Serial Monitor log file when running the Client and Server test code.<br>

<b>Now it is time to test the bridge!</b><br>
The <b>Test_Wahoo_Zwift_Bridge_v03</b> code needs these "hardware" addresses to unmistakingly establish a BLE connection with the right device. I know it can be implemented differently but this is to avoid unwanted BLE connection(s) with an additional power meter, another fitness device or a second computer/laptop, etcetera. The two precise device addresses are critical to assure a reliable test! You have to insert the values in the program code!<br> 

1) First insert in the <b>Test_Wahoo_Zwift_Bridge_v03</b> code the two precise BLE MAC Addresses it has to connect with
2) Upload and Run this code on the Feather nRF52840
2) Start the Serial Monitor to catch debugging info
3) Start/Power-On the Wahoo trainer  
4) Feather and Trainer will pair as reported on the Serial Monitor
5) Start Zwift on your computer or tablet
6) Search on Zwift pairing screen "<b>Power</b>" for the Feather nRF52 a.k.a. "<b>Wahoo Sim</b>"
7) Pair <b>Power</b> and <b>Controllable</b> with "<b>Wahoo Sim</b>"
8) Notice Wahoo does NOT support Speed nor Cadence, optionally pair with alternative
9) After successful pairing start the default Zwift ride or any ride you wish
10) Make Serial Monitor visible on top of the Zwift window 
11) Hop on the bike and make it happen..
12) Inspect the info presented by Serial Monitor and check the SSD1306 for the Zwift road inclination values.....
<br clear="left">

# Electronic Components and Circuitry<br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/Simcline_circuitry.jpg"  alt="Circuitry">
This project could have been elaborated with many different electronic parts that would lead to more or less the same succesfull end product. I have choosen for the following 4 active components: <br>
<b>Cytron Motor Driver MDD3A.</b> Two channel motor driver for 12 V and 3 Amperes with buttons to test manually the working of the attached DC motor. This board enables the processor to set the Actuator motor in up or down movement. It transforms logical digital levels (Go Up, Go Down and Stop) from the Feather nRF52 to switching of 12 Volt at 3 Amperes max., the levels at which the Actuator works.<br>
<b>Adafruit Feather nRF52840 Express</b><br>
Is another easy-to-use all-in-one Bluetooth Low Energy board with a native-Bluetooth chip, the nRF52840! Notice that the Feather nRF52840 Express is to be prefered and has better value for money!

> This chip has twice the flash, and four times the SRAM of its earlier sibling, the nRF52832 - 1 MB of FLASH and 256KB of SRAM. Compared to the nRF51, this board has 4-8 times more of everything.

It's Adafruits's take on an 'all-in-one' Arduino-compatible + Bluetooth Low Energy with built in USB and battery charging. It is a low power, handsome and fast processor board with lots of memory and I/O pins. Can easily be programmed over the USB connection. The programmed Feather nRF52840 is communicating with (a) the trainer to collect power output  information and (b) with the training App for resistance settings (like grade) or (c) with the Companion App on your mobile phone. The programmed Feather nRF52 is in full control of the Simcline operation.<br>
<b>OLED display blue 128x64 pixels</b><br>
Small display with screen of: 26.6 mm x 19 mm. Shows cycling data and diagnostic info that is gathered during operation by the programmed Feather nRF52 to inform the Simcline user about relevant information. NOTICE: Install Straight Pin Through Hole Male PCB Header on the board; these will allow later flat mounting of the board on top of the frame!<br>
<b>Time-of-Flight-Distance sensor VL6180X</b><br>
The sensor contains a very tiny laser source, and a matching sensor. The VL6180X can detect the "time of flight", or how long the laser light has taken to bounce back to the sensor. Since it uses a very narrow light source, it is perfect for determining distance of only the surface directly in front of it. The sensor registers quite accurately the (change in) position of the wheel axle during operation, by measuring the distance between the top of the inner frame and the reflection plate that is mounted on the carriage. The distance feedback of the sensor is crucial for determining how to set the position of the carriage and axle in accordance with the grade information that for example Zwift is using to set the resistance of the trainer. NOTICE: Install Right Angle Through Hole Male PCB Header Pins on the board.<br>
In retrospect I do not regret the component choices made. All components are documented very well. There are lots of examples for use in an Arduino enviroment. They have turned out to be very reliable.<br clear="left">

<b>How to detect the grade of the simulated track?</b><br>
The SIMCLINE is paired with the trainer over a different channel: Bluetooth! In that configuration it is complying to the ANT+ FE-C protocol as well but over Bluetooth LE. The trainer is not only broadcasting FE-C messages with cycling data (speed, cadence, power, etcetera) over ANT+ to the <b>controller</b>-application (like Zwift), but also over the BLE connection to the paired Feather nRF52. The program of the Feather nRF52 is dealing with these data in its own way, independent of the <b>ANT+ controller</b>-application.
At regular intervals the Feather nRF52 is programmed to send a socalled Common Page 70 (0x46) (Request Data Page) with the request data page field set to Data Page <b>51</b>.
```C++
.
.
```

```C++
.
.
```

# Overview of Arduino Program Code Flow and Snippets<br>
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

# Mechanical Construction of SIMCLINE<br>
There is an elaborated <b>Instructable</b> available with all the nitty gritty of how to buy or create, construct and install the various parts and components of the SIMCLINE.<br> 
See: <img src="https://www.instructables.com/assets/img/instructables-logo-v2.png" width="32" height="48" align="left" alt="Instructables"> [SIMCLINE Instructables](https://www.instructables.com/id/SIMCLINE-Simulation-of-Changing-Road-Incline-for-I/)
<br clear="left">

# Simcline in TTS4 controlled operation<br>
In the following images is shown how the Simcline is operating when different inclination values are applied by the training software. In the present situation the Simcline is controlled by the TACX Training System (TTS4 is no longer supported by Tacx). The software is still running on my laptop and is operating in Workout Mode: <b>Slope-Time</b>. After setting the slope (in workout mode by the user) the Tacx trainer is instructed to set resistance exactly with that inclination. The Simcline polls regularly for this value with the Tacx trainer and sets the height of the front wheel axle in accordance. Notice the position and inclination with respect to the measuring tape. The measuring tape starts (zero centimeter position) at the lowest possible position of -10%. 10 Centimeter height is flat road level (0% inclination)....<br> Notice that TTS4 in this mode is <b>WhatYouSeeIsWhatYouGet</b>, the inclination that is shown on the app screen is exactly sent to the trainer for resistance setting!<br>
Please notice that today many training apps are concerned more about "<i>the optimal user experience</i>" rather than with <b>WYSIWYG</b>, with respect to the road inclination. Most apps manipulate in some way the inclination that is sent to the trainer. When it is made clear <b>HOW</b> that inclination is manipulated (like the Difficulty Setting with Zwift) one can choose to allow for that <i>optimal user experience</i> or undo the setting and go for real road feel!<br>
So when the Simcline seems not to follow the road inclination values on the training app screen, be aware that the training app is <b>NOT WYSIWYG</b>!
<br>

<img src="https://github.com/Berg0162/simcline/blob/master/images/Simcline_in_operation.jpg" width="1000" height="600" alt="Simcline at work"><br clear="left">
<br>


