/*********************************************************************
 This is programming code for ESP32 Espressif Wroom boards
 
      Tested with Adafruit Feather ESP32 V2 a.k.a. Huzzah
 
 The code uses heavily the supplied ESP32 NimBLE libraries !!          
      see: https://github.com/h2zero/NimBLE-Arduino
 Many have invested time and resources providing open source code!
 
        MIT license, check LICENSE for more information
        All text must be included in any redistribution
*********************************************************************/


/*  This sketch heavily uses the NimBLE library to implement a custom client (a.k.a. CENTRAL) that 
 *  is used to listen and talk with a Gatt server on the FE-C capable indoor cycling trainer. 
 *  In our case a TACX Indoor Bike Trainer of type NEO... a so called "smart trainer" that is capable
 *  of working with BLE and ANT+ using the standard FE-C protocol for this type of equipment.  
 *  In addition a mobile phone can be (optionally) connected over BLE UART, (a) to set
 *  critical and persistent values that constrain high level functions, (b) to manually
 *  control, up and down, movement (in absence of a connected trainer), and (c) to present
 *  the most relevant real time cycling data (from the trainer) on the display of the mobile phone...
 *  In analogy to Zwift you need to download and install the SIMCLINE Companion App on your phone!
 *  Note: you need a TACX Trainer AND/OR Mobile Phone to exploit this sketch to its max.
 *  Stand alone (i.e. with no BLE connectable peripherals) the sketch tests well functioning and
 *  generates messages on the "internal" Oled display to signal its findings and status. Even when
 *  the Actuator is not connected to 12 V power the sketch is in control and visibly functioning....
 *  
 *  Requirements: Operational Simcline and a FE-C Ant+ over BLE supporting indoor trainer -> TACX NEO
 *  0) Upload and Run this code on the Simcline (i.c. ESP32 board)
 *  1) Start the Serial Monitor to catch debugging info and check the Oled display
 *  2) The code will do basic testing of mechanical parts and sensors
 *  3) Start/Power On the TACX trainer  
 *  4) Simcline and TACX will pair as reported in the output
 *  5) Start the default Zwift/Tacx/Rouvy/etcetera training App on your PC/Mac
 *  6) Pair the TACX trainer with the training app and start the ride you wish
 *  7) Make Serial Monitor output window visible on top of the App window 
 *  8) Hop on the bike: do the work and feel resistance change with the road
 *  9) Inspect the info presented by Serial Monitor.....
 *  
 * --------------------------------------------------------------------------------------------------------------------------------------
 *  NOTICE that many older smart trainer devices allow ANT+ and BLE, however, they only support 1 (ONE!) BLE connection at the time, 
 *  which means that in that case you cannot concurrently connect your trainer with ZWIFT AND with the Feather ESP32 over BLE, since it 
 *  is considered a second device and will not connect over BLE. ANT+ supports by definition multiple devices to connect!!!
 *  --> Blame the economical manufacturer and not the messenger!
 *  Solution: Apply ANT+ connection for the regular Trainer-Zwift/PC-link and use a single BLE connection for connecting the Feather ESP32.
 *  I connect in addition my Garmin cycling computer with the trainer over ANT+ and have the Feather ESP32 use the single BLE connection!
 * ---------------------------------------------------------------------------------------------------------------------------------------
 */

// -------------------------------------------------------------------------------------------
// COMPILER DIRECTIVE to allow/suppress DEBUG messages that help debugging...
// Uncomment general "#define DEBUG" to activate
#define DEBUG
// Include these debug utility macros in all cases!
#include "DebugUtils.h"
#ifdef DEBUG
//#define DEBUG_MOVEMENT
//#define DEBUG_NUS_TXD
//#define DEBUG_FEC_PACKETS
#endif
// --------------------------------------------------------------------------------------------

#ifndef ADAFRUIT_FEATHER_ESP32_V2
#define ADAFRUIT_FEATHER_ESP32_V2
#endif

// Exponential EMA ALPHA filter definition
// Used to filter sequence of actuator target positions --> minimize consecutive small up/down movements
// Should be between low (10-40) is maximal and high (50-90) is minimal filtering
// Uncomment "#define EMA_ALPHA" to activate
//#define EMA_ALPHA 60    // Value is in percentage 0-99.

#define BLE_APPEARANCE_GENERIC_CYCLING 1152
#define BLE_APPEARANCE_UNKNOWN 0
#define BLE_APPEARANCE_GENERIC_PHONE 64
#define BLE_APPEARANCE_GENERIC_COMPUTER 128

#include <NimBLEDevice.h>
// We need this for setting the Server-side Generic Access Char's --> Appearance and DeviceName
#include <nimble/nimble/host/services/gap/include/services/gap/ble_svc_gap.h>

const uint8_t MAX_PAYLOAD = 20;  // Max 20 byte data size for single packet BLE transfer

// ---------------------------------------------------------------------------------
// Struct containing Device info to administer dis/connected devices
typedef struct
{
  uint8_t PeerAddress[6];
  std::string PeerName;
  uint16_t conn_handle;
  bool IsConnected;
} Device_info_t;

// Initialize connectable device registration
Device_info_t Trainer =  { { 0x00 }, "MyTrainer", BLE_HS_CONN_HANDLE_NONE, false };
Device_info_t Smartphone = { { 0x00 }, "MyPhone", BLE_HS_CONN_HANDLE_NONE, false };
// ----------------------------------------------------------------------------------

// Client Generic Access ------------------------------------------------------------
#define UUID16_SVC_GENERIC_ACCESS BLEUUID((uint16_t)0x1800)
#define UUID16_CHR_DEVICE_NAME BLEUUID((uint16_t)0x2A00)
#define UUID16_CHR_APPEARANCE BLEUUID((uint16_t)0x2A01)
#define UUID16_CHR_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS BLEUUID((uint16_t)0x2A04)
#define UUID16_CHR_CENTRAL_ADDRESS_RESOLUTION BLEUUID((uint16_t)0x2AA6)
BLERemoteService* pRemote_GenericAccess_Service;
BLERemoteCharacteristic* pRemote_GA_Appearance_Chr;                    // Read
uint16_t client_GA_Appearance_Value = BLE_APPEARANCE_GENERIC_CYCLING;  // Default decimal: 1152 -> Generic Cycling
BLERemoteCharacteristic* pRemote_GA_DeviceName_Chr;                    // Read, Write
std::string client_GA_DeviceName_Str = "Tacx";
// Server Generic Access --------------------------------------------------------------
uint16_t server_GA_Appearance_Value = BLE_APPEARANCE_GENERIC_COMPUTER;
std::string server_GA_DeviceName_Str = "Simcline";

// Client Service Device Information --------------------------------------------------
#define UUID16_SVC_DEVICE_INFORMATION BLEUUID((uint16_t)0x180A)
#define UUID16_CHR_MODEL_NUMBER_STRING BLEUUID((uint16_t)0x2A24)
#define UUID16_CHR_SERIAL_NUMBER_STRING BLEUUID((uint16_t)0x2A25)
#define UUID16_CHR_FIRMWARE_REVISION_STRING BLEUUID((uint16_t)0x2A26)
#define UUID16_CHR_HARDWARE_REVISION_STRING BLEUUID((uint16_t)0x2A27)
#define UUID16_CHR_SOFTWARE_REVISION_STRING BLEUUID((uint16_t)0x2A28)
#define UUID16_CHR_MANUFACTURER_NAME_STRING BLEUUID((uint16_t)0x2A29)
BLERemoteService* pRemote_DeviceInformation_Service;
BLERemoteCharacteristic* pRemote_DIS_ManufacturerName_Chr;  // Read
std::string client_DIS_Manufacturer_Str;
BLERemoteCharacteristic* pRemote_DIS_ModelNumber_Chr;  // Read
std::string client_DIS_ModelNumber_Str;
BLERemoteCharacteristic* pRemote_DIS_SerialNumber_Chr;  // Read
std::string client_DIS_SerialNumber_Str;
// -------------------------------------------------------------------------------------

///////////////////////////////////////////////
/////////// TAXC FE-C ANT+ over BLE ///////////
///////////////////////////////////////////////
/* TAXC FE-C ANT+ over BLE---------------------------------------------------------------
 * TACX_FE-C_PRIMARY_SERVICE      is 128 bit:    6E40FEC1-B5A3-F393-E0A9-E50E24DCCA9E 
 * TACX_FE-C_READ_CHARACTERISTIC  is 128 bit:    6E40FEC2-B5A3-F393-E0A9-E50E24DCCA9E
 * TACX_FE-C_WRITE_CHARACTERISTIC is 128 bit:    6E40FEC3-B5A3-F393-E0A9-E50E24DCCA9E
 */
BLEUUID UUID_TACX_FEC_PRIMARY_SERVICE("6E40FEC1-B5A3-F393-E0A9-E50E24DCCA9E");
BLEUUID UUID_TACX_FEC_RXD_CHARACTERISTIC("6E40FEC2-B5A3-F393-E0A9-E50E24DCCA9E");
BLEUUID UUID_TACX_FEC_TXD_CHARACTERISTIC("6E40FEC3-B5A3-F393-E0A9-E50E24DCCA9E");

static BLERemoteService* pRemote_Tacx_FEC_Service;
static BLERemoteCharacteristic* pRemote_Tacx_FEC_Rxd_Chr;  // Read Notify
static BLERemoteCharacteristic* pRemote_Tacx_FEC_Txd_Chr;  // Write No Response

// --------------------------------------------------------------------------------------

// TACX trainer specific calculated & measured basic cycling data
long PowerValue = 0;
uint8_t InstantaneousCadence = 0;
float SpeedValue = 0;
long AccumulatedElapsedTime = 0;
uint8_t PreviousETValue = 0;
float AccumulatedDistanceTravelled = 0;
uint8_t PreviousDTValue = 0;
bool IsTrainerMoving = false;

//Define the Request Page 51 Command to send
const uint8_t Page51Bytes[13] = {
  0xA4,  //Sync
  0x09,  //Length
  0x4F,  //Acknowledge message type
  0x05,  //Channel
         //Data
  0x46,  //Common Page 70
  0xFF,
  0xFF,
  0xFF,  //Descriptor byte 1 (0xFF for no value)
  0xFF,  //Descriptor byte 2 (0xFF for no value)
  0x80,  //Requested transmission response
  0x33,  //Requested page number 51
  0x01,  //Command type (0x01 for request data page, 0x02 for request ANT-FS session)
  0x47
};  //Checksum;

// FE-C Requested Page 51 globals
unsigned long REQUEST_PAGE_51_DELAY = 1250;  //Sample rate for Page 51 requests 1.25 seconds ?
unsigned long sendRequestPage51Event = 0;    //Millis of last Page 51 request event
// --------------------------------------------------------------------------------------

/* NORDIC UART SERVICE a.k.a. NUS
 * NUS Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 * NUS RXD    : 6E400002-B5A3-F393-E0A9-E50E24DCCA9E
 * NUS TXD    : 6E400003-B5A3-F393-E0A9-E50E24DCCA9E
 */
BLEUUID UUID_NUS_SERVICE("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLEUUID UUID_NUS_CHR_RXD("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
BLEUUID UUID_NUS_CHR_TXD("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");
BLEService* server_NordicUart_Service;
BLECharacteristic* server_NUS_Rxd_Chr;  // Write No Response (Receiving Data)
BLECharacteristic* server_NUS_Txd_Chr;  // Read Notify (Sending Data)

// ----------------------------------------------------------------------------------------------
BLEClient* pClient_Tacx = nullptr;
BLEAdvertisedDevice* myDevice = nullptr;
BLEScan* pBLEScan = nullptr;
BLEServer* pServer = nullptr;
NimBLEAdvertising* pAdvertising = nullptr;

// These variables are handled in loop() to start sort of Callback functions
boolean doClientConnectCall = false;
boolean RestartScanningOnDisconnect = false;

// Values used to enable or disable notifications/indications
const uint8_t notificationOff[] = { 0x0, 0x0 };
const uint8_t notificationOn[] = { 0x1, 0x0 };
const uint8_t indicationOff[] = { 0x0, 0x0 };
const uint8_t indicationOn[] = { 0x2, 0x0 };
// ---------------------------------------------------------------------------------------
// Libraries for use of I2C devices (Oled and VL6180X distance sensor)
#include <SPI.h>
#include <Wire.h>
// Necessary libraries for use of Oled display(s)
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// Additional splash screen bitmap and icon(s) for Oled
#include "Adafruit_SSD1306_Icons.h"  // needs to be in the SAME (!) directory
// Declarations for an SSD1306 128x64 display connected to I2C (SDA, SCL pins)
#define SCREEN_WIDTH 128       // SSD1306-OLED display width, in pixels
#define SCREEN_HEIGHT 64       // SSD1306-OLED display height, in pixels
#define OLED_RESET -1          // No reset pin on this OLED display
#define OLED_I2C_ADDRESS 0x3C  // I2C Address of OLED display
// Declare the display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// Declare Global var for OLED Display selection 1 (Cycling data) or 2 (Road Grade)
uint8_t OledDisplaySelection = 2;  // default Road Grade to show

// LittleFS for internal storage of persistent data on the ESP32
#include <LittleFS.h>
using namespace fs;
// Managing persistence of some data with LittleFS system
// PeRSistent Data is written to a file with the name: PRS_FILENAME
#define PRS_FILENAME "/prsdata.txt"
// Minimum(!) Read and Write block sizes
#define LITTLEFS_BLOCK_SIZE 128
#define FORMAT_LITTLEFS_IF_FAILED true
// LittleFS--------------------------------------------------------------


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
//-----------------------------------------------------------------

/* ------------------------------------------------------------------------------------------------
 * Warning I/O Pins can have identical board position but different I/O Pin declarations for 
 * connection with the pins of the Motor driver board
 * ADAFRUIT_FEATHER_ESP32_V2 is nearly pin compatible with ARDUINO_NRF52840_FEATHER
*/
#ifdef ADAFRUIT_FEATHER_ESP32_V2
#define actuatorOutPin1 A0  // --> A0/P0.02 connected to pin IN2 of the DRV8871 Motor Driver board
#define actuatorOutPin2 A1  // --> A1/P0.03 connected to pin IN1 of the DRV8871 Motor Driver board
#endif

// -------------------------- WARNING ------------------------------------------------------------
// The following VL6180X sensor values are a 100% construction specific and
// should be experimentally determined, when the Actuator AND the VL6180X sensor are mounted!
// ------>>>> Test manually and use example/test sketches that go with the VL6180X sensor! <<<<---
// Microswitches should limit physically/mechanically the upper and lower position of the Actuator!
// The microswitches are mechanically controlled, and NOT by the software --> should be fail safe!
// Notice that unrestricted movement at the boundaries can damage the Actuator and/or construction!
// The following values are respected by the software and will (in normal cases!) never be exceeded!
#define MINPOSITION 265 // VL6180X highest value top microswitch activated to mechanically stop operation
#define MAXPOSITION 535 // VL6180X lowest value bottom microswitch activated to mechanically stop operation

// -------------------------- WARNING ------------------------------------------------------------
// Operational boundaries of the VL6180X sensor are used/set in class Lifter after calling its "init".
// A safe measuring range of at least 30 cm of total movement is recommended for the VL6180X sensor setting!
//
// Bandwidth is used in the code to take measuring errors and a safe margin into account when reaching
// the above defined max or min positions of the construction! The software does painstakingly respect
// these and is independent of the appropriate working of the microswitches when reaching the boundaries!
// These microswitches are a SECOND line of defence against out of range and potentially damaging movement!
#define BANDWIDTH 4

// Library code for low level measuring (VL6180X) and controlling UP and down movement
#include <Lifter.h>
// Decalaration of Lifter Class for control of the low level up/down movement
Lifter lift;
// Global variables for Lifter position control --> RawGradeValue has been defined/set previously to flat road level!!
int16_t TargetPosition = map(RawgradeValue, RGVMIN, RGVMAX, MAXPOSITION, MINPOSITION);
bool IsBasicMotorFunctions = false;  // Mechanical motor functions
SemaphoreHandle_t xSemaphore = NULL;
TaskHandle_t ControlTaskHandle = NULL;

// ---------------------------------------------------------------------------------
void xControlUpDownMovement(void* arg);

// Client Connect and Disconnect callbacks defined
class client_Connection_Callbacks : public BLEClientCallbacks {
  void onConnect(BLEClient* pClient);
  void onDisconnect(BLEClient* pClient);
  bool onConnParamsUpdateRequest(BLEClient* pClient, ble_gap_upd_params* params);
};
// Server Connect and Disconnect callbacks defined
class server_Connection_Callbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer, ble_gap_conn_desc* desc);
  void onDisconnect(BLEServer* pServer, ble_gap_conn_desc* desc);
  void onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc);
};

bool getPRSdata(void);
void setPRSdata(void);
void ConvertMacAddress(char* fullAddress, uint8_t addr[6], bool NativeFormat);
#ifdef EMA_ALPHA
int16_t EMA_TargetPositionFilter(int16_t current_value);
#endif
void ShowIconsOnTopBar(void);
void ShowOnOledLarge(const char* Line1, const char* Line2, const char* Line3, uint16_t Pause);
void BuildBasicOledScreen(void);
void ShowValuesOnOled(void);
void ShowSlopeTriangleOnOled(void);
void SetManualGradePercentValue(void);
void SetNewRawGradeValue(float RoadGrade);
void SetNewActuatorPosition(void);
void server_setupGA(void);
void server_setupNUS(void);
void server_startADV(void);
bool client_DeviceInformation_Connect(void);
bool client_GenericAccess_Connect(void);
bool client_Tacx_FEC_Connect(void);
void client_Start_Scanning(void);
bool client_Connect_Callback(void);
void SendRequestPage51(void);
// ---------------------------------------------------------------------------------

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

  DEBUG_PRINTLN("ESP32 NimBLE SIMCLINE supporting: Tacx FE-C Trainer");
  DEBUG_PRINTLN("----------------- Version 00.0 --------------------");
  // LittleFS start the Littlefilesystem lib and see if we have persistent data ----
  // This opens LittleFS with a root subdirectory /littlefs/
  LittleFS.begin();
  // WARNING --------- Uncomment only when strictly necessary!!! ---------
  // Uncomment only the very first time the Simcline code is executed!!!
  /* This creates LittleFS with a root subdirectory /littlefs/
  LittleFS.format();
  DEBUG_PRINTLN("Wipe out all persistent data, including any file(s)....");
  */
  // Get or set (first time only) the values of relevant and crucial variables
  // to persistence, whith the Companion App the user can set these on the fly!
  // Get or set the values of aRGVmax, aRGVmin, GradeChangeFactor in PRSdata.
  if (!getPRSdata()) {
    setPRSdata();
  }
  // LittleFS------------------------------------------------------------------------

  // Start the show for the Oled display
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) {
    DEBUG_PRINTLN(F("SSD1306 OLED display allocation failed!"));
  } else {
    DEBUG_PRINTLN(F("SSD1306 OLED display is running..."));
    // Load Oled with initial display buffer contents on the screen,
    // the SSD1306 library initializes with a Adafruit splash screen,
    // (respect or edit the splash.h in the library).
    display.display();  // Acknowledge Adafruit rights, license and efforts
    delay(500);         // show some time
  }
  // Ready to show our own SIMCLINE splash screen
  display.clearDisplay();  // clean the oled screen
  display.setTextColor(SSD1306_WHITE);
  display.drawBitmap(24, 0, Mountain_bw_79x64, 79, 64, 1);
  display.display();
  delay(2000);  // Take somewhat more time.....
  //Show Name and SW version on Oled
  ShowOnOledLarge("SIMCLINE", "Tacx", "v00.0", 500);
  // Initialize Lifter Class data, variables, test and set to work !
  lift.Init(actuatorOutPin1, actuatorOutPin2, MINPOSITION, MAXPOSITION, BANDWIDTH);
  // Test Actuator and VL8106X for proper functioning
  ShowOnOledLarge("Testing", "Up & Down", "Functions", 100);
  if (!lift.TestBasicMotorFunctions()) {
    ShowOnOledLarge("Testing", "Functions", "Failed!", 500);
    IsBasicMotorFunctions = false;  // Not working properly
    DEBUG_PRINTLN("Simcline >> ERROR << Basic Motor Funtions are NOT working!!");
  } else {
    ShowOnOledLarge("Testing", "Functions", "Done!", 500);
    // Is working properly --> Start Motor Control Task
    xSemaphore = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(xControlUpDownMovement, "xControlUpDownMovement", 4096, NULL, 10, &ControlTaskHandle, 1);
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
  // Initialize NimBLE with maximum connections as Peripheral = 1, Central = 1
  BLEDevice::init("ESP32");  // Shortname of Dev. Board
  // Start the Server-side now!
  pServer = BLEDevice::createServer();
  //Setup callbacks onConnect and onDisconnect
  pServer->setCallbacks(new server_Connection_Callbacks());
  // Set server auto-restart advertise on
  pServer->advertiseOnDisconnect(true);
  // Server setup
  DEBUG_PRINTLN("Configuring the Server Generic Access Service");
  server_setupGA();
  DEBUG_PRINTLN("Configuring the Server Nordic Uart Service");
  server_setupNUS();
  DEBUG_PRINTLN("Setting up the Server advertising payload(s)");
  server_startADV();
  DEBUG_PRINTLN("Server is advertising: NUS");

  // Start the Client-side!
  pClient_Tacx = BLEDevice::createClient();
  pClient_Tacx->setClientCallbacks(new client_Connection_Callbacks());
  client_Start_Scanning();
  if (doClientConnectCall) {
    doClientConnectCall = false;
    bool dummy = client_Connect_Callback();
  }
  if (!Trainer.IsConnected) {
    DEBUG_PRINTLN(">>> Failed to connect Trainer! Reset ESP32 and try again!");
    while (1) { delay(100); }
  }
}  // End of setup.

// LittleFS --------------------------------------------------
bool getPRSdata(void) {  // aRGVmax, aRGVmin, GradeChangeFactor -> PRSdata
  if (LittleFS.exists(PRS_FILENAME)) {
    File file = LittleFS.open(PRS_FILENAME, FILE_READ);
    if (file) {
      uint32_t readLen;
      uint8_t buffer[LITTLEFS_BLOCK_SIZE + 1] = { 0 };
      readLen = file.read(buffer, LITTLEFS_BLOCK_SIZE);
      buffer[readLen] = 0;  // set the values to the pointed by variables
      sscanf((char*)buffer, "%d %d %d %d", &aRGVmax, &aRGVmin, &GradeChangeFactor, &OledDisplaySelection);
      DEBUG_PRINT(F("ESP32 internally Got persistent storage from: /littlefs/PRSdata -> "));
      DEBUG_PRINTF("Max: %d Min: %d Perc.: %d Displ.: %d\n", aRGVmax, aRGVmin, GradeChangeFactor, OledDisplaySelection);
      file.close();
      return true;
    }
  }
  return false;
}

void setPRSdata(void) {  // aRGVmax, aRGVmin, GradeChangeFactor -> PRSdata
  uint8_t buffer[LITTLEFS_BLOCK_SIZE + 1] = { 0 };
  File file = LittleFS.open(PRS_FILENAME, FILE_WRITE);
  if (file) {
    sprintf((char*)buffer, "%d %d %d %d", aRGVmax, aRGVmin, GradeChangeFactor, OledDisplaySelection);
    file.write(buffer, LITTLEFS_BLOCK_SIZE);
    file.close();
    DEBUG_PRINT(F("ESP32 internally Set new values in persistent storage to: /littlefs/PRSdata -> "));
    DEBUG_PRINTLN((char*)buffer);
  }
}
// LittleFS --------------------------------------------------

void ConvertMacAddress(char* fullAddress, uint8_t addr[6], bool NativeFormat) {  // Display byte by byte in HEX
  if (NativeFormat) {                                                            // Unaltered: in Little Endian machine-representation
    sprintf(fullAddress, "%02X:%02X:%02X:%02X:%02X:%02X", addr[0], addr[1], addr[2],
            addr[3], addr[4], addr[5], HEX);
  } else {  // Altered: In reversed order
    sprintf(fullAddress, "%02X:%02X:%02X:%02X:%02X:%02X", addr[5], addr[4], addr[3],
            addr[2], addr[1], addr[0], HEX);
  }
};

void ShowIconsOnTopBar(void) {
  // Show Icons on Top Bar
  if (Trainer.IsConnected) {  // show icon
    display.drawBitmap(112, 0, power_icon16x16, 16, 16, 1);
  }
  if (Smartphone.IsConnected) {  // show icon Phone
    display.drawBitmap(0, 0, mobile_icon16x16, 16, 16, 1);
  }
}

void ShowOnOledLarge(const char* Line1, const char* Line2, const char* Line3, uint16_t Pause) {
  // Clear and set Oled to display 3 line info -> centered
  int pos = 1;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  ShowIconsOnTopBar();
  display.setTextSize(2);  // Large characters 11 pixels wide
  if (Line1) {
    pos = round((127 - (12 * strlen(Line1))) / 2);
    display.setCursor(pos, 2);  // 16
    display.print(Line1);
  }
  if (Line2) {
    pos = round((127 - (12 * strlen(Line2))) / 2);
    display.setCursor(pos, 22);  // 16
    display.print(Line2);
  }
  if (Line3) {
    pos = round((127 - (12 * strlen(Line3))) / 2);
    display.setCursor(pos, 44);  // 16
    display.print(Line3);
  }
  display.display();
  delay(Pause);  // Pause indicated time in ms
}

// Create basic Oled screen for measurement data
void BuildBasicOledScreen(void) {
  display.clearDisplay();  // clean the oled screen
  display.setTextColor(SSD1306_WHITE);
  ShowIconsOnTopBar();
  display.setTextSize(1);
  display.setCursor(14, 32);
  display.print(F("Watt"));
  display.setCursor(62, 32);
  display.print(F("Rpm"));
  display.setCursor(99, 32);
  display.print(F("Kph"));
  display.setCursor(102, 10);
  display.setTextSize(2);
  display.print(F("%"));
}  // ---------------------------

// Funtion to show measurement data: Grade, Power, Cadence and Speed on Oled screen
void ShowValuesOnOled(void) {
  BuildBasicOledScreen();
  display.setTextColor(SSD1306_WHITE);
  char tmp[12];
  dtostrf(gradePercentValue, 5, 1, tmp);  // show sign only if negative
  display.setCursor(10, 6);
  display.setTextSize(3);
  display.print(tmp);
  sprintf(tmp, "%03d %03d %02d", PowerValue, InstantaneousCadence, int(SpeedValue + 0.5));
  display.setCursor(4, 44);
  display.setTextSize(2);
  display.print(tmp);
  display.display();
}  // -----------------------------------

void ShowSlopeTriangleOnOled(void) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  ShowIconsOnTopBar();
  display.setCursor(102, 10);  //26
  display.setTextSize(2);
  display.print(F("%"));
  char tmp[7];
  dtostrf(gradePercentValue, 5, 1, tmp);  // show sign only if negative
  display.setCursor(10, 6);               // 22
  display.setTextSize(3);
  display.print(tmp);
  // The following calculations give more "weight" to lower grade values
  // (like: 1.2% or 0.5%), these will occur more often in practice and are not well
  // displayable at 128*64! --> 64 * 64 = 4096 and this value should not be
  // exceeded (4096/20) = 204.8
  int pos = 64 - int(sqrt(abs(204 * gradePercentValue)));  // cast to int to get rid of decimals only now!
  if (gradePercentValue > 0) {
    display.fillTriangle(1, 63, 127, 63, 127, pos, SSD1306_INVERSE);
  } else {
    display.fillTriangle(127, 63, 1, 63, 0, pos, SSD1306_INVERSE);
  }
  // Draw the baseline to smooth small decimal values and show flat road case
  display.drawFastHLine(1, 63, 127, SSD1306_WHITE);
  display.display();
}
// ---------------------------------------------------------------------------------


// ---------------------------------------------------------------------------------
#ifdef EMA_ALPHA
int16_t EMA_TargetPositionFilter(int16_t current_value) {
  static int16_t exponential_average = current_value;

  exponential_average = int16_t((EMA_ALPHA * (uint32_t)current_value + (100 - EMA_ALPHA) * (uint32_t)exponential_average) / 100);
  return exponential_average;
}
#endif

void SetManualGradePercentValue(void) {
  gradePercentValue = float((RawgradeValue - 20000 + MEASUREOFFSET)) / 100;
  SetNewActuatorPosition();
}

void SetNewRawGradeValue(float RoadGrade) {
  // ----- Recalculate to relevant values for this project ------
  // Take into account the allowed Increase Percentage of the inclination
  // 100% has no effect, 50% means every increase or decrease is divided by 2
  // --> the increase or decrease of inclination is in 50% smaller steps...
  gradePercentValue = RoadGrade;
  RawgradeValue = (long)(RoadGrade * 100) + 20000;
  RawgradeValue = 20000 + long((RawgradeValue - 20000) * GradeChangeFactor / 100);
  // in steps of 0.01% and with an offset of -200%
  // gradeValue     gradePercentValue
  //     0                 -200%
  //  19000                 -10%
  //  20000                   0%
  //  22000                 +20%
  //  40000                +200%
  // -------------------------------------
  // Take into account the measuring offset
  RawgradeValue = RawgradeValue - MEASUREOFFSET;
  // Test for Maximally en Minimally Allowed Raw Grade Values ----------------------------------------
  if (RawgradeValue < aRGVmin) {
    RawgradeValue = aRGVmin;  // Do not allow lower values than aRGVmin !!
  }
  if (RawgradeValue > aRGVmax) {
    RawgradeValue = aRGVmax;  // Do not allow values to exceed aRGVmax !!
  }
  // --------------------------------------------------------------------------------------------------
#ifndef DEBUG_MOVEMENT
  //DEBUG_PRINTF("Set Simcline to Percentage: %02.1f %% RawgradeValue: %05d \n", gradePercentValue, RawgradeValue);  
#endif
}

void SetNewActuatorPosition(void) {
  // Handle mechanical movement i.e. wheel position in accordance with Road Inclination
  // Map RawgradeValue ranging from 0 to 40.000 on the
  // TargetPosition (between MINPOSITION and MAXPOSITION) of the Lifter
  // Notice 22000 is equivalent to +20% incline and 19000 to -10% incline
  RawgradeValue = constrain(RawgradeValue, RGVMIN, RGVMAX);  // Keep values within the safe range
  TargetPosition = map(RawgradeValue, RGVMIN, RGVMAX, MAXPOSITION, MINPOSITION);
  // EMA filter for smoothing quickly fluctuating Target Position values see: Zwift Titan Grove
#ifdef EMA_ALPHA
  TargetPosition = EMA_TargetPositionFilter(TargetPosition);
#endif
  if (IsBasicMotorFunctions) {
    xSemaphoreTake(xSemaphore, portMAX_DELAY);
    lift.SetTargetPosition(TargetPosition);
    xSemaphoreGive(xSemaphore);
//#ifdef DEBUG_MOVEMENT
    DEBUG_PRINTF(" -> New Position RawgradeValue: %05d Grade percent: %03.1f%% ", RawgradeValue, gradePercentValue);
    DEBUG_PRINTF("TargetPosition: %03d\n", TargetPosition, DEC);
//#endif
  }
}

void xControlUpDownMovement(void* arg) {
  // Check "continuously" the Actuator Position and move Motor Up/Down until target position is reached
  int OnOffsetAction = 0;
  while (1) {
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
      // BLE channels can interrupt and consequently target position changes on-the-fly !!
      // We do not want changes in TargetPosition during one of the following actions!!!
      OnOffsetAction = lift.GetOffsetPosition();  // calculate offset to target and determine action
      switch (OnOffsetAction) {
        case 0:
          lift.brakeActuator();
#ifdef DEBUG_MOVEMENT
          DEBUG_PRINTLN(F(" -> Brake"));
#endif
          break;
        case 1:
          lift.moveActuatorUp();
#ifdef DEBUG_MOVEMENT
          DEBUG_PRINTLN(F(" -> Upward"));
#endif
          break;
        case 2:
          lift.moveActuatorDown();
#ifdef DEBUG_MOVEMENT
          DEBUG_PRINTLN(F(" -> Downward"));
#endif
          break;
        case 3:
          // Timeout --> OffsetPosition is undetermined --> do nothing and brake
          lift.brakeActuator();
#ifdef DEBUG_MOVEMENT
          DEBUG_PRINTLN(F(" -> Timeout"));
#endif
          break;
      }  // switch
      xSemaphoreGive(xSemaphore);
    }
    vTaskDelay(20); // 10
  }  // while
}  // end

// ----------------------------------------------------------------------------------
void client_Connection_Callbacks::onConnect(BLEClient* pClient) {
  Trainer.PeerName = myDevice->getName().c_str();
  Trainer.conn_handle = pClient_Tacx->getConnId();
#ifdef DEBUG
  DEBUG_PRINT("Client Connection Parameters -> ");
  uint16_t max_payload = pClient_Tacx->getMTU() - 3;
  //DEBUG_PRINTF("Max Transmission Unit: [%d] ", max_payload);
  uint16_t clientConnectionInterval = pClient_Tacx->getConnInfo().getConnInterval();
  DEBUG_PRINTF("Interval: [%d] ", clientConnectionInterval);
  uint16_t clientConnectionLatency = pClient_Tacx->getConnInfo().getConnLatency();
  DEBUG_PRINTF("Latency: [%d] ", clientConnectionLatency);
  uint16_t clientConnectionSupTimeout = pClient_Tacx->getConnInfo().getConnTimeout();
  DEBUG_PRINTF("Supervision Timeout: [%d]\n", clientConnectionSupTimeout);
  char fullMacAddress[18] = {};                                   //
  ConvertMacAddress(fullMacAddress, Trainer.PeerAddress, false);  // true -> Native representation!
  DEBUG_PRINTF("ESP32 Client connected to Trainer device with Name: [%s] MAC Address: [%s] Handle: [%d] MTU: [%d]\n",
               Trainer.PeerName.c_str(), fullMacAddress, Trainer.conn_handle, max_payload);
#endif
  /** We can use the connection handle here to ask for different connection parameters.
     *  Args: connection handle, min connection interval, max connection interval, latency and supervision timeout.
     *  Units; Min/Max Intervals: 1.25 millisecond increments --> between: 24 (30 ms) resp. 48 (60 ms)
     *  Latency: number of intervals allowed to skip. --> keep zero
     *  Timeout: 10 millisecond increments. --> Try 400 (4000 ms)
     */
  //pClient_Tacx->updateConnParams(pClient_Tacx->getConnId(), 24, 48, 0, 400);
  //DEBUG_PRINTLN("Client Updates Connection Parameters -> Min Interval: [24] Max Interval: [48] Latency: [0] Supervision Timeout: [400]");
};

bool client_Connection_Callbacks::onConnParamsUpdateRequest(BLEClient* pClient, ble_gap_upd_params* params) {
  DEBUG_PRINTLN("Client Connection Parameter Update Request!");
  /** Minimum value for connection interval in 1.25ms units */
  uint16_t clientConnectionMinInterval = params->itvl_min;
  DEBUG_PRINTF("Min Interval: [%d]\n", clientConnectionMinInterval);
  /** Maximum value for connection interval in 1.25ms units */
  uint16_t clientConnectionMaxInterval = params->itvl_max;
  DEBUG_PRINTF("Max Interval: [%d]\n", clientConnectionMaxInterval);
  /** Connection latency */
  uint16_t clientConnectionLatency = params->latency;
  DEBUG_PRINTF("Latency: [%d]\n", clientConnectionLatency);
  /** Supervision timeout in 10ms units */
  uint16_t clientConnectionSupTimeout = params->supervision_timeout;
  DEBUG_PRINTF("Sup. Timeout: [%d]\n", clientConnectionSupTimeout);
  /** Minimum length of connection event in 0.625ms units */
  uint16_t clientMinLenEvent = params->min_ce_len;
  DEBUG_PRINTF("Min Length Event: [%d]\n", clientMinLenEvent);
  /** Maximum length of connection event in 0.625ms units */
  uint16_t clientMaxLenEvent = params->max_ce_len;
  DEBUG_PRINTF("Max Length Event: [%d]\n", clientMaxLenEvent);
  return true;  // That is OK!
};

void client_Connection_Callbacks::onDisconnect(BLEClient* pClient) {
  Trainer.IsConnected = false;
  Trainer.conn_handle = BLE_HS_CONN_HANDLE_NONE;
  char fullMacAddress[18] = {};                                   //
  ConvertMacAddress(fullMacAddress, Trainer.PeerAddress, false);  // true -> Native representation!
  DEBUG_PRINTF("Client Disconnected from Trainer device with Name: [%s] Mac Address: [%s]!\n", Trainer.PeerName.c_str(), fullMacAddress);
  // Show the message on the Oled
  ShowOnOledLarge("Trainer", "Connection", "Lost!", 500);
  RestartScanningOnDisconnect = true;
};

bool client_DeviceInformation_Connect(void) {
  // If Device Information is not found then go on.... NOT FATAL !
  pRemote_DeviceInformation_Service = pClient_Tacx->getService(UUID16_SVC_DEVICE_INFORMATION);
  if (pRemote_DeviceInformation_Service == nullptr) {
    DEBUG_PRINT(F("Device Information Service: NOT Found!\n"));
    return true;
  }
  DEBUG_PRINT(F("Client Device Information Service: Found!\n"));
  pRemote_DIS_ManufacturerName_Chr = pRemote_DeviceInformation_Service->getCharacteristic(UUID16_CHR_MANUFACTURER_NAME_STRING);
  if (pRemote_DIS_ManufacturerName_Chr != nullptr) {
    if (pRemote_DIS_ManufacturerName_Chr->canRead()) {
      client_DIS_Manufacturer_Str = pRemote_DIS_ManufacturerName_Chr->readValue();
      DEBUG_PRINTF(" -> Client Reads Manufacturer Name: [%s]\n", client_DIS_Manufacturer_Str.c_str());
    }
  }
  pRemote_DIS_ModelNumber_Chr = pRemote_DeviceInformation_Service->getCharacteristic(UUID16_CHR_MODEL_NUMBER_STRING);
  if (pRemote_DIS_ModelNumber_Chr != nullptr) {
    if (pRemote_DIS_ModelNumber_Chr->canRead()) {
      client_DIS_ModelNumber_Str = pRemote_DIS_ModelNumber_Chr->readValue();
      DEBUG_PRINTF(" -> Client Reads Model Number:      [%s]\n", client_DIS_ModelNumber_Str.c_str());
    }
  }
  pRemote_DIS_SerialNumber_Chr = pRemote_DeviceInformation_Service->getCharacteristic(UUID16_CHR_SERIAL_NUMBER_STRING);
  if (pRemote_DIS_SerialNumber_Chr != nullptr) {
    if (pRemote_DIS_SerialNumber_Chr->canRead()) {
      client_DIS_SerialNumber_Str = pRemote_DIS_SerialNumber_Chr->readValue();
      DEBUG_PRINTF(" -> Client Reads Serial Number:     [%s]\n", client_DIS_SerialNumber_Str.c_str());
    }
  }
  return true;
}

bool client_GenericAccess_Connect(void) {
  // If Generic Access is not found then go on.... NOT FATAL !
  pRemote_GenericAccess_Service = pClient_Tacx->getService(UUID16_SVC_GENERIC_ACCESS);
  if (pRemote_GenericAccess_Service == nullptr) {
    DEBUG_PRINTLN(F("Client Generic Access: NOT Found!"));
    return true;
  }
  DEBUG_PRINTLN("Client Generic Access: Found!");
  pRemote_GA_DeviceName_Chr = pRemote_GenericAccess_Service->getCharacteristic(UUID16_CHR_DEVICE_NAME);
  if (pRemote_GA_DeviceName_Chr != nullptr) {
    if (pRemote_GA_DeviceName_Chr->canRead()) {
      client_GA_DeviceName_Str = pRemote_GA_DeviceName_Chr->readValue();
      DEBUG_PRINTF(" -> Client Reads Device Name:   [%s]\n", client_GA_DeviceName_Str.c_str());
    }
  }
  pRemote_GA_Appearance_Chr = pRemote_GenericAccess_Service->getCharacteristic(UUID16_CHR_APPEARANCE);
  if (pRemote_GA_Appearance_Chr != nullptr) {
    if (pRemote_GA_Appearance_Chr->canRead()) {
      client_GA_Appearance_Value = pRemote_GA_Appearance_Chr->readUInt16();
      DEBUG_PRINTF(" -> Client Reads Appearance:    [%d]\n", client_GA_Appearance_Value);
    }
  }
  return true;
}

void server_NUS_Txd_GRD(void) {
  uint8_t TX_GRD_Str[9] = { 0 };
  sprintf((char*)TX_GRD_Str, "!G%.1f;", gradePercentValue);
  if (IsTrainerMoving) {
    server_NUS_Txd_Chr->notify(TX_GRD_Str, strlen((char*)TX_GRD_Str));
#ifdef DEBUG_NUS_TXD
    DEBUG_PRINT(F("-> server_NUS_Txd sends GRD: "));
    DEBUG_PRINTF("[%s] Ismoving: [%s]\n", TX_GRD_Str, (IsTrainerMoving ? "true" : "false"));
#endif
  }
  return;
}

void server_NUS_Txd_PWR_CAD(void) {
  uint8_t TX_PC_Str[MAX_PAYLOAD] = { 0 };
  sprintf((char*)TX_PC_Str, "!C%d;%d;", PowerValue, InstantaneousCadence);
  if (IsTrainerMoving) {
    server_NUS_Txd_Chr->notify(TX_PC_Str, strlen((char*)TX_PC_Str));
#ifdef DEBUG_NUS_TXD
    DEBUG_PRINT(F("-> server_NUS_Txd sends PWR CAD: "));
    DEBUG_PRINTF("[%s] IsMoving: [%s]\n", TX_PC_Str, (IsTrainerMoving ? "true" : "false"));
#endif
  } else {
    server_NUS_Txd_Chr->setValue("!CPaused..;");
    server_NUS_Txd_Chr->notify();
#ifdef DEBUG_NUS_TXD
    DEBUG_PRINT(F("-> server_NUS_Txd sends PWR CAD: "));
    DEBUG_PRINTF("[%s] IsMoving: %s\n", "!APaused..;" , (IsTrainerMoving ? "true" : "false"));
#endif     
  }
  return;
}

void server_NUS_Txd_AET_SPD_TIME(void) {
  uint8_t TX_AET_SPD_TIME_Str[MAX_PAYLOAD] = { 0 };
  char StrTime[10] = { 0 };
  sprintf(StrTime, "%02d:%02d", ((AccumulatedElapsedTime / 4) / 3600), ((AccumulatedElapsedTime / 4) % 3600 / 60));
  sprintf((char*)TX_AET_SPD_TIME_Str, "!A%.1f;%.1f;%s;", (AccumulatedDistanceTravelled / 1000), SpeedValue, StrTime);
  if (IsTrainerMoving) {
    server_NUS_Txd_Chr->notify(TX_AET_SPD_TIME_Str, strlen((char*)TX_AET_SPD_TIME_Str));
#ifdef DEBUG_NUS_TXD
    DEBUG_PRINT(F("-> server_NUS_Txd sends AET-SPD-TIME: "));
    DEBUG_PRINTF("[%s] IsMoving: %s\n", TX_AET_SPD_TIME_Str, (IsTrainerMoving ? "true" : "false"));
#endif
  } else {
    server_NUS_Txd_Chr->setValue("!APaused.;");
    server_NUS_Txd_Chr->notify();
#ifdef DEBUG_NUS_TXD
    DEBUG_PRINT(F("-> server_NUS_Txd sends AET-SPD-TIME: "));
    DEBUG_PRINTF("[%s] IsMoving: %s\n", "!APaused.;" , (IsTrainerMoving ? "true" : "false"));
#endif    
  }
  return;
}

void Tacx_FEC_Rxd_Notify_Callback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  // The FE-C Read charateristic of ANT+ packets
  // In TACX context receive or send arrays of data ranging from 1--20 bytes so FE-C
  // will not exceed the 20 byte maximum

  uint8_t buffer[MAX_PAYLOAD];
  memset(buffer, 0, sizeof(buffer));
  if (length > MAX_PAYLOAD) { length = MAX_PAYLOAD; }
#ifdef DEBUG_FEC_PACKETS
  DEBUG_PRINTF("Rec'd Raw FE-C Data len: [%02d] [", length);
#endif
  for (int i = 0; i < length; i++) {
    if (i <= sizeof(buffer)) {
      buffer[i] = *pData++;
#ifdef DEBUG_FEC_PACKETS
      DEBUG_PRINTF("%02X ", buffer[i], HEX);
#endif
    }
  }
#ifdef DEBUG_FEC_PACKETS
  DEBUG_PRINT("] ");
#endif
  uint8_t PageValue = buffer[4];  // Get Page number from packet
  switch (PageValue) {
    case 0x47:
      ////////////////////////////////////////////////////////////
      //////////////////// Handle PAGE 71 ////////////////////////
      ////////////// Requested PAGE 51 for grade info ////////////
      ////////////////////////////////////////////////////////////
      if (buffer[5] == 0x33) {  // check for requested page 51
        uint8_t lsb_gradeValue = buffer[9];
        uint8_t msb_gradeValue = buffer[10];
        long gradeValue = lsb_gradeValue + msb_gradeValue * 256;
        float roadGrade = float((gradeValue - 20000)) / 100;
#ifdef DEBUG_FEC_PACKETS
        DEBUG_PRINTF("Page: %02d (0x%02X) Requested Page 51 (0x33) Data Received - gradeValue: %05d  ", PageValue, PageValue, gradeValue);
        DEBUG_PRINTF("Grade percentage: [%05.1f]%%", roadGrade);
        DEBUG_PRINTLN();
#endif
        SetNewRawGradeValue(roadGrade);
        SetNewActuatorPosition();
        // Send position with the new settings
        if (Smartphone.IsConnected) {  // Update Phone display data changed!
          server_NUS_Txd_GRD();
        }
      }
      break;
    case 0x19:
      {
        /////////////////////////////////////////////////
        /////////// Handle PAGE 25 Trainer/Bike Data ////
        /////////////////////////////////////////////////
        uint8_t UpdateEventCount = buffer[5];
        InstantaneousCadence = buffer[6];
        uint8_t lsb_InstantaneousPower = buffer[9];
        // POWER is stored in 1.5 byte !!!
        uint8_t msb_InstantaneousPower = (buffer[10] & 0x0F);  // bits 0:3 --> MSNibble only!!!
        PowerValue = lsb_InstantaneousPower + msb_InstantaneousPower * 256;
#ifdef DEBUG_FEC_PACKETS
        DEBUG_PRINTF("Page: %02d (0x%02X) Bike Data - Event count: [%03d]", PageValue, PageValue, UpdateEventCount);
        DEBUG_PRINTF(" - Cadence: [%03d]", InstantaneousCadence);
        DEBUG_PRINTF(" - Power in Watts: [%04d]", PowerValue);
        DEBUG_PRINTLN();
#endif
        if (Smartphone.IsConnected) {
          server_NUS_Txd_PWR_CAD();
        }
      }
      break;
    case 0x10:
      {
        //////////////////////////////////////////////
        //////////// Handle PAGE 16 General FE Data //
        //////////////////////////////////////////////
        // buffer[4] -> Data Page Number 16 (0x10)
        // buffer[5] -> Equipment Type Bit Field
        uint8_t ReceivedValue = buffer[6];  // Elapsed Time --> in units of 0.25 seconds --> 256 rollover (every 64 seconds!)
        // process Elapsed Time ... since the start of a workout
        AccumulatedElapsedTime += ReceivedValue - PreviousETValue;
        if (PreviousETValue > ReceivedValue) {  // rollover took place
          AccumulatedElapsedTime += 256;
        }
        PreviousETValue = ReceivedValue;
        ReceivedValue = buffer[7];  // Distance Travelled -> in meters -> 256 rollover (every 256 m!)
        // process distance travelled ... since the start of a workout
        AccumulatedDistanceTravelled += ReceivedValue - PreviousDTValue;
        if (PreviousDTValue > ReceivedValue) {  // rollover took place
          AccumulatedDistanceTravelled += 256;
        }
        PreviousDTValue = ReceivedValue;
        uint8_t lsb_SpeedValue = buffer[8];  // Speed LSB -> in units of 0.001 m/s
        uint8_t msb_SpeedValue = buffer[9];  // Speed MSB -> in units of 0.001 m/s
        // Calculate Instantaneous Speed
        SpeedValue = (((lsb_SpeedValue + msb_SpeedValue * 256)) / 1000) * 3.6;  // in units of 0,001 m/s to km/h (kp/h)
        if ((buffer[11] >> 4) == 3) {                                           // make it the LS Nibble and check for IN USE = moving!
          IsTrainerMoving = true;
        } else {
          IsTrainerMoving = false;
        }
#ifdef DEBUG_FEC_PACKETS
        DEBUG_PRINTF("Page: %02d (0x%02X) General Data - ", PageValue, PageValue); DEBUG_PRINTF(" In Use: [%s]", (IsTrainerMoving ? "true" : "false"));
        DEBUG_PRINTF(" - Elapsed time: [%02dh:%02dm:%02ds]", ((AccumulatedElapsedTime / 4) / 3600), ((AccumulatedElapsedTime / 4) % 3600 / 60), ((AccumulatedElapsedTime / 4) % 60));
        DEBUG_PRINTF(" - Accumulated Distance: [%03.2f] km ", AccumulatedDistanceTravelled / 1000);
        DEBUG_PRINTF(" - Speed: [%.1f] km/h\n", SpeedValue);
#endif
        if (Smartphone.IsConnected) {
          server_NUS_Txd_AET_SPD_TIME();
        }
      }
      break;
    case 0x80:
      {
        // Manufacturer Identification Page
      }
      break;
    default:
      {
#ifdef DEBUG_FEC_PACKETS
        DEBUG_PRINTF("Page: %02d (0x%02X) Undecoded\n", PageValue, PageValue);
#endif
        return;
      }
  }  // Switch

  // Show the actual values of the trainer on the Oled
  if (OledDisplaySelection == 1) {
    ShowValuesOnOled();
  } else {
    ShowSlopeTriangleOnOled();
  }
  //////////////////////// DONE! /////////////////////////
}

bool client_Tacx_FEC_Connect(void) {
  // Obtain a reference to the remote CP service.
  pRemote_Tacx_FEC_Service = pClient_Tacx->getService(UUID_TACX_FEC_PRIMARY_SERVICE);
  if (pRemote_Tacx_FEC_Service == nullptr) {
    DEBUG_PRINTLN("Mandatory Tacx FE-C Service: Not Found!");
    return false;
  }
  DEBUG_PRINTLN("Client_Tacx_FEC_Service: Found!");

  pRemote_Tacx_FEC_Rxd_Chr = pRemote_Tacx_FEC_Service->getCharacteristic(UUID_TACX_FEC_RXD_CHARACTERISTIC);
  if (pRemote_Tacx_FEC_Rxd_Chr == nullptr) {
    DEBUG_PRINTLN("Mandatory Client_Tacx_FEC_Rxd_Chr: Not Found!");
    return false;
  }
  DEBUG_PRINTLN("Client_Tacx_FEC_Rxd_Chr: Found!");
  if (pRemote_Tacx_FEC_Rxd_Chr->canNotify()) {
    pRemote_Tacx_FEC_Rxd_Chr->registerForNotify(Tacx_FEC_Rxd_Notify_Callback, false, true);  // Notifications false
  } else {
    DEBUG_PRINTLN("Mandatory Client_Tacx_FEC_Rxd_Chr: Cannot Notify!");
    return false;
  }

  pRemote_Tacx_FEC_Txd_Chr = pRemote_Tacx_FEC_Service->getCharacteristic(UUID_TACX_FEC_TXD_CHARACTERISTIC);
  if (pRemote_Tacx_FEC_Txd_Chr == nullptr) {
    DEBUG_PRINTLN("Mandatory Client_Tacx_FEC_Rxd_Chr: Not Found!");
    return false;
  }
  DEBUG_PRINTLN("Client_Tacx_FEC_Rxd_Chr: Found!");

  if (!pRemote_Tacx_FEC_Txd_Chr->canWriteNoResponse()) {
    DEBUG_PRINTLN("Mandatory Client_Tacx_FEC_Rxd_Chr: Cannot Write!");
    return false;
  }

  // Now Separately -> Notify Enable
  if (pRemote_Tacx_FEC_Rxd_Chr != nullptr) {
    pRemote_Tacx_FEC_Rxd_Chr->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
    DEBUG_PRINTLN("Client_Tacx_FEC_Rxd_Chr: Notify Enabled!");
  }
  return true;
}

// This is NOT really a Callback --> It should have been implemented that way (see for instance Adafruit Bluefruit BLE library),
// however, now it is called from loop() ... a poor man's solution!
bool client_Connect_Callback() {
  // Connect to the Tacx BLE Server.
  pClient_Tacx->connect(myDevice);  // if you pass BLEAdvertisedDevice it will recognize type of peer device address (public or private)
  DEBUG_PRINTLN("Now checking all Client Services and Characteristics!");
  DEBUG_PRINTLN("If Mandatory Services Fail --> the Client will disconnect!");
  // Discover all relevant Services and Char's
  if (!client_GenericAccess_Connect()) {
    pClient_Tacx->disconnect();
    return false;
  }
  if (!client_DeviceInformation_Connect()) {
    pClient_Tacx->disconnect();
    return false;
  }
  if (!client_Tacx_FEC_Connect()) {
    pClient_Tacx->disconnect();
    return false;
  }

  Trainer.IsConnected = true;
  sendRequestPage51Event = millis() + REQUEST_PAGE_51_DELAY;
  return true;
}

/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice* advertisedDevice) {
    DEBUG_PRINT("Advertising Device-> ");
    DEBUG_PRINTLN(advertisedDevice->toString().c_str());
    // We have found a server device, now see if it contains the service we are looking for.
    if (advertisedDevice->haveServiceUUID() && advertisedDevice->isAdvertisingService(UUID_TACX_FEC_PRIMARY_SERVICE)) {
      NimBLEAddress MyAddress = advertisedDevice->getAddress();
      memcpy(&Trainer.PeerAddress, MyAddress.getNative(), 6);
      DEBUG_PRINTLN("Found advertising Peripheral with FE-C enabled! See data:");
      DEBUG_PRINTLN(advertisedDevice->toString().c_str());
      BLEDevice::getScan()->stop();
      myDevice = advertisedDevice;
      /* Connect to the Tacx BLE Server -> Sorry you can't do that here!!! --------------------------------
      ** pClient_Tacx->connect(myDevice);  NOT ALLOWED TO CALL CONNECT --> CAUSES FATAL ERROR !!!! ???? */
      doClientConnectCall = true;  // Work around via loop()
    }                              // Found our server
  }                                // onResult
};                                 // MyAdvertisedDeviceCallbacks


void client_Start_Scanning(void) {
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for ## seconds.
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  DEBUG_PRINTLN("Client Starts Scanning for Trainer Device with FE-C!");
  //pBLEScan->start(5, false); // Scan for 5 seconds only
  pBLEScan->start(0, false);
}

void server_startADV(void) {
  // Prepare for advertising
  /** Optional: set the transmit power, default is 3db */
#ifdef ESP_PLATFORM
  NimBLEDevice::setPower(ESP_PWR_LVL_P9); /** +9db */
#else
  NimBLEDevice::setPower(9); /** +9db */
#endif
  pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(UUID_NUS_SERVICE);
  pAdvertising->setAppearance(server_GA_Appearance_Value);
  DEBUG_PRINTF("Setting Appearance in Advertised data to [%d]\n", server_GA_Appearance_Value);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinInterval(144);  // 32 in 0.625ms units, 0 = use default.
  pAdvertising->setMaxInterval(244);  // 244 in 0.625ms units, 0 = use default.
  BLEDevice::startAdvertising();
  // Start Advertising
}

void server_Connection_Callbacks::onConnect(BLEServer* pServer, ble_gap_conn_desc* desc) {
  // Get some connection parameters of the peer device.
  uint16_t serverConnectionHandle = desc->conn_handle;
  uint16_t serverConnectionInterval = desc->conn_itvl;              // Connection interval
  uint16_t serverConnectionLatency = desc->conn_latency;            // Connection latency
  uint16_t serverConnectionSupTimeout = desc->supervision_timeout;  // Connection supervision timeout
  uint8_t RemoteAddress[6];
  memcpy(&RemoteAddress, NimBLEAddress(desc->peer_id_addr).getNative(), 6);
  char fullMacAddress[18] = {};
  ConvertMacAddress(fullMacAddress, RemoteAddress, false);  // true -> Native format!
  BLEDevice::stopAdvertising();
  DEBUG_PRINTF("Server Connection Parameters -> Interval: [%d] Latency: [%d] Supervision Timeout: [%d]\n", serverConnectionInterval,
               serverConnectionLatency, serverConnectionSupTimeout);
  DEBUG_PRINTF("ESP32 Server connected to Client device with MAC Address: [%s] Conn Handle: [%d]\n", fullMacAddress, serverConnectionHandle);
  /** We can use the connection handle here to ask for different connection parameters.
     *  Args: connection handle, min connection interval, max connection interval, latency and supervision timeout.
     *  Units; Min/Max Intervals: 1.25 millisecond increments --> between: 24 (30 ms) resp. 48 (60 ms)
     *  Latency: number of intervals allowed to skip. --> keep zero
     *  Timeout: 10 millisecond increments. --> Try 400 (4000 ms)
     */
  //pServer->updateConnParams(serverConnectionHandle, 24, 48, 0, 400);
  //DEBUG_PRINTLN("Server Updates Connection Parameters -> Min Interval: [24] Max Interval: [48] Latency: [0] Supervision Timeout: [400]");
  // Smartphone is connecting
  Smartphone.conn_handle = serverConnectionHandle;
  Smartphone.IsConnected = true;
  memcpy(Smartphone.PeerAddress, RemoteAddress, 6);
  DEBUG_PRINTF("Central (%s/Simcline App) has to set NUS CCCD 'Notify' (enable) and start....\n", Smartphone.PeerName.c_str());
};

void server_Connection_Callbacks::onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc) {
  DEBUG_PRINTF("Central (%s) updated MTU to: [%u] for connection ID: %u\n", Smartphone.PeerName.c_str(), MTU, desc->conn_handle);
};

void server_Connection_Callbacks::onDisconnect(BLEServer* pServer, ble_gap_conn_desc* desc) {
  uint32_t count = pServer->getConnectedCount();
  // Get some Disconnection parameters of the peer device.
  uint16_t serverConnectionHandle = desc->conn_handle;
  uint8_t RemoteAddress[6] = {};
  memcpy(&RemoteAddress, NimBLEAddress(desc->peer_id_addr).getNative(), 6);
  char fullMacAddress[18] = {};                             //
  ConvertMacAddress(fullMacAddress, RemoteAddress, false);  // true -> Native format!
  if (Smartphone.conn_handle == serverConnectionHandle) {   // Smartphone is disconnected
    Smartphone.conn_handle = BLE_HS_CONN_HANDLE_NONE;
    Smartphone.IsConnected = false;
    DEBUG_PRINTF("ESP32 Server disconnected from Central (%s) Conn handle: [%d] Mac Address: [%s]\n", Smartphone.PeerName.c_str(), serverConnectionHandle, fullMacAddress);
    // Show the message on the Oled
    ShowOnOledLarge("Phone", "Connection", "Lost!", 500);
  }
  DEBUG_PRINTLN(" --> ESP32 Server is advertising again!");
  // NimBLe does auto advertise after disconnect
};

class server_NUS_Rxd_Chr_callback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    // Read data received over NUS Rxd from Mobile Phone
    std::string NusRxdData = server_NUS_Rxd_Chr->getValue();
    uint8_t NusRxdDataLen = NusRxdData.length();  // Get the actual length of data bytes
    // Display the raw packet data in actual length
    DEBUG_PRINTF("-> server NUS Rxd Rec'd Data [%d][%s]\n", NusRxdDataLen, NusRxdData.c_str());
    // The following routines parse and process the incoming commands
    // Every NusRxdData packet starts with a '!' otherwise corrupt/invalid
    if (NusRxdData[0] != '!') {
      DEBUG_PRINTLN(F("-> Error: RXD-packet does not start with a '!'"));
      return;  // invalid NusRxdData packet: do not further parse and process
    }
    // RXpacket buffer has IdCode = "S"
    if (NusRxdData[1] == 'S') {  // Settings packet
      // Besides what is mechanically possible there are also limits in what is physically pleasant/comfortable
      // The allowed Raw Grade Value min and max values should be within the limits of the mechanically feasible values !!!
      // Minimally allowed Raw Grade Value that should not be exceeded: -5%!
      // default: aRGVmin is default set to 19500
      // Maximally allowed Raw Grade Value that should not be exceeded: 15%!
      // default: aRGVmax is default set to: 21500
      // New Settings values have arrived --> parse, set values and store persistently
      uint8_t iMax = 0, iMin = 0, iPerc = 0, iDispl = 0;
      sscanf((char*)NusRxdData.c_str(), "!S%d;%d;%d;%d;", &iMax, &iMin, &iPerc, &iDispl);
      // set Ascent Grade Limit to aRGVmax
      iMax = constrain(iMax, 0, RGVMAX_GRADE);
      aRGVmax = map(iMax, 0, RGVMAX_GRADE, 20000, RGVMAX);
      // set Descent Grade Limit to aRGVmin
      iMin = constrain(iMin, 0, RGVMIN_GRADE);  // Notice: positive value!
      aRGVmin = map(iMin, RGVMIN_GRADE, 0, RGVMIN, 20000);
      // set Road Grade Change Factor
      GradeChangeFactor = iPerc;
      // set OledDisplaySelection
      OledDisplaySelection = iDispl;
      // LittleFS for persistent storage of these values
      setPRSdata();
      // LittleFS --------------------------------------
      DEBUG_PRINT(F(" Settings: Max: "));
      DEBUG_PRINT(iMax);
      DEBUG_PRINT(F(" Min: "));
      DEBUG_PRINT(iMin);
      DEBUG_PRINT(F(" Perc: "));
      DEBUG_PRINT(iPerc);
      DEBUG_PRINT(F(" Displ: "));
      DEBUG_PRINTLN(iDispl);
      // Confirm to the PHONE: settings rcvd and set to persistent
#ifdef DEBUG_NUS_TXD
      DEBUG_PRINTLN("-> server_NUS_Txd sends Confirm message: Done!");
#endif
      server_NUS_Txd_Chr->setValue("!SDone!;");
      server_NUS_Txd_Chr->notify();
      return;  // Settings rcvd and set to persistent
    }
    // Manual Control Buttons Up Down get parsed and processed!
    // ONLY when the Actuator plus sensor are working well!
    // i.e. low level up/down movement functions work !!
    if (NusRxdData[1] == 'U' && IsBasicMotorFunctions) {
      DEBUG_PRINTLN("-> Set motor UPward moving!");
      RawgradeValue = RawgradeValue + 100;
      SetManualGradePercentValue();
      ShowSlopeTriangleOnOled();
      return;
    }
    if (NusRxdData[1] == 'D' && IsBasicMotorFunctions) {
      DEBUG_PRINTLN("-> Set motor DOWNward moving!");
      RawgradeValue = RawgradeValue - 100;
      SetManualGradePercentValue();
      ShowSlopeTriangleOnOled();
      return;
    }
    server_NUS_Txd_Chr->setValue("!UOut of Order!;");
    server_NUS_Txd_Chr->notify();
#ifdef DEBUG_NUS_TXD
    DEBUG_PRINTLN("-> server_NUS_Txd sends Error message: Out of Order!");
#endif
  };  // onWrite
};

void server_NUS_Txd_Persistent_Settings(void) {
  // Send persistent stored values to Mobile Phone for correct Settings!
  // recalculate the values for use on the Phone
  uint8_t TXpacketBuffer[16] = { 0 };
  int iMax, iMin, iPerc, iDispl;
  // set within limits
  aRGVmax = constrain(aRGVmax, 0, RGVMAX);
  aRGVmin = constrain(aRGVmin, 0, RGVMIN);
  // set aRGVmax to Ascent Grade Limit in whole number
  iMax = map(aRGVmax, 20000, RGVMAX, 0, RGVMAX_GRADE);
  // set aRGVmin to Descent Grade Limit in whole number
  iMin = map(aRGVmin, RGVMIN, 20000, RGVMIN_GRADE, 0);
  // set GradeChangeFactor to Road Grade Change Factor
  iPerc = GradeChangeFactor;
  iDispl = OledDisplaySelection;
  sprintf((char*)TXpacketBuffer, "!S%d;%d;%d;%d;", iMax, iMin, iPerc, iDispl);
  // send these persistent data to the Settings page on the smartphone
  server_NUS_Txd_Chr->notify(TXpacketBuffer, sizeof(TXpacketBuffer));
#ifdef DEBUG_NUS_TXD
  DEBUG_PRINTF("-> server_NUS_Txd sends Persistent settings: [%s]", (char*)TXpacketBuffer);
#endif
}

// Handler class for Server NUS Txd Characteristic actions limited to onSubscribe
class server_NUS_Txd_Callback : public NimBLECharacteristicCallbacks {
  /*  We only define onSubscribe !!!
    void onRead(NimBLECharacteristic* pCharacteristic);
    void onWrite(NimBLECharacteristic* pCharacteristic);
    void onNotify(NimBLECharacteristic* pCharacteristic);    
    void onStatus(NimBLECharacteristic* pCharacteristic, Status status, int code);
*/
  void onSubscribe(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue) {
    String str = "Central Updated CCCD -->";
    if (subValue == 0) {
      str += " Notify/Indicate Disabled for Char:";
    } else if (subValue == 1) {
      str += " Notify Enabled for Char:";
    } else if (subValue == 2) {
      str += " Indicate Enabled for Char:";
    } else if (subValue == 3) {
      str += " Notify & Indicate Enabled for Char:";
    }
    DEBUG_PRINTF("%s", str.c_str());
    str = std::string(pCharacteristic->getUUID()).c_str();
    DEBUG_PRINTF(" [%s]\n", str.c_str());

    if (subValue == 1) {
      server_NUS_Txd_Persistent_Settings();
    }
  };
};

void server_setupGA(void) {
  // Set the Server Generic Access Appearance value from default: [0] --> Unknown to [128] --> Generic Computer
  int RespErr = ble_svc_gap_device_appearance_set(server_GA_Appearance_Value);
  if (RespErr == 0) {
    DEBUG_PRINTF("Successfully Set Server Generic Access Appearance Chr value to:  [%d] Generic Computer\n", server_GA_Appearance_Value);
  } else {
    DEBUG_PRINTLN("Unable to Set Server Generic Access Appearance Chr value!");
  }
  // Set Generic Access Device Name Chr to a value
  RespErr = ble_svc_gap_device_name_set((const char*)server_GA_DeviceName_Str.c_str());
  if (RespErr == 0) {
    DEBUG_PRINTF("Successfully Set Server Generic Access Device Name Chr value to: [%s]\n", server_GA_DeviceName_Str.c_str());
  } else {
    DEBUG_PRINTLN("Unable to Set Server Generic Access Device Name Chr value!");
  }
}

void server_setupNUS(void) {
  server_NordicUart_Service = pServer->createService(UUID_NUS_SERVICE);
  server_NUS_Rxd_Chr = server_NordicUart_Service->createCharacteristic(UUID_NUS_CHR_RXD,
                                                                       NIMBLE_PROPERTY::WRITE_NR);  // Write with No response !!
  server_NUS_Rxd_Chr->setCallbacks(new server_NUS_Rxd_Chr_callback());
  server_NUS_Txd_Chr = server_NordicUart_Service->createCharacteristic(UUID_NUS_CHR_TXD,
                                                                       NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  server_NUS_Txd_Chr->setCallbacks(new server_NUS_Txd_Callback());  //NIMBLE
  server_NordicUart_Service->start();
}

void SendRequestPage51(void) {
  DEBUG_PRINT("Send Common Page [70] (0x46) with Request for Data Page [51] (0x33)");
  // Page51Bytes are globally defined
  // const uint8_t* data, size_t length, bool response = false
  if ( !pRemote_Tacx_FEC_Txd_Chr->writeValue(Page51Bytes, sizeof(Page51Bytes), false) ) {
    DEBUG_PRINT(" -> ERROR: not connected or otherwise cannot perform write");
  }
  DEBUG_PRINTLN();
}

void loop() {  // loop() is used to start sort of Callback functions
  if (Trainer.IsConnected) {
    // If time is there, send Request Page 51 to the Tacx trainer...
    if (millis() > sendRequestPage51Event) {
      SendRequestPage51();
      sendRequestPage51Event = millis() + REQUEST_PAGE_51_DELAY;
    }  // sendRequestPage51Event
  }
  // If the flag "doClientConnectCall" is true, we connect to the BLE server!
  if (doClientConnectCall) {
    doClientConnectCall = false;
    bool dummy = client_Connect_Callback();
  }
  // If the flag "RestartScanningOnDisconnect" is true, we start scanning for a (new) BLE Server!
  if (RestartScanningOnDisconnect) {
    pBLEScan->clearResults();  // delete results from BLEScan buffer to release memory
    DEBUG_PRINTLN("Client Starts Scanning again for Trainer Device with FE-C");
    RestartScanningOnDisconnect = false;
    pBLEScan->start(0, false);
  }
  delay(50);  // DO NOT REMOVE or Task watchdog will be triggered!!!
} // End of loop
