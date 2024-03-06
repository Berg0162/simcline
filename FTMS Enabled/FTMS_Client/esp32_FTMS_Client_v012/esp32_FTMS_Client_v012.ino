/*********************************************************************
 This is programming code for ESP32 Espressif Wroom boards
 
      Tested with Adafruit Feather ESP32 V2 a.k.a. Huzzah
 
 The code uses heavily the supplied ESP32 NimBLE libraries !!          
      see: https://github.com/h2zero/NimBLE-Arduino
 Many have invested time and resources providing open source code!
 
        MIT license, check LICENSE for more information
        All text must be included in any redistribution
*********************************************************************/

/* 
 *  This Adafruit Feather ESP32 V2 code scans for the CPS, CSC and FTMS
 *  that the trainer is advertising, it tries to connect and then 
 *  enables .....
 *  
 *  Requirements: FTMS trainer and an ESP32 board
 *  1) Upload and Run this code on ESP32 board
 *  2) Start the Serial Monitor to catch verbose debugging and data info
 *  3) Power ON/Wake UP trainer -> do NOT connect with other devices
 *  4) Trainer and Feather should successfully pair or disconnect...
 *  5) Keep the Serial Monitor visible on top of all windows 
 *  6) Move the trainer pedals and notice/feel changes in resistance...
 *     The Client sends Resistance Parameters to the Trainer that coincide 
 *     with the first 5 minutes of the Zwift Volcano Circuit!
 *  7) Inspect the info presented by Serial Monitor.....
 *  
 */

/*
Version 1.1 
Cycling Speed Cadence Service changed to NOT Mandatory
Version 1.2
Inserted checks on the input values of sensor location and location description array sizes (CP, CSC and HBM)
*/

// -------------------------------------------------------------------------------------------
// COMPILER DIRECTIVE to allow/suppress DEBUG messages that help debugging...
// Uncomment general "#define DEBUG" to activate
#define DEBUG
// Include these debug utility macros in all cases!
#include "DebugUtils.h"
//
#ifdef DEBUG
#define DEBUG_IBD               // If defined allows for parsing the Indoor Bike Data
//#define DEBUG_HBM             // If defined allows for parsing the Heart Beat Data
//#define DEBUG_CP_MEASUREMENT  // If defined allows for presentation of Cycling Power Data
//#define DEBUG_CSC_MEASUREMENT // If defined allows for presentation of Cadence & Speed Data
#endif
// --------------------------------------------------------------------------------------------

#include <NimBLEDevice.h>

// Client Generic Access --------------------------------------------------------------
#define UUID16_SVC_GENERIC_ACCESS                             BLEUUID((uint16_t)0x1800)
#define UUID16_CHR_DEVICE_NAME                                BLEUUID((uint16_t)0x2A00)
#define UUID16_CHR_APPEARANCE                                 BLEUUID((uint16_t)0x2A01)
#define UUID16_CHR_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS BLEUUID((uint16_t)0x2A04)
#define UUID16_CHR_CENTRAL_ADDRESS_RESOLUTION                 BLEUUID((uint16_t)0x2AA6)
static BLERemoteService* pRemote_GenericAccess_Service;
static BLERemoteCharacteristic* pRemote_GA_Appearance_Chr; // Read
uint16_t client_GA_Appearance_Value = 0;
static BLERemoteCharacteristic* pRemote_GA_DeviceName_Chr;// Read, Write
std::string client_GA_DeviceName_Str;

// Client Service Device Information --------------------------------------------------
#define UUID16_SVC_DEVICE_INFORMATION                         BLEUUID((uint16_t)0x180A)
#define UUID16_CHR_MODEL_NUMBER_STRING                        BLEUUID((uint16_t)0x2A24)
#define UUID16_CHR_SERIAL_NUMBER_STRING                       BLEUUID((uint16_t)0x2A25)
#define UUID16_CHR_FIRMWARE_REVISION_STRING                   BLEUUID((uint16_t)0x2A26)
#define UUID16_CHR_HARDWARE_REVISION_STRING                   BLEUUID((uint16_t)0x2A27)
#define UUID16_CHR_SOFTWARE_REVISION_STRING                   BLEUUID((uint16_t)0x2A28)
#define UUID16_CHR_MANUFACTURER_NAME_STRING                   BLEUUID((uint16_t)0x2A29)
static BLERemoteService*  pRemote_DeviceInformation_Service; 
static BLERemoteCharacteristic* pRemote_DIS_ManufacturerName_Chr;   // Read
std::string client_DIS_Manufacturer_Str;
static BLERemoteCharacteristic* pRemote_DIS_ModelNumber_Chr;       // Read
std::string client_DIS_ModelNumber_Str;
static BLERemoteCharacteristic* pRemote_DIS_SerialNumber_Chr;      // Read
std::string client_DIS_SerialNumber_Str;
// -------------------------------------------------------------------------------------

#define MAXPAYLOAD 20

/* Cycling Power Service ---------------------------------------------------------------
 * CP Service: 0x1818  
 * CP Characteristic: 0x2A63 (Measurement)    Mandatory
 * CP Characteristic: 0x2A65 (Feature)        Mandatory
 * CP Characteristic: 0x2A5D (Location)       Optional
 * CP Characteristic: 0x2A66 (Control Point)  Optional
 */
#define UUID16_SVC_CYCLING_POWER                BLEUUID((uint16_t)0x1818)
#define UUID16_CHR_CYCLING_POWER_MEASUREMENT    BLEUUID((uint16_t)0x2A63)
#define UUID16_CHR_CYCLING_POWER_FEATURE        BLEUUID((uint16_t)0x2A65)
#define UUID16_CHR_SENSOR_LOCATION              BLEUUID((uint16_t)0x2A5D)
#define UUID16_CHR_CYCLING_POWER_CONTROL_POINT  BLEUUID((uint16_t)0x2A66)

static BLERemoteService*        pRemote_CyclingPower_Service;
static BLERemoteCharacteristic* pRemote_CP_Measurement_Chr;    // Notify, Read
static BLERemoteCharacteristic* pRemote_CP_Feature_Chr;        // Read
uint32_t client_CP_Feature_Flags = 0;
static BLERemoteCharacteristic* pRemote_CP_Location_Chr;       // Read
uint8_t client_CP_Location_Value = 0; // UINT8
static BLERemoteCharacteristic* pRemote_CP_ControlPoint_Chr;   // Indicate, Write

const uint8_t client_CP_Feature_Len = 20; // Num. of Feature elements
const char* client_CP_Feature_Str[client_CP_Feature_Len] = { 
      "Pedal power balance supported",
      "Accumulated torque supported",
      "Wheel revolution data supported",
      "Crank revolution data supported",
      "Extreme magnitudes supported",
      "Extreme angles supported",
      "Top/bottom dead angle supported",
      "Accumulated energy supported",
      "Offset compensation indicator supported",
      "Offset compensation supported",
      "Cycling power measurement characteristic content masking supported",
      "Multiple sensor locations supported",
      "Crank length adj. supported",
      "Chain length adj. supported",
      "Chain weight adj. supported",
      "Span length adj. supported",
      "Sensor measurement context",
      "Instantaineous measurement direction supported",
      "Factory calibrated date supported",
      "Enhanced offset compensation supported" };

const uint8_t client_Sensor_Location_Str_Len = 17;      
const char* client_Sensor_Location_Str[client_Sensor_Location_Str_Len] = { "Other", "Top of shoe", "In shoe", "Hip", 
    "Front wheel", "Left crank", "Right crank", "Left pedal", "Right pedal", "Front hub", 
    "Rear dropout", "Chainstay", "Rear wheel", "Rear hub", "Chest", "Spider", "Chain ring"};

/*---------------------------------------------------------------------------------------
 * Cycling Speed and Cadence Service
 * CSC Service:   0x1816 
 * CSC Measurement Characteristic:  0x2A5B
 * CSC Feature Characteristic:      0x2A5C  
 */
#define UUID16_SVC_CYCLING_SPEED_AND_CADENCE  BLEUUID((uint16_t)0x1816)
#define UUID16_CHR_CSC_MEASUREMENT            BLEUUID((uint16_t)0x2A5B)
#define UUID16_CHR_CSC_FEATURE                BLEUUID((uint16_t)0x2A5C)
// PM   UUID16_CHR_SENSOR_LOCATION shared with Cycling Power Service
static BLERemoteService*        pRemote_CyclingSpeedCadence_Service;
static BLERemoteCharacteristic* pRemote_CSC_Measurement_Chr;         // Notify, Read
static BLERemoteCharacteristic* pRemote_CSC_Feature_Chr;             // Read
const uint8_t CSC_FEATURE_FIXED_DATALEN = 2; // UINT16
uint16_t client_CSC_Feature_Flags = 0;
static BLERemoteCharacteristic* pRemote_CSC_Location_Chr;             // Read
uint8_t client_CSC_Location_Value = 0; 

/* CSC Control Point Characteristic:0x2A55 --->  not implemented
// CSC Control Point op codes 
#define     SC_CP_OP_SET_CUMULATIVE_VALUE           1
#define     SC_CP_OP_START_SENSOR_CALIBRATION       2
#define     SC_CP_OP_UPDATE_SENSOR_LOCATION         3
#define     SC_CP_OP_REQ_SUPPORTED_SENSOR_LOCATIONS 4
#define     SC_CP_OP_RESPONSE                       16
// CSC Control Point response values
#define     SC_CP_RESPONSE_SUCCESS                  1
#define     SC_CP_RESPONSE_OP_NOT_SUPPORTED         2
#define     SC_CP_RESPONSE_INVALID_PARAM            3
#define     SC_CP_RESPONSE_OP_FAILED                4
*/

/*CSC Measurement flags*/
#define     CSC_MEASUREMENT_WHEEL_REV_PRESENT       0x01
#define     CSC_MEASUREMENT_CRANK_REV_PRESENT       0x02

/* CSC Feature flags */
#define     CSC_FEATURE_WHEEL_REV_DATA              0x01
#define     CSC_FEATURE_CRANK_REV_DATA              0x02
#define     CSC_FEATURE_MULTIPLE_SENSOR_LOC         0x04

const uint8_t client_CSC_Feature_Len = 3;
const char* client_CSC_Feature_Str[client_CSC_Feature_Len] = {"Wheel rev supported", "Crank rev supported", "Multiple locations supported"};

/* HRM Service Definitions -------------------------------------------------------------
 * Heart Rate Monitor Service:  0x180D
 * Heart Rate Measurement Char: 0x2A37 (Mandatory)
 * Body Sensor Location Char:   0x2A38 (Optional)
 */
#define UUID16_SVC_HEART_RATE             BLEUUID((uint16_t)0x180D)
#define UUID16_CHR_HEART_RATE_MEASUREMENT BLEUUID((uint16_t)0x2A37)
#define UUID16_CHR_BODY_SENSOR_LOCATION   BLEUUID((uint16_t)0x2A38)
static BLERemoteService* pRemote_HeartRate_Service;
static BLERemoteCharacteristic* pRemote_HR_Measurement_Chr;
static BLERemoteCharacteristic* pRemote_HR_Location_Chr;
static uint8_t client_HR_Location_Value = 0;
// --------------------------------------------------------------------------------------

/* --------------------------------------------------------------------------------------
 * Fitness Machine Service, uuid 0x1826 or 00001826-0000-1000-8000-00805F9B34FB
*/
#define UUID16_SVC_FITNESS_MACHINE                            BLEUUID((uint16_t)0x1826)
#define UUID16_CHR_FITNESS_MACHINE_FEATURE                    BLEUUID((uint16_t)0x2ACC)
#define UUID16_CHR_INDOOR_BIKE_DATA                           BLEUUID((uint16_t)0x2AD2)
#define UUID16_CHR_TRAINING_STATUS                            BLEUUID((uint16_t)0x2AD3)
//#define UUID16_CHR_SUPPORTED_SPEED_RANGE                      BLEUUID((uint16_t)0x2AD4)
//#define UUID16_CHR_SUPPORTED_INCLINATION_RANGE                BLEUUID((uint16_t)0x2AD5)
#define UUID16_CHR_SUPPORTED_RESISTANCE_LEVEL_RANGE           BLEUUID((uint16_t)0x2AD6)
//#define UUID16_CHR_SUPPORTED_HEART_RATE_RANGE                 BLEUUID((uint16_t)0x2AD7)
#define UUID16_CHR_SUPPORTED_POWER_RANGE                      BLEUUID((uint16_t)0x2AD8)
#define UUID16_CHR_FITNESS_MACHINE_CONTROL_POINT              BLEUUID((uint16_t)0x2AD9)
#define UUID16_CHR_FITNESS_MACHINE_STATUS                     BLEUUID((uint16_t)0x2ADA)

static BLERemoteService* pRemote_FitnessMachine_Service; // FTM Service
// Service characteristics exposed by FTM Service
BLERemoteCharacteristic* pRemote_FTM_SupportedResistanceLevelRange_Chr; // Supported Resistance Level, read, optional
//const uint8_t FTM_SRLR_FIXED_DATALEN = 6;
std::string client_FTM_SupportedResistanceLevelRange_Str;
BLERemoteCharacteristic* pRemote_FTM_SupportedPowerRange_Chr;  // Supported Power Levels, read, optional
//const uint8_t FTM_SPR_FIXED_DATALEN = 6;
std::string client_FTM_SupportedPowerRange_Str;
BLERemoteCharacteristic* pRemote_FTM_Feature_Chr; //  Fitness Machine Feature, mandatory, read
//const uint8_t FTM_FEATURE_FIXED_DATALEN = 8;
std::string client_FTM_Feature_Str;
#ifdef DEBUG
// ---------------------Fitness Machine Features (bytes 1-4):
const uint8_t client_FTM_Feature_Str_One_Len = 32;
const char* client_FTM_Feature_Str_One[client_FTM_Feature_Str_One_Len] = {
"Average Speed Supported", "Cadence Supported", "Total Distance Supported", "Inclination Supported",
"Elevation Gain Supported", "Pace Supported", "Step Count Supported", "Resistance Level Supported",
"Stride Count Supported", "Expended Energy Supported", "Heart Rate Measurement Supported", "Metabolic Equivalent Supported",
"Elapsed Time Supported", "Remaining Time Supported", "Power Measurement Supported", "Force on Belt and Power Output Supported",
"User Data Retention Supported", "Reserved for Future Use","","","","","","","","","","","","","",""};
// ---------------------Fitness Machine Target Setting Features (bytes 5-8) :
const uint8_t client_FTM_Feature_Str_Two_Len = 32;
const char* client_FTM_Feature_Str_Two[client_FTM_Feature_Str_Two_Len] = {
"Speed Target Setting Supported", "Inclination Target Setting Supported", "Resistance Target Setting Supported", "Power Target Setting Supported",
"Heart Rate Target Setting Supported", "Targeted Expended Energy Configuration Supported", "Targeted Step Number Configuration Supported",
"Targeted Stride Number Configuration Supported", "Targeted Distance Configuration Supported", "Targeted Training Time Configuration Supported",
"Targeted Time in Two Heart Rate Zones Configuration Supported", "Targeted Time in Three Heart Rate Zones Configuration Supported",
"Targeted Time in Five Heart Rate Zones Configuration Supported", "Indoor Bike Simulation Parameters Supported", "Wheel Circumference Configuration Supported",
"Spin Down Control Supported", "Targeted Cadence Configuration Supported", "Reserved for Future Use","","","","","","","","","","","","","",""};
#endif
BLERemoteCharacteristic* pRemote_FTM_TrainingStatus_Chr; //  Training Status, optional, read & notify
BLERemoteCharacteristic* pRemote_FTM_Status_Chr; //  Fitness Machine Status, mandatory, notify
BLERemoteCharacteristic* pRemote_FTM_IndoorBikeData_Chr; //  Indoor Bike Data, optional, notify
BLERemoteCharacteristic* pRemote_FTM_ControlPoint_Chr; //  Fitness Machine Control Point, optional, write & indicate

// --------------------------------------------------------------------------------------
static BLEClient* pClient_FTMS;
static BLEAdvertisedDevice* myDevice;
static BLEScan* pBLEScan;

static boolean doClientConnectCall = false;
static boolean clientIsConnected = false;
static uint8_t clientPeerAddress[6] = {};
static std::string clientPeerName;
static boolean RestartScanningOnDisconnect = false;

// Values used to enable or disable notifications/indications
const uint8_t notificationOff[] = {0x0, 0x0};
const uint8_t notificationOn[] = {0x1, 0x0};
const uint8_t indicationOff[] = {0x0, 0x0};
const uint8_t indicationOn[] = {0x2, 0x0};
// ---------------------------------------------------------------------------------------
#define CONTROL_POINT_TIME_SPAN 2000    // Time span for sending Control Point data
unsigned long TimeInterval = 0;
// TEST DATA set -----------------------------------------------------
// Test set (Zwift Volcano Circuit) with Control and 2D Resistance data to be sent to Control Point
uint8_t ControlPointMessageCount = 0;
const uint8_t ControlPointData[32][8] = {
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // Request Control
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // Request Control
{0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // Start or Resume
{0x11,0x00,0x00,0x22,0x01,0x28,0x33,0x00}, // Set Indoor Bike Simulation Parameters..
{0x11,0x00,0x00,0x09,0x00,0x28,0x33,0x00},
{0x11,0x00,0x00,0x00,0x00,0x28,0x33,0x00},
{0x11,0x00,0x00,0x14,0x00,0x28,0x33,0x00},
{0x11,0x00,0x00,0x22,0x00,0x28,0x33,0x00},
{0x11,0x00,0x00,0x25,0x00,0x28,0x33,0x00},
{0x11,0x00,0x00,0x23,0x00,0x28,0x33,0x00},
{0x11,0x00,0x00,0x1C,0x00,0x28,0x33,0x00},
{0x11,0x00,0x00,0x0F,0x00,0x28,0x33,0x00},
{0x11,0x00,0x00,0x00,0x00,0x28,0x33,0x00},
{0x11,0x00,0x00,0xF5,0xFF,0x28,0x33,0x00},
{0x11,0x00,0x00,0xE4,0xFF,0x28,0x33,0x00},
{0x11,0x00,0x00,0xCF,0xFF,0x28,0x33,0x00},
{0x11,0x00,0x00,0xA8,0xFF,0x28,0x33,0x00},
{0x11,0x00,0x00,0x84,0xFF,0x28,0x33,0x00},
{0x11,0x00,0x00,0x67,0xFF,0x28,0x33,0x00},
{0x11,0x00,0x00,0x5D,0xFF,0x28,0x33,0x00},
{0x11,0x00,0x00,0x5E,0xFF,0x28,0x33,0x00},
{0x11,0x00,0x00,0x6F,0xFF,0x28,0x33,0x00},
{0x11,0x00,0x00,0x8A,0xFF,0x28,0x33,0x00},
{0x11,0x00,0x00,0xB8,0xFF,0x28,0x33,0x00},
{0x11,0x00,0x00,0x00,0x00,0x28,0x33,0x00},
{0x11,0x00,0x00,0xA2,0x00,0x28,0x33,0x00},
{0x11,0x00,0x00,0x09,0x01,0x28,0x33,0x00},
{0x11,0x00,0x00,0x4F,0x01,0x28,0x33,0x00},
{0x11,0x00,0x00,0x67,0x01,0x28,0x33,0x00},
{0x11,0x00,0x00,0x57,0xFF,0x28,0x33,0x00},
{0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // Stop or Pause
{0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00}};// Reset
// TEST DATA set ---------------------------------------------------

// Client Connect and Disconnect callbacks defined
class client_Connection_Callbacks : public BLEClientCallbacks {
  void onConnect(BLEClient* pClient);
  void onDisconnect(BLEClient* pClient);
  bool onConnParamsUpdateRequest(BLEClient* pClient, ble_gap_upd_params *params);  
};

void ConvertMacAddress(char *fullAddress, uint8_t addr[6], bool NativeFormat)
{ // Display byte by byte in HEX 
  if(NativeFormat) { // Unaltered: representation
    sprintf(fullAddress, "%02X:%02X:%02X:%02X:%02X:%02X", addr[0], addr[1], addr[2], \
      addr[3], addr[4], addr[5], HEX);   
  } else { // Altered: in reversed order
    sprintf(fullAddress, "%02X:%02X:%02X:%02X:%02X:%02X", addr[5], addr[4], addr[3], \
      addr[2], addr[1], addr[0], HEX);       
  }
}
void client_Start_Scanning(void);

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
  while ( !Serial ) 
      delay(10); 
#endif
  DEBUG_PRINTLN("  ESP32 BLE Client/Central: CPS, CSC, HBM and FTMS");
  DEBUG_PRINTLN("------------------- Version 01.2 --------------------");
  BLEDevice::init("");
  pClient_FTMS  = BLEDevice::createClient();
  pClient_FTMS->setClientCallbacks(new client_Connection_Callbacks());    
  client_Start_Scanning();
} // End of setup.

static void client_HR_Measurement_Notify_Callback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    // Measurement contains of Flags byte, measurement (8 or 16 bit) and optional fields
#ifdef DEBUG_HBM
  uint8_t HRDataLen = (uint8_t)length;
  uint8_t HRDataBuf[HRDataLen] = {};
  // Transfer first the contents of data to buffer (array of chars)
  DEBUG_PRINTF(" -> Client Rec'd Raw Heart Rate Measurement Data: [%d] [%d] [ ", isNotify, length);
  for (int i = 0; i <  sizeof(HRDataBuf); i++) {
      HRDataBuf[i] = *pData++;
      DEBUG_PRINTF("%02X ", HRDataBuf[i], HEX);
  }
  DEBUG_PRINTLN("] ");
  uint8_t offset = 0;
  uint8_t flags = 0;
  memcpy(&flags, &HRDataBuf[offset], 1); // Transfer buffer fields to flags variable
  offset += 1;  // UINT8
  if(flags & 1) { // 16 bit data value is present flag
    uint16_t HRMvalue = 0;
    memcpy(&HRMvalue, &HRDataBuf[offset], 2); // Transfer buffer fields to variable
    DEBUG_PRINTF("Heart Beats: %d HBM (16)", HRMvalue);
    offset += 2;  // UINT16 
  } else { // 8 bit value
    uint8_t HRMvalue = 0;
    memcpy(&HRMvalue, &HRDataBuf[offset], 1); // Transfer buffer fields to variable
    DEBUG_PRINTF("Heart Beats: %d HBM (8)", HRMvalue);
    offset += 1;  // UINT8     
  }
  if(flags & 2) { // sensor
    DEBUG_PRINT(" Contact is detected: ");
    if (flags & 4) { DEBUG_PRINT("ON"); } 
    else { DEBUG_PRINT("OFF"); }
  } else { DEBUG_PRINT(" Contact is NOT detected!"); }
  if(flags & 8) { DEBUG_PRINT(" Expended Energy"); }
  if(flags & 16) { DEBUG_PRINT(" RR interval"); }
  DEBUG_PRINTLN("");
#endif
}

void client_Connection_Callbacks::onConnect(BLEClient* pClient) {
    NimBLEAddress MyAddress = myDevice->getAddress();
    clientPeerName = myDevice->getName().c_str();
    //DEBUG_PRINTF("BLEDevice address in Little Endian order: [%s]\n", MyAddress->getAddress().toString().c_str());
    memcpy(&clientPeerAddress, MyAddress.getNative(), 6); 
#ifdef DEBUG
    DEBUG_PRINT("Client Connection Parameters -> ");
    uint16_t max_payload = pClient_FTMS->getMTU()-3;
    //DEBUG_PRINTF("Max Transmission Unit: [%d] ", max_payload);
    uint16_t clientConnectionInterval = pClient_FTMS->getConnInfo().getConnInterval();
    DEBUG_PRINTF("Interval: [%d] ", clientConnectionInterval);
    uint16_t clientConnectionLatency = pClient_FTMS->getConnInfo().getConnLatency();
    DEBUG_PRINTF("Latency: [%d] ", clientConnectionLatency);
    uint16_t clientConnectionSupTimeout = pClient_FTMS->getConnInfo().getConnTimeout();
    DEBUG_PRINTF("Supervision Timeout: [%d]\n", clientConnectionSupTimeout);
    char fullMacAddress[18] = {}; //
    ConvertMacAddress(fullMacAddress, clientPeerAddress, false); // true -> Native Format
    DEBUG_PRINTF("ESP32 Client connected to Server device with Name: [%s] MAC Address: [%s] MTU: [%d]\n", clientPeerName.c_str(), fullMacAddress, max_payload); 
#endif 
    /** We can use the connection handle here to ask for different connection parameters.
     *  Args: connection handle, min connection interval, max connection interval, latency and supervision timeout.
     *  Units; Min/Max Intervals: 1.25 millisecond increments --> between: 24 (30 ms) resp. 48 (60 ms)
     *  Latency: number of intervals allowed to skip. --> keep zero
     *  Timeout: 10 millisecond increments. --> Try 400 (4000 ms)
     */ 
    //pClient_FTMS->updateConnParams(pClient_FTMS->getConnId(), 24, 48, 0, 400); 
    //DEBUG_PRINTLN("Client Updates Connection Parameters -> Min Interval: [24] Max Interval: [48] Latency: [0] Supervision Timeout: [400]");         
    doClientConnectCall = true;    
  };

bool client_Connection_Callbacks::onConnParamsUpdateRequest(BLEClient* pClient, ble_gap_upd_params *params) {
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
    return true; // That is OK!  
};

void client_Connection_Callbacks::onDisconnect(BLEClient* pClient) {
    clientIsConnected = false;
    char fullMacAddress[18] = {}; //
    ConvertMacAddress(fullMacAddress, clientPeerAddress, false); // true -> Native Format
    DEBUG_PRINTF("Client Disconnected from Server device with Name: [%s] Mac Address: [%s]!\n",  clientPeerName.c_str(), fullMacAddress); 
    RestartScanningOnDisconnect = true;
};

bool client_DeviceInformation_Connect(void)
{
    // If Device Information is not found then go on.... NOT FATAL !
    pRemote_DeviceInformation_Service = pClient_FTMS->getService(UUID16_SVC_DEVICE_INFORMATION);    
    if ( pRemote_DeviceInformation_Service == nullptr ) {
      DEBUG_PRINT(F("Device Information Service: NOT Found!\n"));
      return true;
    }
      DEBUG_PRINT(F("Client Device Information Service: Found!\n"));
      pRemote_DIS_ManufacturerName_Chr = pRemote_DeviceInformation_Service->getCharacteristic(UUID16_CHR_MANUFACTURER_NAME_STRING);  
      if ( pRemote_DIS_ManufacturerName_Chr != nullptr ) {
          if(pRemote_DIS_ManufacturerName_Chr->canRead()) {
            client_DIS_Manufacturer_Str = pRemote_DIS_ManufacturerName_Chr->readValue();
            DEBUG_PRINTF(" -> Client Reads Manufacturer Name: [%s]\n", client_DIS_Manufacturer_Str.c_str());
          }            
      }     
      pRemote_DIS_ModelNumber_Chr = pRemote_DeviceInformation_Service->getCharacteristic(UUID16_CHR_MODEL_NUMBER_STRING);       
      if ( pRemote_DIS_ModelNumber_Chr != nullptr ) {
          if(pRemote_DIS_ModelNumber_Chr->canRead()) {
            client_DIS_ModelNumber_Str = pRemote_DIS_ModelNumber_Chr->readValue();
            DEBUG_PRINTF(" -> Client Reads Model Number:      [%s]\n", client_DIS_ModelNumber_Str.c_str());
          }
      }  
      pRemote_DIS_SerialNumber_Chr = pRemote_DeviceInformation_Service->getCharacteristic(UUID16_CHR_SERIAL_NUMBER_STRING);       
      if ( pRemote_DIS_SerialNumber_Chr != nullptr ) {
          if(pRemote_DIS_SerialNumber_Chr->canRead()) {
            client_DIS_SerialNumber_Str = pRemote_DIS_SerialNumber_Chr->readValue();
            DEBUG_PRINTF(" -> Client Reads Serial Number:     [%s]\n", client_DIS_SerialNumber_Str.c_str());
          }
      }       
  return true;    
}

bool client_GenericAccess_Connect(void)
{
    // If Generic Access is not found then go on.... NOT FATAL !
    pRemote_GenericAccess_Service = pClient_FTMS->getService(UUID16_SVC_GENERIC_ACCESS);    
    if ( pRemote_GenericAccess_Service == nullptr ) {
      DEBUG_PRINTLN(F("Client Generic Access: NOT Found!"));
      return true;
    }
    DEBUG_PRINTLN("Client Generic Access: Found!");
    pRemote_GA_DeviceName_Chr = pRemote_GenericAccess_Service->getCharacteristic(UUID16_CHR_DEVICE_NAME);  
      if ( pRemote_GA_DeviceName_Chr != nullptr ) {
          if(pRemote_GA_DeviceName_Chr->canRead()) {
            client_GA_DeviceName_Str = pRemote_GA_DeviceName_Chr->readValue();
            DEBUG_PRINTF(" -> Client Reads Device Name:   [%s]\n", client_GA_DeviceName_Str.c_str());
          }            
      }     
      pRemote_GA_Appearance_Chr = pRemote_GenericAccess_Service->getCharacteristic(UUID16_CHR_APPEARANCE);       
      if ( pRemote_GA_Appearance_Chr != nullptr ) {
          if(pRemote_GA_Appearance_Chr->canRead()) {
            client_GA_Appearance_Value = pRemote_GA_Appearance_Chr->readUInt16();
            DEBUG_PRINTF(" -> Client Reads Appearance:    [%d]\n", client_GA_Appearance_Value);
          }
      }     
    return true;     
}

void client_CP_Measurement_Notify_Callback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) 
{
#ifdef DEBUG_CP_MEASUREMENT
  uint8_t buffer[length]= {}; 
  // Transfer first the contents of data to buffer (array of chars)
  DEBUG_PRINTF(" -> Client Rec'd Raw CP Data: [%d] [%d] [ ", isNotify, length); 
  for (int i = 0; i < sizeof(buffer); i++) {
    if ( i <= sizeof(buffer)) {
      buffer[i] = *pData++;
      DEBUG_PRINTF("%02X ", buffer[i], HEX);
    }
  }
  DEBUG_PRINT("] ");
  uint8_t offset = 0;
  // Get flags field
  uint16_t flags = 0;
  memcpy(&flags, &buffer[offset], 2); // Transfer buffer fields to variable
  offset += 2;  // UINT16
  // Get Instantaneous Power values UINT16
  uint16_t PowerValue = 0;
  memcpy(&PowerValue, &buffer[offset], 2); // Transfer buffer fields to variable
  offset += 2;  // UINT16
  DEBUG_PRINTF("Instantaneous Power: %4d\n", PowerValue);
  // Get the other CP measurement values
  if ((flags & 1) != 0) {
    //  Power Balance Present
    DEBUG_PRINT(" --> Pedal Power Balance!");
  }
  if ((flags & 2) != 0) {
    // Accumulated Torque
    DEBUG_PRINTLN(" --> Accumulated Torque!");
  }
  // etcetera...
#endif
} // End cpmc_notify_callback

bool client_CyclingPower_Connect(void)
{
    // Obtain a reference to the remote CP service.
    pRemote_CyclingPower_Service = pClient_FTMS->getService(UUID16_SVC_CYCLING_POWER);
    if (pRemote_CyclingPower_Service == nullptr) {
      DEBUG_PRINTLN("Mandatory Cycling Power Service: Not Found!");
      return false;
    }
    DEBUG_PRINTLN("Client_CyclingPower_Service: Found!");
    pRemote_CP_Measurement_Chr = pRemote_CyclingPower_Service->getCharacteristic(UUID16_CHR_CYCLING_POWER_MEASUREMENT);
    if (pRemote_CP_Measurement_Chr == nullptr) {
      DEBUG_PRINTLN("Mandatory client_CP_Measurement_Chr: Not Found!");
      return false;
    }
    DEBUG_PRINTLN("Client_CP_Measurement_Chr: Found!");  
    if(pRemote_CP_Measurement_Chr->canNotify()) {
      pRemote_CP_Measurement_Chr->registerForNotify(client_CP_Measurement_Notify_Callback, false, true); // Notifications false
    } else {
      DEBUG_PRINTLN("Mandatory Client_CP_Measurement_Chr: Cannot Notify!");
      return false;
    }
    pRemote_CP_Feature_Chr = pRemote_CyclingPower_Service->getCharacteristic(UUID16_CHR_CYCLING_POWER_FEATURE);
    if (pRemote_CP_Feature_Chr == nullptr) {
      DEBUG_PRINTLN("Mandatory Client_CP_Feature_Chr: Not Found!");
      return false;      
    }
    DEBUG_PRINTLN("Client_CP_Feature_Chr: Found!");
    // Read the value of the characteristic.
    if(pRemote_CP_Feature_Chr->canRead()) 
      {
       // Read 32-bit client_CP_Feature_Chr value
      client_CP_Feature_Flags = pRemote_CP_Feature_Chr->readUInt32();
#ifdef DEBUG
      const uint8_t CPFC_FIXED_DATALEN = 4;
      uint8_t cpfcData[CPFC_FIXED_DATALEN] = {(uint8_t)(client_CP_Feature_Flags & 0xff), (uint8_t)(client_CP_Feature_Flags >> 8), 
                                          (uint8_t)(client_CP_Feature_Flags >> 16), (uint8_t)(client_CP_Feature_Flags >> 24)};
      DEBUG_PRINT(" -> Client Reads Raw CP Feature bytes: [4] [ ");
      for (int i = 0; i < CPFC_FIXED_DATALEN; i++) {
        if ( i <= sizeof(cpfcData)) {
        DEBUG_PRINTF("%02X ", cpfcData[i], HEX);
        }
      }
      DEBUG_PRINTLN("] ");
      for (int i = 0; i < client_CP_Feature_Len; i++) {
        if ( client_CP_Feature_Flags & (1 << i) ) {
          DEBUG_PRINTLN(client_CP_Feature_Str[i]);
        }
      }
#endif
      } // canRead Feature
    pRemote_CP_Location_Chr = pRemote_CyclingPower_Service->getCharacteristic(UUID16_CHR_SENSOR_LOCATION);
    if (pRemote_CP_Location_Chr == nullptr) {
      DEBUG_PRINTLN("Client_CP_Location_Chr: Not Found!");
    } else {
      DEBUG_PRINTLN("Client_CP_Location_Chr: Found!");
      // Read the value of the characteristic.
      if(pRemote_CP_Location_Chr->canRead()) {
        client_CP_Location_Value = pRemote_CP_Location_Chr->readUInt8();
        // CP sensor location value is 8 bit
#ifdef DEBUG
        DEBUG_PRINT(" -> Client Reads CP Location Sensor:");
        if(client_CP_Location_Value <= client_Sensor_Location_Str_Len) 
          DEBUG_PRINTF(" Loc#: %d %s\n", client_CP_Location_Value, client_Sensor_Location_Str[client_CP_Location_Value]);
        else 
          DEBUG_PRINTF(" Loc#: %d \n", client_CP_Location_Value);
#endif
      }
    }
    // Now Separately -> Notify Enable 
    if ( pRemote_CP_Measurement_Chr != nullptr ) {
      pRemote_CP_Measurement_Chr->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true); 
      DEBUG_PRINTLN("Cycling Power Measurement: Notify Enabled!");
    }
    return true;    
}

void client_CSC_Measurement_Notify_Callback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify)
{
#ifdef DEBUG_CSC_MEASUREMENT
  uint8_t buffer[length]= {}; 
  // Transfer first the contents of data to buffer (array of chars)
  DEBUG_PRINTF(" -> Client Rec'd Raw CSC Data: [%d] [%d] [ ", isNotify, length); 
  for (int i = 0; i < length; i++) {
    if ( i <= sizeof(buffer)) {
      buffer[i] = *pData++;
      DEBUG_PRINTF("%02X ", buffer[i], HEX);
    }
  }
  DEBUG_PRINT("] ");
  uint8_t offset = 0; 
  // we define the offset that is to be used when reading the next field
  // Size of variables (e.g. 2 (16) or 4 bytes (32)) are constants in BluetoothGattCharacteristic
  // these represent the values you can find in the "Value Fields" table in the "Format" column
  // Read the Flags field at buffer[0]
  uint8_t flags = buffer[offset]; 
  offset += 1; // UINT8 
  // we have to check the flags' nth bit to see if C1 field exists 
  if ((flags & 1) != 0) {
    uint32_t cum_wheel_rev = 0;
    memcpy(&cum_wheel_rev, &buffer[offset], 4);
    offset += 4; // UINT32
    uint16_t last_wheel_event = 0;
    memcpy(&last_wheel_event, &buffer[offset], 2);
    offset += 2; // UINT16
    DEBUG_PRINTF(" Cum. wheel rev.: %d Last wheel event: %d ", cum_wheel_rev, last_wheel_event);
/* Calculation of speed at the Collector can be derived from the wheel circumference and
* data in two successive measurements. The Collector calculation can be performed as
* shown below:
* Speed = (Difference in two successive Cumulative Wheel Revolution values * Wheel Circumference) 
*         / (Difference in two successive Last Wheel Event Time values)
*/
  }
  // we have to check the flags' nth bit to see if C2 field exists 
  if ((flags & 2) != 0) {
    uint16_t cum_cranks = 0;
    memcpy(&cum_cranks, &buffer[offset], 2);
    offset += 2; // UINT16
    uint16_t last_crank_event = 0;
    memcpy(&last_crank_event, &buffer[offset], 2);
    offset += 2; // UINT16
    DEBUG_PRINTF(" Cum cranks: %d Last crank event: %d", cum_cranks, last_crank_event);
/* Calculation of cadence at the Collector can be derived from data in two successive
* measurements. The Collector calculation can be performed as shown below:
* Cadence = (Difference in two successive Cumulative Crank Revolution values)
*           / (Difference in two successive Last Crank Event Time values)        
*/         
  }
  // etcetera...
  DEBUG_PRINTLN();
#endif
}

bool client_CyclingSpeedCadence_Connect(void)
{
    // Obtain a reference to the remote HRM service.
    pRemote_CyclingSpeedCadence_Service = pClient_FTMS->getService(UUID16_SVC_CYCLING_SPEED_AND_CADENCE);
    if (pRemote_CyclingSpeedCadence_Service == nullptr) {
      DEBUG_PRINTLN("Cycling Speed Cadence Service: Not Found!");
      return true;
    }
    DEBUG_PRINTLN("Client_CyclingSpeedCadence_Service: Found!");
    pRemote_CSC_Measurement_Chr = pRemote_CyclingSpeedCadence_Service->getCharacteristic(UUID16_CHR_CSC_MEASUREMENT);
    if (pRemote_CSC_Measurement_Chr == nullptr) {
      DEBUG_PRINTLN("Mandatory client_CSC_Measurement_Chr: Not Found!");
      return false;
    }
    DEBUG_PRINTLN("Client_CSC_Measurement_Chr: Found!");  
    if(pRemote_CSC_Measurement_Chr->canNotify()) {
      pRemote_CSC_Measurement_Chr->registerForNotify(client_CSC_Measurement_Notify_Callback, false, true); // Notifications false
    } else {
      DEBUG_PRINTLN("Mandatory Client_CSC_Measurement_Chr: Cannot Notify!");
      return false;
    }
    pRemote_CSC_Feature_Chr = pRemote_CyclingSpeedCadence_Service->getCharacteristic(UUID16_CHR_CSC_FEATURE);
    if (pRemote_CSC_Feature_Chr == nullptr) {
      DEBUG_PRINTLN("Mandatory Client_CSC_Feature_Chr: Not Found!");
      return false;      
    }
    DEBUG_PRINTLN("Client_CSC_Feature_Chr: Found!");
    // Read the value of the characteristic.
    if(pRemote_CSC_Feature_Chr->canRead()) 
      {
        // Read 16-bit client_CSC_Feature_Chr value
        client_CSC_Feature_Flags = pRemote_CSC_Feature_Chr->readUInt16();
#ifdef DEBUG
        uint8_t cscfcData[CSC_FEATURE_FIXED_DATALEN] = { (uint8_t)(client_CSC_Feature_Flags & 0xff), (uint8_t)(client_CSC_Feature_Flags >> 8) }; //  Little Endian Representation
        DEBUG_PRINTF(" -> Client Reads Raw CSC Feature bytes: [2] [ ");
        for (int i = 0; i < sizeof(cscfcData); i++) {
          DEBUG_PRINTF("%02X ", cscfcData[i], HEX);
        }
        DEBUG_PRINTLN("] ");
        for (int i = 0; i < client_CSC_Feature_Len; i++) {
          if ( (client_CSC_Feature_Flags & (1 << i)) != 0 ) {
            DEBUG_PRINTLN(client_CSC_Feature_Str[i]);
            }
        }
#endif
      } // canRead Feature
     
    pRemote_CSC_Location_Chr = pRemote_CyclingSpeedCadence_Service->getCharacteristic(UUID16_CHR_SENSOR_LOCATION);
    if (pRemote_CSC_Location_Chr == nullptr) {
      DEBUG_PRINTLN("Client_CSC_Location_Chr: Not Found!");
    } else {
      DEBUG_PRINTLN("Client_CSC_Location_Chr: Found!");
      // Read the value of the characteristic.
      if(pRemote_CSC_Location_Chr->canRead()) {
        client_CSC_Location_Value = pRemote_CSC_Location_Chr->readUInt8();
        // CSC sensor location value is 8 bit
#ifdef DEBUG
        DEBUG_PRINT(" -> Client Reads CSC Location Sensor:");
        if(client_CSC_Location_Value <= client_Sensor_Location_Str_Len) 
          DEBUG_PRINTF(" Loc#: %d %s\n", client_CSC_Location_Value, client_Sensor_Location_Str[client_CSC_Location_Value]);
        else 
          DEBUG_PRINTF(" Loc#: %d \n", client_CSC_Location_Value);
#endif
      }
    }
    // Now Separately -> Notify Enable 
    if ( pRemote_CSC_Measurement_Chr != nullptr ) {  
      pRemote_CSC_Measurement_Chr->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true); 
      DEBUG_PRINTLN("Client_CSC_Measurement_Chr: Notify Enabled!");
    }
    return true;    
}

bool client_HeartRate_Connect(void)
{
    // Obtain a reference to the remote HRM service.
    pRemote_HeartRate_Service = pClient_FTMS->getService(UUID16_SVC_HEART_RATE);
    if (pRemote_HeartRate_Service == nullptr) {
      DEBUG_PRINTLN("Heart Rate Service: Not Found!");
      return true;
    }
    DEBUG_PRINTLN("Client_HeartRate_Service: Found!");
    pRemote_HR_Measurement_Chr = pRemote_HeartRate_Service->getCharacteristic(UUID16_CHR_HEART_RATE_MEASUREMENT);
    if (pRemote_HR_Measurement_Chr == nullptr) {
      DEBUG_PRINTLN("Mandatory client_HR_Measurement_Chr: Not Found!");
      return false;
    }
    DEBUG_PRINTLN("Client_HR_Measurement_Chr: Found!");  
    if(pRemote_HR_Measurement_Chr->canNotify()) {
      pRemote_HR_Measurement_Chr->registerForNotify(client_HR_Measurement_Notify_Callback, false, true); // Notifications false
    } else {
      DEBUG_PRINTLN("Mandatory Client_HR_Measurement_Chr: Cannot Notify!");
      return false;
    }
    pRemote_HR_Location_Chr = pRemote_HeartRate_Service->getCharacteristic(UUID16_CHR_BODY_SENSOR_LOCATION);
    if (pRemote_HR_Location_Chr == nullptr) {
      DEBUG_PRINTLN("Client_HR_Location_Chr: Not Found!");
    } else {
      DEBUG_PRINTLN("Client_HR_Location_Chr: Found!");
      // Read the value of the characteristic.
      if(pRemote_HR_Location_Chr->canRead()) {
        client_HR_Location_Value = pRemote_HR_Location_Chr->readUInt8();
        // Body sensor location value is 8 bit
#ifdef DEBUG
        const uint8_t body_str_len = 7;
        const char* body_str[body_str_len] = { "Other", "Chest", "Wrist", "Finger", "Hand", "Ear Lobe", "Foot" };
        DEBUG_PRINT(" -> Client Reads HR Location Sensor:");
        if(client_HR_Location_Value <= body_str_len)
          DEBUG_PRINTF(" Loc#: %d %s\n", client_HR_Location_Value, body_str[client_HR_Location_Value]);
        else 
          DEBUG_PRINTF(" Loc#: %d \n", client_HR_Location_Value);
#endif
      }
    }
    // Now Separately -> Notify Enable 
    if ( pRemote_HR_Measurement_Chr != nullptr ) {  
      pRemote_HR_Measurement_Chr->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true); 
      DEBUG_PRINTLN("Client_HR_Measurement_Chr: Notify Enabled!");
    }
    return true;    
}

void client_FTM_TrainingStatus_Notify_Callback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify)
{
#ifdef DEBUG
  uint8_t SDataLen = (uint8_t)length;
  uint8_t SDataBuf[SDataLen] = {};
  // Transfer first the contents of data to buffer (array of chars)
  DEBUG_PRINTF(" -> Client Rec'd Raw FTM Training Status Data: [%d] [%d] [ ", isNotify, length);
  for (int i = 0; i <  sizeof(SDataBuf); i++) {
      SDataBuf[i] = *pData++;
      DEBUG_PRINTF("%02X ", SDataBuf[i], HEX);
  }
  DEBUG_PRINTLN("] ");
#endif
}

void client_FTM_Status_Notify_Callback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify)
{
#ifdef DEBUG
  uint8_t SDataLen = (uint8_t)length;
  uint8_t SDataBuf[SDataLen] = {};
  // Transfer first the contents of data to buffer (array of chars)
  DEBUG_PRINTF(" -> Client Rec'd Raw FTM Machine Status Data: [%d] [%d] [ ", isNotify, length);
  for (int i = 0; i < sizeof(SDataBuf); i++) {
      SDataBuf[i] = *pData++;
      DEBUG_PRINTF("%02X ", SDataBuf[i], HEX);
  }
  DEBUG_PRINTLN("] ");
#endif
}

void client_FTM_IndoorBikeData_Notify_Callback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify)
{
#ifdef DEBUG
  uint8_t IBDDataLen = (uint8_t)length;
  uint8_t IBDDataBuf[IBDDataLen] = {};
  // Transfer first the contents of data to buffer (array of chars)
  DEBUG_PRINTF(" -> Client Rec'd Raw FTM Indoor Bike Data: [%d] [%d] [ ", isNotify, length);
  for (int i = 0; i < sizeof(IBDDataBuf); i++) {
      IBDDataBuf[i] = *pData++;
      DEBUG_PRINTF("%02X ", IBDDataBuf[i], HEX);
  }
  DEBUG_PRINTLN("]");
#endif
// Decoding Indoor Bike Data and presenting
// ---> Buffer Data Length depends on data flagged to be present !!!!
#ifdef DEBUG_IBD
  uint8_t offset = 0;
  uint16_t flags = 0;
  memcpy(&flags, &IBDDataBuf[offset], 2); // Transfer buffer fields to variable
  offset += 2;  // UINT16
  if ((flags & 1) == 0) { // Inst. Speed is present (!) if flag (bit 0) is set to 0 (null)
    //  More Data --> Instantaneous Speed 0.01 
    uint16_t inst_Speed = 0;
    memcpy(&inst_Speed, &IBDDataBuf[offset], 2); // Transfer buffer fields to variable
    DEBUG_PRINTF("Instant. Speed: %d KPH", (inst_Speed/100));
    offset += 2;  // UINT16 
    }
  if ((flags & 2) != 0) { 
    //  Average Speed 0.01 
    uint16_t av_Speed = 0;
    memcpy(&av_Speed, &IBDDataBuf[offset], 2); // Transfer buffer fields to variable
    DEBUG_PRINTF(" Average Speed: %d KPH", (av_Speed/100));
    offset += 2;  // UINT16 
    }
  if ((flags & 4) != 0) {
    //  Instantaneous Cadence 0.5
    uint16_t inst_Cadence = 0;
    memcpy(&inst_Cadence, &IBDDataBuf[offset], 2); // Transfer buffer fields to variable
    DEBUG_PRINTF(" Instantaneous Cadence: %d RPM", (inst_Cadence/2));
    offset += 2;  // UINT16 
    }
  if ((flags & 8) != 0) {
    //  Average Cadence 0.5
    uint16_t av_Cadence = 0;
    memcpy(&av_Cadence, &IBDDataBuf[offset], 2); // Transfer buffer fields to variable
    DEBUG_PRINTF(" Average Cadence: %d RPM", (av_Cadence/2));
    offset += 2;  // UINT16 
    }
  if ((flags & 16) != 0) {
    //  Total Distance  1
    // Little endian format, transfer 24 bit to 32 bit variable
    uint32_t tot_Distance = 0;
    memcpy(&tot_Distance, &IBDDataBuf[offset], 3); // Transfer buffer fields to variable
    DEBUG_PRINTF(" Total Distance: %d m", tot_Distance);
    offset += 3;  // UINT24 16 + 8
    }
  if ((flags & 32) != 0) {
    //  Resistance Level  1
    uint16_t res_Level = 0;
    memcpy(&res_Level, &IBDDataBuf[offset], 2); // Transfer buffer fields to variable
    DEBUG_PRINTF(" Resistance Level: %d ", res_Level);
    offset += 2;  // UINT16 
    }
  if ((flags & 64) != 0) {
    //  Instantaneous Power 1
    uint16_t inst_Power = 0;
    memcpy(&inst_Power, &IBDDataBuf[offset], 2); // Transfer buffer fields to variable
    DEBUG_PRINTF(" Instantaneous Power: %d Watt", inst_Power);
    offset += 2;  // UINT16 
    }
  if ((flags & 128) != 0) {
    //  Average Power 1
    uint16_t av_Power = 0;
    memcpy(&av_Power, &IBDDataBuf[offset], 2); // Transfer buffer fields to variable
    DEBUG_PRINTF(" Average Power: %d Watt", av_Power);
    offset += 2;  // UINT16 
    }
  if ((flags & 256) != 0) {
    //  Expended Energy -> UINT16 UINT16 UINT8
    // Total Energy UINT16  1
    uint16_t tot_Energy = 0;
    memcpy(&tot_Energy, &IBDDataBuf[offset], 2); // Transfer buffer fields to variable
    DEBUG_PRINTF(" Tot. Energy: %d kCal", tot_Energy);
    offset += 2;  // UINT16 
    // Energy per hour UINT16 1
    uint16_t Energy_hr = 0;
    memcpy(&Energy_hr, &IBDDataBuf[offset], 2); // Transfer buffer fields to variable
    DEBUG_PRINTF(" Energy/hr: %d kCal/hr", Energy_hr);
    offset += 2;  // UINT16 
    // Energy per minute UINT8  1
    uint8_t Energy_pm = 0;
    memcpy(&Energy_pm, &IBDDataBuf[offset], 1); // Transfer buffer fields to variable
    DEBUG_PRINTF(" Energy/m: %d kCal/m", Energy_pm);
    offset += 1;  // UINT8 
    }
  if ((flags & 512) != 0) {
    //  Heart Rate  1
    uint8_t Heart_Rate = 0;
    memcpy(&Heart_Rate, &IBDDataBuf[offset], 1); // Transfer buffer fields to variable
    DEBUG_PRINTF(" Heart Rate: %d HBM", Heart_Rate);
    offset += 1;  // UINT8 
    }
  if ((flags & 1024) != 0) {
    //  Metabolic Equivalent 0.1
    uint8_t Mets = 0;
    memcpy(&Mets, &IBDDataBuf[offset], 1); // Transfer buffer fields to variable
    DEBUG_PRINTF(" Metabolic Equivalent: %d ", Mets/10);
    offset += 1;  // UINT8 
    }
  if ((flags & 2048) != 0) {
    //  Elapsed Time  1
    uint16_t elap_time = 0;
    memcpy(&elap_time, &IBDDataBuf[offset], 2); // Transfer buffer fields to variable
    DEBUG_PRINTF(" Elapsed time: %d s", elap_time);
    offset += 2;  // UINT16 
    }
  if ((flags & 4096) != 0) {
    //  Remaining Time  1
    uint16_t rem_time = 0;
    memcpy(&rem_time, &IBDDataBuf[offset], 2); // Transfer buffer fields to variable
    DEBUG_PRINTF(" Remaining time: %d s", rem_time);
    offset += 2;  // UINT16 
    }
  DEBUG_PRINTLN();
#endif
}

void client_FTM_ControlPoint_Indicate_Callback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify)
{
#ifdef DEBUG
  uint8_t RespBufferLen = (uint8_t)length;
  uint8_t RespBuffer[RespBufferLen] = {}; // It is max 6 bytes long
  // Transfer first the contents of data to buffer (array of chars)
  DEBUG_PRINTF(" -> Client Rec'd Raw FTM Control Point Response Data: [%d] [%d] [ ", isNotify, length); 
  for (int i = 0; i < sizeof(RespBuffer); i++) {
      RespBuffer[i] = *pData++;
      DEBUG_PRINTF("%02X ", RespBuffer[i], HEX);
  }
  DEBUG_PRINTLN("]");
#endif  
}

bool client_FitnessMachine_Connect(void)
{
    // Obtain a reference to the remote FTMS service.
    pRemote_FitnessMachine_Service = pClient_FTMS->getService(UUID16_SVC_FITNESS_MACHINE);
    if (pRemote_FitnessMachine_Service == nullptr) {
      DEBUG_PRINTLN("Mandatory client_FitnessMachine_Service: Not Found!");
      return false;
    }
    DEBUG_PRINTLN("Client_FitnessMachine_Service: Found!");

    pRemote_FTM_Feature_Chr = pRemote_FitnessMachine_Service->getCharacteristic(UUID16_CHR_FITNESS_MACHINE_FEATURE);
    if (pRemote_FTM_Feature_Chr == nullptr) {
      DEBUG_PRINTLN("Mandatory Client_FTM_Feature_Chr: Not Found!");
      return false;
    } else {
      DEBUG_PRINTLN("Client_FTM_Feature_Chr: Found!");
      // Read the value of the characteristic.
      if(pRemote_FTM_Feature_Chr->canRead()) {
        // Read Feature Data
        client_FTM_Feature_Str = pRemote_FTM_Feature_Chr->readValue();
#ifdef DEBUG
        uint8_t* MyPtr = (uint8_t*)client_FTM_Feature_Str.c_str();  // Points to first 4 bytes of std::string     
        DEBUG_PRINT(" -> Client Reads Raw FTM Feature bytes: [8] [ ");
        for (int i = 0; i < client_FTM_Feature_Str.length(); i++) {
          DEBUG_PRINTF("%02X ", client_FTM_Feature_Str[i], HEX);
        } // for
        DEBUG_PRINTLN("] ");
        DEBUG_PRINTLN("- Fitness Machine Features:");
        // Load 32-bit client_CP_Feature_Chr value
        uint32_t client_FTM_Feature_Flags_One;
        memcpy(&client_FTM_Feature_Flags_One, MyPtr, 4);  
        for (int i = 0; i < client_FTM_Feature_Str_One_Len; i++) {
          if ( client_FTM_Feature_Flags_One & (1 << i) ) {
            DEBUG_PRINTLN(client_FTM_Feature_Str_One[i]);
          }
        }
        DEBUG_PRINTLN("- Target Setting Features:");
        // Load 32-bit client_CP_Feature_Chr value
        uint32_t client_FTM_Feature_Flags_Two;
        memcpy(&client_FTM_Feature_Flags_Two, MyPtr+4, 4); // Points to second 4 bytes of std::string
        for (int i = 0; i < client_FTM_Feature_Str_Two_Len; i++) {
          if ( client_FTM_Feature_Flags_Two & (1 << i) ) {
            DEBUG_PRINTLN(client_FTM_Feature_Str_Two[i]);
          }
        }
#endif
      }
    } 

    pRemote_FTM_IndoorBikeData_Chr = pRemote_FitnessMachine_Service->getCharacteristic(UUID16_CHR_INDOOR_BIKE_DATA);
    if (pRemote_FTM_IndoorBikeData_Chr == nullptr) {
      DEBUG_PRINTLN("Mandatory Client_FTM_IndoorBikeData_Chr: Not Found!");
      return false; // Mandatory when service is present
    }
    DEBUG_PRINTLN("Client_FTM_IndoorBikeData_Chr: Found!");  
    if(pRemote_FTM_IndoorBikeData_Chr->canNotify()) {
      pRemote_FTM_IndoorBikeData_Chr->registerForNotify(client_FTM_IndoorBikeData_Notify_Callback, false, true); // Notifications false
    } else {
      DEBUG_PRINTLN("Mandatory Client_FTM_IndoorBikeData_Chr: Cannot Notify!");
      return false; // Mandatory when service is present
    }
    // Now Separately -> Notify Enable 
    if( pRemote_FTM_IndoorBikeData_Chr != nullptr) { // Check: Is it exposed?
      pRemote_FTM_IndoorBikeData_Chr->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true); 
      DEBUG_PRINTLN("Client_FTM_IndoorBikeData_Chr: Notify Enabled!");
    }
          
    pRemote_FTM_TrainingStatus_Chr = pRemote_FitnessMachine_Service->getCharacteristic(UUID16_CHR_TRAINING_STATUS);
    if (pRemote_FTM_TrainingStatus_Chr == nullptr) {
      DEBUG_PRINTLN("Client_FTM_TrainingStatus_Chr: Not Found!");
      // NOT Mandatory
    } else {  
      DEBUG_PRINTLN("Client_FTM_TrainingStatus_Chr: Found!");  
      if(pRemote_FTM_TrainingStatus_Chr->canNotify()) {
        pRemote_FTM_TrainingStatus_Chr->registerForNotify(client_FTM_TrainingStatus_Notify_Callback, false, true); // Notifications false
      } else {
        DEBUG_PRINTLN("Mandatory Client_FTM_TrainingStatus_Chr: Cannot Notify!");
        return false; // Mandatory when service is present
      }
    // Now Separately -> Notify Enable 
      if( pRemote_FTM_TrainingStatus_Chr != nullptr) { // Check: Is it exposed?
        pRemote_FTM_TrainingStatus_Chr->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true); 
        DEBUG_PRINTLN("Client_FTM_TrainingStatus_Chr: Notify Enabled!");
      }
    }

    pRemote_FTM_SupportedResistanceLevelRange_Chr = pRemote_FitnessMachine_Service->getCharacteristic(UUID16_CHR_SUPPORTED_RESISTANCE_LEVEL_RANGE);
    if (pRemote_FTM_SupportedResistanceLevelRange_Chr == nullptr) {
      DEBUG_PRINTLN("Client_FTM_SupportedResistanceLevelRange_Chr: Not Found!");
    } else {
      DEBUG_PRINTLN("Client_FTM_SupportedResistanceLevelRange_Chr: Found!");
      // Read the value of the characteristic.
      if(pRemote_FTM_SupportedResistanceLevelRange_Chr->canRead()) {
        // Read Supported Resistance Level Range Data
        client_FTM_SupportedResistanceLevelRange_Str = pRemote_FTM_SupportedResistanceLevelRange_Chr->readValue();
#ifdef DEBUG
        DEBUG_PRINT(" -> Client Reads Raw FTM Supported Resistance Level Range bytes: [6] [ ");
        for (int i = 0; i < client_FTM_SupportedResistanceLevelRange_Str.length(); i++) {
          DEBUG_PRINTF("%02X ", client_FTM_SupportedResistanceLevelRange_Str[i], HEX);
        } // for
      DEBUG_PRINTLN("] ");
#endif
      }
    } 

    pRemote_FTM_SupportedPowerRange_Chr = pRemote_FitnessMachine_Service->getCharacteristic(UUID16_CHR_SUPPORTED_POWER_RANGE);
    if (pRemote_FTM_SupportedPowerRange_Chr == nullptr) {
      DEBUG_PRINTLN("Client_FTM_SupportedPowerRange_Chr: Not Found!");
    } else {
      DEBUG_PRINTLN("Client_FTM_SupportedPowerRange_Chr: Found!");
      // Read the value of the characteristic.
      if(pRemote_FTM_SupportedPowerRange_Chr->canRead()) {
        // Read Supported Power Range Data
        client_FTM_SupportedPowerRange_Str = pRemote_FTM_SupportedPowerRange_Chr->readValue();
#ifdef DEBUG
        DEBUG_PRINT(" -> Client Reads Raw FTM Supported Power Range bytes: [6] [ ");
        for (int i = 0; i < client_FTM_SupportedPowerRange_Str.length(); i++) {
          DEBUG_PRINTF("%02X ", client_FTM_SupportedPowerRange_Str[i], HEX);
        } // for
      DEBUG_PRINTLN("] ");
#endif
      }
    }  

    pRemote_FTM_ControlPoint_Chr = pRemote_FitnessMachine_Service->getCharacteristic(UUID16_CHR_FITNESS_MACHINE_CONTROL_POINT);
    if (pRemote_FTM_ControlPoint_Chr == nullptr) {
      DEBUG_PRINTLN("Mandatory Client_FTM_ControlPoint_Chr: Not Found!");
      return false; // Mandatory when service is present
    }
    DEBUG_PRINTLN("Client_FTM_ControlPoint_Chr: Found!");
    if(pRemote_FTM_ControlPoint_Chr->canIndicate()) {
      pRemote_FTM_ControlPoint_Chr->registerForNotify(client_FTM_ControlPoint_Indicate_Callback, false, true); // Notifications false
    } else {
      DEBUG_PRINTLN("Mandatory Client_FTM_ControlPoint_Chr: Cannot Indicate!");
      return false; // Mandatory when service is present
    }
    // Now Separately -> Indicate Enable 
    if( pRemote_FTM_ControlPoint_Chr != nullptr) { // Check: Is it exposed?
      pRemote_FTM_ControlPoint_Chr->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)indicationOn, 2, true); 
      DEBUG_PRINTLN("Client_FTM_ControlPoint_Chr: Indicate Enabled!");
    }

    pRemote_FTM_Status_Chr = pRemote_FitnessMachine_Service->getCharacteristic( UUID16_CHR_FITNESS_MACHINE_STATUS);
    if (pRemote_FTM_Status_Chr == nullptr) {
      DEBUG_PRINTLN("Mandatory Client_FTM_Status_Chr: Not Found!");
      return false; // Mandatory when service is present
    }
    DEBUG_PRINTLN("Client_FTM_Status_Chr: Found!");  
    if(pRemote_FTM_Status_Chr->canNotify()) {
      pRemote_FTM_Status_Chr->registerForNotify(client_FTM_Status_Notify_Callback, false, true); // Notifications false
    } else {
      DEBUG_PRINTLN("Mandatory Client_FTM_Status_Chr: Cannot Notify!");
      return false; // Mandatory when service is present
    }
    // Now Separately -> Notify Enable 
    if( pRemote_FTM_Status_Chr != nullptr) { // Check: Is it exposed?
      pRemote_FTM_Status_Chr->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true); 
      DEBUG_PRINTLN("Client_FTM_Status_Chr: Notify Enabled!");
    }
return true;
} // end Fitness Machine SVC

// This is NOT really a Callback --> It should have been implemented that way (see for instance Adafruit Bluefruit BLE library), 
// however, now it is called from loop() ... a poor man's solution!
bool client_Connect_Callback() {
    // Connect to the FTMS BLE Server.
    pClient_FTMS->connect(myDevice);  // if you pass BLEAdvertisedDevice it will recognize type of peer device address (public or private)     
    DEBUG_PRINTLN("Now checking all Client Services and Characteristics!");
    DEBUG_PRINTLN("If Mandatory Services Fail --> the Client will disconnect!");
  // Discover all relevant Services and Char's
  if( !client_GenericAccess_Connect() ) {
    pClient_FTMS->disconnect();
    return false;    
  }
  if( !client_DeviceInformation_Connect() ) {
    pClient_FTMS->disconnect();
    return false;    
  }
  if( !client_CyclingPower_Connect() ) {
    pClient_FTMS->disconnect();
    return false;    
  }
 if( !client_CyclingSpeedCadence_Connect() ) {
    pClient_FTMS->disconnect();
    return false;    
  }
 if( !client_FitnessMachine_Connect() ) {
    pClient_FTMS->disconnect();
    return false;    
  }
  if( !client_HeartRate_Connect() ) {
    pClient_FTMS->disconnect();
    return false;    
  }
 
  clientIsConnected = true;
  TimeInterval = millis() + CONTROL_POINT_TIME_SPAN;  
  return true;
}

/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
/*
 * Called for each advertising BLE server.
*/
  void onResult(BLEAdvertisedDevice* advertisedDevice) {  // NIMBLE

    DEBUG_PRINT("BLE Advertised Device found: ");    
    DEBUG_PRINTLN(advertisedDevice->toString().c_str()); // NIMBLE
    // We have found a device, let us now see if it contains the FTMS service we are looking for.
    if (advertisedDevice->haveServiceUUID() && advertisedDevice->isAdvertisingService(UUID16_SVC_FITNESS_MACHINE)) { // NIMBLE
      BLEDevice::getScan()->stop();
      myDevice = advertisedDevice; // NIMBLE --> Just save the reference now, no need to copy the object         
      /* Connect to the FTMS BLE Server -> Sorry you can't do that here!!! --------------------------------
      ** pClient_FTMS->connect(myDevice);  NOT ALLOWED TO CALL CONNECT --> CAUSES FATAL ERROR !!!! ???? */  
      doClientConnectCall = true;         // Work around via loop()           
    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks

void client_Start_Scanning(void)
{
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start in loop()
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  RestartScanningOnDisconnect = true;
}

void loop() {
  // If the flag "doClientConnectCall" is true, we have found the desired BLE server!
  // Once we are connected and ALL is set the clientIsConnected flag is set true.
  if (doClientConnectCall == true) {
    bool dummy = client_Connect_Callback();
    doClientConnectCall = false;
  } // doClientConnectCall

  if (clientIsConnected) {
    // If time is there, send test values of Indoor Bike Simulation Parameters to 
    // the Trainer's FTM Control Point to drive the FTM...
    if(millis() > TimeInterval) {
        if(ControlPointMessageCount > 31) { ControlPointMessageCount = 0; } // start all over again!
        uint8_t CPData[8] = {};
        DEBUG_PRINT("Client sends Indoor Bike Simulation Parameters to Trainer's FTM Control Point: [ ");
        // Transfer multidimensional test data to CPData buffer
        for (int i = 0; i < sizeof(CPData); i++) {
          CPData[i] = ControlPointData[ControlPointMessageCount][i];
          DEBUG_PRINTF("%02X ", CPData[i], HEX);
        }
        DEBUG_PRINTLN("] ");
        // write with response !!! writeValue(uint8_t* data, size_t length, bool response = false);
        pRemote_FTM_ControlPoint_Chr->writeValue(CPData, 8, true);
        TimeInterval = millis() + CONTROL_POINT_TIME_SPAN;
        ControlPointMessageCount++;
    } // TimeInterval
  } else { // client is NOT connected check for (re)start scanning
    if(RestartScanningOnDisconnect) {
        pBLEScan->clearResults();   // delete results from BLEScan buffer to release memory    
        DEBUG_PRINTLN("Client Starts Scanning for Server Device with CPS, CSC and FTMS!");
        RestartScanningOnDisconnect = false;        
        pBLEScan->start(0, false);
    }
  }      
} // End of loop