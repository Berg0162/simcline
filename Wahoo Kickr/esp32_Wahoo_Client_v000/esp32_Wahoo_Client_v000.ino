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
 *  This Adafruit Feather ESP32 V2 code scans for the CPS + Wahoo Char
 *  that the trainer is advertising, it tries to connect and then 
 *  enables .....
 *  
 *  Requirements: Wahoo trainer and an ESP32 board
 *  1) Upload and Run this code on ESP32 board
 *  2) Start the Serial Monitor to catch verbose debugging and data info
 *  3) Power ON/Wake UP trainer -> do NOT connect with other devices
 *  4) Trainer and ESP32 board should successfully pair or disconnect...
 *  5) Keep the Serial Monitor visible on top of all windows 
 *  6) Move the trainer pedals and notice/feel changes in resistance...
 *     The Client sends Resistance Parameters to the Trainer
 *  7) Inspect the info presented by Serial Monitor.....
 *  
 */

#include <algorithm>

// -------------------------------------------------------------------------------------------
// COMPILER DIRECTIVE to allow/suppress DEBUG messages that help debugging...
// Uncomment general "#define DEBUG" to activate
#define DEBUG
// Include these debug utility macros in all cases!
#include "DebugUtils.h"
//
#ifdef DEBUG
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
/* ---------------------------------------------------------------------------------------------------------------
 * Wahoo Proprietary Control Point Characteristic
 * ---------------------------------------------------------------------------------------------------------------*/
// Proprietary Wahoo Trainer Control Point Characteristic is part of the Cycling Power Service !
static BLEUUID UUID16_CHR_WAHOO_CONTROL_POINT("A026E005-0A7D-4AB3-97FA-F1500F9FEB8B");
static BLERemoteCharacteristic* pRemote_Wahoo_ControlPoint_Chr; //  Wahoo Control Point, optional, write & indicate

// client_CPS Wahoo Trainer Operation Codes in Decimal
const uint8_t unlock                     = 32;
/*
const uint8_t setResistanceMode          = 64;
const uint8_t setStandardMode            = 65;
const uint8_t setErgMode                 = 66;
const uint8_t setSimMode                 = 67;
const uint8_t setSimCRR                  = 68;
const uint8_t setSimWindResistance       = 69;
const uint8_t setSimGrade                = 70;
const uint8_t setSimWindSpeed            = 71;
const uint8_t setWheelCircumference      = 72;
*/
const uint8_t unlockCommand[3] = {unlock, 0xEE, 0xFC}; // Unlock codes

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
      
const char* client_Sensor_Location_Str[] = { "Other", "Top of shoe", "In shoe", "Hip", 
    "Front wheel", "Left crank", "Right crank", "Left pedal", "Right pedal", "Front hub", 
    "Rear dropout", "Chainstay", "Rear wheel", "Rear hub", "Chest", "Spider", "Chain ring"};

/*------------------------------------------------ OPTIONAL ---------------------------------------
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

// --------------------------------------------------------------------------------------
static BLEClient* pClient_Wahoo;
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
#define CONTROL_POINT_TIME_SPAN 3000    // Time span for sending Wahoo Control Point data
unsigned long TimeInterval = 0;
float Grade = 0.0;

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
  DEBUG_PRINTLN("ESP32 BLE Wahoo Client/Central: CPS + Wahoo and CSC (optional)");
  DEBUG_PRINTLN("------------------------ Version 00.0 ------------------------");
  BLEDevice::init("");
  pClient_Wahoo  = BLEDevice::createClient();
  pClient_Wahoo->setClientCallbacks(new client_Connection_Callbacks());    
  client_Start_Scanning();
} // End of setup.

void client_Connection_Callbacks::onConnect(BLEClient* pClient) {
    NimBLEAddress MyAddress = myDevice->getAddress();
    clientPeerName = myDevice->getName().c_str();
    //DEBUG_PRINTF("BLEDevice address in Little Endian order: [%s]\n", MyAddress->getAddress().toString().c_str());
    memcpy(&clientPeerAddress, MyAddress.getNative(), 6); 
#ifdef DEBUG
    DEBUG_PRINT("Client Connection Parameters -> ");
    uint16_t max_payload = pClient_Wahoo->getMTU()-3;
    //DEBUG_PRINTF("Max Transmission Unit: [%d] ", max_payload);
    uint16_t clientConnectionInterval = pClient_Wahoo->getConnInfo().getConnInterval();
    DEBUG_PRINTF("Interval: [%d] ", clientConnectionInterval);
    uint16_t clientConnectionLatency = pClient_Wahoo->getConnInfo().getConnLatency();
    DEBUG_PRINTF("Latency: [%d] ", clientConnectionLatency);
    uint16_t clientConnectionSupTimeout = pClient_Wahoo->getConnInfo().getConnTimeout();
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
    //pClient_Wahoo->updateConnParams(pClient_Wahoo->getConnId(), 24, 48, 0, 400); 
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
    pRemote_DeviceInformation_Service = pClient_Wahoo->getService(UUID16_SVC_DEVICE_INFORMATION);    
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
    pRemote_GenericAccess_Service = pClient_Wahoo->getService(UUID16_SVC_GENERIC_ACCESS);    
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

void client_Wahoo_ControlPoint_Indicate_Callback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify)
{
#ifdef DEBUG
  uint8_t RespBufferLen = (uint8_t)length;
  uint8_t RespBuffer[RespBufferLen] = {}; // It is max 6 bytes long
  // Transfer first the contents of data to buffer (array of chars)
  DEBUG_PRINT(" -> Client Rec'd Raw Wahoo Control Point Response Data: [ "); 
  for (int i = 0; i < sizeof(RespBuffer); i++) {
      RespBuffer[i] = *pData++;
      DEBUG_PRINTF("%02X ", RespBuffer[i], HEX);
  }
  DEBUG_PRINTLN("]");
#endif  
}

bool client_CyclingPower_Connect(void)
{
    // Obtain a reference to the remote CP service.
    pRemote_CyclingPower_Service = pClient_Wahoo->getService(UUID16_SVC_CYCLING_POWER);
    if (pRemote_CyclingPower_Service == nullptr) {
      DEBUG_PRINTLN("Mandatory Cycling Power Service: Not Found!");
      return false;
    }
    DEBUG_PRINTLN("Client_CyclingPower_Service: Found!");
    pRemote_Wahoo_ControlPoint_Chr = pRemote_CyclingPower_Service->getCharacteristic(UUID16_CHR_WAHOO_CONTROL_POINT);
    if (pRemote_Wahoo_ControlPoint_Chr == nullptr) {
      DEBUG_PRINTLN("Mandatory Client_Wahoo_ControlPoint_Chr: Not Found!");
      return false; // Mandatory when service is present
    }
    DEBUG_PRINTLN("Client_Wahoo_ControlPoint_Chr: Found!");
    if(pRemote_Wahoo_ControlPoint_Chr->canIndicate()) {
      pRemote_Wahoo_ControlPoint_Chr->registerForNotify(client_Wahoo_ControlPoint_Indicate_Callback, false, true); // Notifications false
    } else {
      DEBUG_PRINTLN("Mandatory Client_Wahoo_ControlPoint_Chr: Cannot Indicate!");
      return false; // Mandatory when service is present
    }
    // Now Separately -> Indicate Enable 
    if( pRemote_Wahoo_ControlPoint_Chr != nullptr) { // Check: Is it exposed?
      pRemote_Wahoo_ControlPoint_Chr->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)indicationOn, 2, true); 
      DEBUG_PRINTLN("Client_Wahoo_ControlPoint_Chr: Indicate Enabled!");
      // Unlock the client_Wahoo_ControlPoint Characteristic at the Wahoo trainer
      DEBUG_PRINTLN("Client sends to Wahoo Control Point: [ 20 EE FC ] -> Unlock Command Key");
      // write with response !!! writeValue(uint8_t* data, size_t length, bool response = false);
      pRemote_Wahoo_ControlPoint_Chr->writeValue(unlockCommand, 3, true);      
      delay(100);    // Give the trainer some time to wake up
      Set_SimMode(); // Set the trainers Simulation mode first time      
    }
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
        DEBUG_PRINT(" -> Client Reads CP Location Sensor:");
        DEBUG_PRINTF(" Loc#: %d %s\n", client_CP_Location_Value, client_Sensor_Location_Str[client_CP_Location_Value]);
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
    // Obtain a reference to the remote CSC service.
    pRemote_CyclingSpeedCadence_Service = pClient_Wahoo->getService(UUID16_SVC_CYCLING_SPEED_AND_CADENCE);
    if (pRemote_CyclingSpeedCadence_Service == nullptr) {
      DEBUG_PRINTLN("Client_CyclingSpeedCadence_Service: Not Found -> Not Mandatory!");
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
        DEBUG_PRINT(" -> Client Reads CSC Location Sensor:");
        DEBUG_PRINTF(" Loc#: %d %s\n", client_CSC_Location_Value, client_Sensor_Location_Str[client_CSC_Location_Value]);
      }
    }
    // Now Separately -> Notify Enable 
    if ( pRemote_CSC_Measurement_Chr != nullptr ) {  
      pRemote_CSC_Measurement_Chr->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true); 
      DEBUG_PRINTLN("Client_CSC_Measurement_Chr: Notify Enabled!");
    }
    return true;    
}



// This is NOT really a Callback --> It should have been implemented that way (see for instance Adafruit Bluefruit BLE library), 
// however, now it is called from loop() ... a poor man's solution!
bool client_Connect_Callback() {
    // Connect to the Wahoo BLE Server.
    pClient_Wahoo->connect(myDevice);  // if you pass BLEAdvertisedDevice it will recognize type of peer device address (public or private)     
    DEBUG_PRINTLN("Now checking all Client Services and Characteristics!");
    DEBUG_PRINTLN("If Mandatory Services Fail --> the Client will disconnect!");
  // Discover all relevant Services and Char's
  if( !client_GenericAccess_Connect() ) {
    pClient_Wahoo->disconnect();
    return false;    
  }
  if( !client_DeviceInformation_Connect() ) {
    pClient_Wahoo->disconnect();
    return false;    
  }
  if( !client_CyclingPower_Connect() ) {
    pClient_Wahoo->disconnect();
    return false;    
  }
 if( !client_CyclingSpeedCadence_Connect() ) {
    pClient_Wahoo->disconnect();
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
    // We have found a device, let us now see if it contains the CPS service we are looking for.
    if (advertisedDevice->haveServiceUUID() && advertisedDevice->isAdvertisingService(UUID16_SVC_CYCLING_POWER)) { // NIMBLE
      BLEDevice::getScan()->stop();
      myDevice = advertisedDevice; // NIMBLE --> Just save the reference now, no need to copy the object         
      /* Connect to the Wahoo BLE Server -> Sorry you can't do that here!!! --------------------------------
      ** pClient_Wahoo->connect(myDevice);  NOT ALLOWED TO CALL CONNECT --> CAUSES FATAL ERROR !!!! ???? */  
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

void Set_SimMode(void)
{
  // Setting simulation mode variables (LEN = 7) Weight: 72.0 RRC: 0.004000 WRC: 0.368000
  uint8_t cpwtData[] = { 0x43, 0x20, 0x1C, 0x04, 0x00, 0x70, 0x01 }; // Set Sim Mode command string
  DEBUG_PRINTLN("Client sends to Wahoo Control Point: [ 43 20 1C 04 00 70 01 ] --> set Weight: 72.0 RRC: 0.004000 WRC: 0.368000");
  // write with response !!! writeValue(uint8_t* data, size_t length, bool response = false);
  pRemote_Wahoo_ControlPoint_Chr->writeValue(cpwtData, 7, true);
}

void Set_SimGrade(float gr)
{
  // Grade is between -1 and +1 (div 100) and NOT in percentage !!!
  uint16_t norm =  uint16_t( (min((float)1, max((float)-1, gr/100)) + 1.0) * 65535 / 2.0 ) ; 
  uint8_t cpwtData[] = { 0x46, uint8_t(norm & 0xFF), uint8_t(norm >> 8 & 0xFF) }; 
  DEBUG_PRINT("Client sends to Wahoo Control Point: [ ");
  for (int i = 0; i < 3; i++) {
    if ( i <= sizeof(cpwtData)) {
      DEBUG_PRINTF("%02X ", cpwtData[i], HEX);
    }
  }
  DEBUG_PRINTF("] --> set Grade: %5.2f%%", gr);
  DEBUG_PRINTLN();
  // write with response !!! writeValue(uint8_t* data, size_t length, bool response = false);
  pRemote_Wahoo_ControlPoint_Chr->writeValue(cpwtData, 3, true);
}

void loop() {
  // If the flag "doClientConnectCall" is true, we have found the desired BLE server!
  // Once we are connected and ALL is set the clientIsConnected flag is set true.
  if (doClientConnectCall == true) {
    bool dummy = client_Connect_Callback();
    doClientConnectCall = false;
  } // doClientConnectCall

  if (clientIsConnected) {
    // If time is there, send test Simulation Parameters to the Wahoo Control Point...
    if(millis() > TimeInterval) {
        Grade += 1.0;     // Increase the simulated road grade
        Set_SimGrade(Grade); 
        if (Grade > 9.0) {
            Set_SimMode(); // Repeat every 10 events
            Grade = 0.0;
        }
        TimeInterval = millis() + CONTROL_POINT_TIME_SPAN;
    } // TimeInterval
  } else { // client is NOT connected check for (re)start scanning
    if(RestartScanningOnDisconnect) {
        pBLEScan->clearResults();   // delete results from BLEScan buffer to release memory    
        DEBUG_PRINTLN("Client Starts Scanning for Server Device with CPS + Wahoo and CSC (optional)!");
        RestartScanningOnDisconnect = false;        
        pBLEScan->start(0, false);
    }
  }      
} // End of loop