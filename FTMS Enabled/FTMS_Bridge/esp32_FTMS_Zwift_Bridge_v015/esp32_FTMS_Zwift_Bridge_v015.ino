/*********************************************************************
 This is programming code for ESP32 Espressif Wroom boards
 
      Tested with Adafruit Feather ESP32 V2 a.k.a. Huzzah
 
 The code uses heavily the supplied ESP32 NimBLE libraries !!          
      see: https://github.com/h2zero/NimBLE-Arduino
 Many have invested time and resources providing open source code!
 
        MIT license, check LICENSE for more information
        All text must be included in any redistribution
*********************************************************************/

/* -----------------------------------------------------------------------------------------------------
               This code should work with all indoor cycling trainers that fully support,
          Fitness Machine Service, Cycling Power Service and Cycling Speed & Cadence Service
   ------------------------------------------------------------------------------------------------------

    The code links a BLE Server (a peripheral to Zwift) and a BLE Client (a central to the Trainer) with a bridge
    in between, the ESP32 being man-in-the-middle (MITM).
    The ESP32-bridge can control, filter and alter the bi-directional interchanged data!
    The client-side (central) scans and connects with the trainer relevant services: FTMS, CPS and CSC. It collects
    all cyling data of the services and passes these on to the server-side....
    The client-side supplies the indoor trainer with target and resistance control data.
    The server-side (peripheral) advertises and enables connection with cycling apps like Zwift and collects the app's
    control commands, target and resistance data. It passes these on to the client-side....
    The server-side supplies the app with the generated cycling data in return.

    The client plus server (MITM) are transparent to the indoor trainer as well as to the training app Zwift or alike!

    Requirements: Zwift app or alike, ESP32 board and a FTMS/CPS/CSC supporting indoor trainer
    1) Upload and Run this code on a ESP32 board
    2) Start the Serial Monitor to catch debugging info
    3) Start/Power On the indoor trainer
    4) ESP32-bridge and trainer (with <name>) will pair as reported in the output
    5) Start Zwift on your computer or tablet and wait....
    6) Search on the Zwift pairing screens for <SIM32>
    7) Pair: Power, Cadence and Controllable one after another with <SIM32>
    8) Optionally one can pair as well devices for heartrate and/or steering (Sterzo)
    9) Start the default Zwift ride or any ride you wish
   10) Make Serial Monitor output window visible on top of the Zwift window
   11) Hop on the bike: do the work and feel resistance change with the road
   12) Inspect the info presented by Serial Monitor.....

    This device is identified with the name <SIM32>. You will see this only when connecting to Zwift on the 
    pairing screens! Notice: Zwift extends device names with additional numbers for identification!

*/
/*
Version 1.1
Changed Stack Depth values from 2048 to 4096 for Server Control Point Indicate (modified) and Write w Response
Version 1.2
Inserted check (boolean) on Write-Response out of synch...
Version 1.3 
Cycling Speed Cadence Service changed to NOT Mandatory
Version 1.4
Inserted checks on the input values of sensor location and location description array sizes (CP, CSC and HBM)
Version 1.5
Corrected invalid (enable/disable) setting of HR measurement Characteristic in client_Set_All_NotificationIndication()
Changed device identification naming to a simpler scheme: SIM32 or SIM52 instead of <SIM DevName>
Server Characteristic values (read only) are now updated when a new client connection is established
*/
// -------------------------------------------------------------------------------------------
// COMPILER DIRECTIVE to allow/suppress DEBUG messages that help debugging...
// Uncomment general "#define DEBUG" to activate
#define DEBUG
// Include these debug utility macros in all cases!
#include "DebugUtils.h"
#ifdef DEBUG
//  Restrict activating one or more of the following DEBUG directives --> process intensive 
//  The overhead can lead to spurious side effects and a loss of quality of service handling!!
//#define DEBUG_HBM               // If defined allows for parsing and decoding the Heart Beat Data
//#define DEBUG_CP_MEASUREMENT    // If defined allows for parsing and decoding the Cycling Power Data
//#define DEBUG_CSC_MEASUREMENT   // If defined allows for parsing and decoding the Cycling Speed and Cadence Data
//#define DEBUG_FTM_INDOORBIKEDATA// If defined allows for parsing the Indoor Bike Data
#ifdef DEBUG_FTM_INDOORBIKEDATA
//#define DEBUG_DECODE_IBD        // If defined allows for decoding the Indoor Bike Data
#endif
#define DEBUG_FTM_TRAININGSTATUS// If defined allows for parsing the Training Status Data
#define DEBUG_FTM_STATUS        // If defined allows for parsing the Machine Status Data
//#define DEBUG_FTM_CONTROLPOINT_RESPONSE     // If defined allows for parsing the Data
#define DEBUG_FTM_CONTROLPOINT_OPCODE_DATA  // If defined allows for parsing and decoding Data
#endif
// --------------------------------------------------------------------------------------------

#define 	BLE_APPEARANCE_GENERIC_CYCLING   1152
/*
#define 	BLE_APPEARANCE_CYCLING_CYCLING_COMPUTER   1153
#define 	BLE_APPEARANCE_CYCLING_SPEED_SENSOR   1154
#define 	BLE_APPEARANCE_CYCLING_CADENCE_SENSOR   1155
#define 	BLE_APPEARANCE_CYCLING_POWER_SENSOR   1156
#define 	BLE_APPEARANCE_CYCLING_SPEED_CADENCE_SENSOR   1157
*/
#include <NimBLEDevice.h>
// We need this for setting the Server-side Generic Access Char's --> Appearance and DeviceName
#include <nimble/nimble/host/services/gap/include/services/gap/ble_svc_gap.h>

#define THISDEVICENAME "SIM32" // Shortname 

const uint8_t MAX_PAYLOAD = 20; // Max 20 byte data size for single packet BLE transfer

// Struct containing Device info to administer dis/connected devices
typedef struct
{
  uint8_t PeerAddress[6];
  std::string PeerName;
  uint16_t conn_handle;
  bool IsConnected;
} Device_info_t;
// -----------------------------------------------------------------
// Your hardware MAC/DEVICE ADDRESSES
// Laptop/Desktop Device Address that runs Zwift, in printed format: [00:01:02:03:04:05]
// NimBLE demands you to enter addresses here in Little Endian format (reversed order)
#define LAPTOPADDRESS {0x05,0x04,0x03,0x02,0x01,0x00} // Little Endian format!!
// Trainer FTMS enabled Device Address, in printed format: [00:01:02:03:04:05]
#define TRAINERADDRESS {0x05,0x04,0x03,0x02,0x01,0x00} // Little Endian format!!
 
// -----------------------------------------------------------------
// Initialize connectable device registration
Device_info_t Trainer    = {TRAINERADDRESS, "MyTrainer", BLE_HS_CONN_HANDLE_NONE, false};
Device_info_t Laptop     = { LAPTOPADDRESS, "MyLaptop" , BLE_HS_CONN_HANDLE_NONE, false};
Device_info_t Smartphone = {        {0x00}, "MyPhone"  , BLE_HS_CONN_HANDLE_NONE, false};
// ----------------------------------------------------------------------------------

// Client Generic Access --------------------------------------------------------------
#define UUID16_SVC_GENERIC_ACCESS                             BLEUUID((uint16_t)0x1800)
#define UUID16_CHR_DEVICE_NAME                                BLEUUID((uint16_t)0x2A00)
#define UUID16_CHR_APPEARANCE                                 BLEUUID((uint16_t)0x2A01)
//#define UUID16_CHR_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS BLEUUID((uint16_t)0x2A04)
//#define UUID16_CHR_CENTRAL_ADDRESS_RESOLUTION                 BLEUUID((uint16_t)0x2AA6)
BLERemoteService* pRemote_GenericAccess_Service;
BLERemoteCharacteristic* pRemote_GA_Appearance_Chr; // Read
uint16_t client_GA_Appearance_Value = BLE_APPEARANCE_GENERIC_CYCLING;  // Default decimal: 1152 -> Generic Cycling
BLERemoteCharacteristic* pRemote_GA_DeviceName_Chr;// Read, Write
std::string client_GA_DeviceName_Str = THISDEVICENAME;

// Client Service Device Information --------------------------------------------------
#define UUID16_SVC_DEVICE_INFORMATION                         BLEUUID((uint16_t)0x180A)
#define UUID16_CHR_MODEL_NUMBER_STRING                        BLEUUID((uint16_t)0x2A24)
#define UUID16_CHR_SERIAL_NUMBER_STRING                       BLEUUID((uint16_t)0x2A25)
#define UUID16_CHR_MANUFACTURER_NAME_STRING                   BLEUUID((uint16_t)0x2A29)
//#define UUID16_CHR_FIRMWARE_REVISION_STRING                   BLEUUID((uint16_t)0x2A26)
//#define UUID16_CHR_HARDWARE_REVISION_STRING                   BLEUUID((uint16_t)0x2A27)
//#define UUID16_CHR_SOFTWARE_REVISION_STRING                   BLEUUID((uint16_t)0x2A28)
BLERemoteService*  pRemote_DeviceInformation_Service; 
BLERemoteCharacteristic* pRemote_DIS_ManufacturerName_Chr;   // Read
std::string client_DIS_Manufacturer_Str;
BLERemoteCharacteristic* pRemote_DIS_ModelNumber_Chr;       // Read
std::string client_DIS_ModelNumber_Str;
BLERemoteCharacteristic* pRemote_DIS_SerialNumber_Chr;      // Read
std::string client_DIS_SerialNumber_Str;
// -------------------------------------------------------------------------------------
BLEService *server_DeviceInformation_Service;
BLECharacteristic *server_DIS_ModelNumber_Chr;       // Read
BLECharacteristic *server_DIS_SerialNumber_Chr;      // Read
BLECharacteristic *server_DIS_ManufacturerName_Chr;  // Read
/*
//BLECharacteristic *server_DIS_Firmware_Chr;          // Read
//std::string client_DIS_Firmware_Str = "12345";
//BLECharacteristic *server_DIS_Hardware_Chr;          // Read
//std::string client_DIS_Hardware_Str = "12345";
//BLECharacteristic *server_DIS_Software_Chr;          // Read
//std::string client_DIS_Software_Str = "12345";
*/
//--------------------------------------------------------------------------------------

/* NORDIC UART SERVICE a.k.a. NUS
 * NUS Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 * NUS RXD    : 6E400002-B5A3-F393-E0A9-E50E24DCCA9E
 * NUS TXD    : 6E400003-B5A3-F393-E0A9-E50E24DCCA9E
 */
BLEUUID UUID_NUS_SERVICE("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLEUUID UUID_NUS_CHR_RXD("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
BLEUUID UUID_NUS_CHR_TXD("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");
BLEService *server_NordicUart_Service; 
BLECharacteristic *server_NUS_Rxd_Chr;        // Write No Response (Receiving Data)
BLECharacteristic *server_NUS_Txd_Chr;        // Read Notify (Sending Data)

/* Cycling Power Service ---------------------------------------------------------------
 * CP Service: 0x1818  
 * CP Characteristic: 0x2A63 (Measurement)    Mandatory
 * CP Characteristic: 0x2A65 (Feature)        Mandatory
 * CP Characteristic: 0x2A5D (Location)       Optional
 * CP Characteristic: 0x2A66 (Control Point)  Optional
 */
#define UUID16_SVC_CYCLING_POWER                              BLEUUID((uint16_t)0x1818)
#define UUID16_CHR_CYCLING_POWER_MEASUREMENT                  BLEUUID((uint16_t)0x2A63)
//#define UUID16_CHR_CYCLING_POWER_VECTOR                       BLEUUID((uint16_t)0x2A64)
#define UUID16_CHR_CYCLING_POWER_FEATURE                      BLEUUID((uint16_t)0x2A65)
//#define UUID16_CHR_CYCLING_POWER_CONTROL_POINT                BLEUUID((uint16_t)0x2A66)
#define UUID16_CHR_SENSOR_LOCATION                            BLEUUID((uint16_t)0x2A5D) // shared with CSC
BLERemoteService*        pRemote_CyclingPower_Service;
BLERemoteCharacteristic* pRemote_CP_Measurement_Chr;    // Notify, Read
BLERemoteCharacteristic* pRemote_CP_Feature_Chr;        // Read
uint32_t client_CP_Feature_Flags = 0;
BLERemoteCharacteristic* pRemote_CP_Location_Chr;       // Read
uint8_t client_CP_Location_Value = {0x0C};              // --> rear wheel !
BLERemoteCharacteristic* pRemote_CP_ControlPoint_Chr;   // Indicate, Write
bool client_ControlPoint_Response = true;   // CPWrite CPIndicate synch test
// ---------------------------------------------------------------------------------------
BLEService        *server_CyclingPower_Service;
BLECharacteristic *server_CP_Measurement_Chr; //                          Notify, Read
BLECharacteristic *server_CP_Feature_Chr; //                              Read
BLECharacteristic *server_CP_Location_Chr; //                             Read
//BLECharacteristic *server_CP_ControlPoint_Chr; //                       Indicate, Write
// ---------------------------------------------------------------------------------------

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
#define UUID16_SVC_CYCLING_SPEED_AND_CADENCE                  BLEUUID((uint16_t)0x1816)
//#define UUID16_CHR_CSC_CONTROL_POINT                          BLEUUID((uint16_t)0x2A55)
#define UUID16_CHR_CSC_MEASUREMENT                            BLEUUID((uint16_t)0x2A5B)
#define UUID16_CHR_CSC_FEATURE                                BLEUUID((uint16_t)0x2A5C)
//#define UUID16_CHR_SENSOR_LOCATION                            BLEUUID((uint16_t)0x2A5D) // shared with CP
BLERemoteService*        pRemote_CyclingSpeedCadence_Service;
BLERemoteCharacteristic* pRemote_CSC_Measurement_Chr;         // Notify, Read
BLERemoteCharacteristic* pRemote_CSC_Feature_Chr;             // Read
const uint8_t CSC_FEATURE_FIXED_DATALEN = 2; // UINT16
uint16_t client_CSC_Feature_Flags = 0;
const uint8_t client_CSC_Feature_Len = 3;
const char* client_CSC_Feature_Str[client_CSC_Feature_Len] = {"Wheel rev supported", "Crank rev supported", "Multiple locations supported"};
BLERemoteCharacteristic* pRemote_CSC_Location_Chr;             // Read
uint8_t client_CSC_Location_Value = {0x0C};                    // Default --> rear wheel !
// ---------------------------------------------------------------------------------------
BLEService        *server_CyclingSpeedCadence_Service;  
BLECharacteristic *server_CSC_Measurement_Chr;               //     Notify, Read
BLECharacteristic *server_CSC_Feature_Chr;                   //     Read
BLECharacteristic *server_CSC_Location_Chr;                  //     Read
// ---------------------------------------------------------------------------------------

/* HRM Service Definitions -------------------------------------------------------------
 * Heart Rate Monitor Service:  0x180D
 * Heart Rate Measurement Char: 0x2A37 (Mandatory)
 * Body Sensor Location Char:   0x2A38 (Optional)
 */
#define UUID16_SVC_HEART_RATE             BLEUUID((uint16_t)0x180D)
#define UUID16_CHR_HEART_RATE_MEASUREMENT BLEUUID((uint16_t)0x2A37)
#define UUID16_CHR_BODY_SENSOR_LOCATION   BLEUUID((uint16_t)0x2A38)
BLERemoteService* pRemote_HeartRate_Service;
BLERemoteCharacteristic* pRemote_HR_Measurement_Chr;
BLERemoteCharacteristic* pRemote_HR_Location_Chr;
/*  HR Body Sensor Location
    0x00 Other
    0x01 Chest
    0x02 Wrist
    0x03 Finger
    0x04 Hand
    0x05 Ear Lobe
    0x06 Foot
    0x07–0xFF Reserved for Future Use
*/
uint8_t client_HR_Location_Value= { 0x01 }; // Chest
// --------------------------------------------------------------------------------------
BLEService *server_HeartRate_Service;
BLECharacteristic *server_HR_Measurement_Chr;  // Notify Write
BLECharacteristic *server_HR_Location_Chr;     // Read
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
BLERemoteService* pRemote_FitnessMachine_Service; // FTM Service
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
BLERemoteCharacteristic* pRemote_FTM_TrainingStatus_Chr; //  Training Status, optional, read & notify
BLERemoteCharacteristic* pRemote_FTM_Status_Chr; //  Fitness Machine Status, mandatory, notify
BLERemoteCharacteristic* pRemote_FTM_IndoorBikeData_Chr; //  Indoor Bike Data, optional, notify
BLERemoteCharacteristic* pRemote_FTM_ControlPoint_Chr; //  Fitness Machine Control Point, optional, write & indicate
// ---------------------------------------------------------------------------------------
BLEService        *server_FitnessMachine_Service; // FTM Service
BLECharacteristic *server_FTM_Feature_Chr; //  Fitness Machine Feature, mandatory, read
BLECharacteristic *server_FTM_Status_Chr; //  Fitness Machine Status, mandatory, notify
BLECharacteristic *server_FTM_IndoorBikeData_Chr;  //  Indoor Bike Data, optional, notify
BLECharacteristic *server_FTM_ControlPoint_Chr; //  Fitness Machine Control Point, optional, write & indicate
BLECharacteristic *server_FTM_TrainingStatus_Chr; //  Training Status, optional, read & notify
BLECharacteristic *server_FTM_SupportedResistanceLevelRange_Chr; // Supported Resistance Level, read, optional
BLECharacteristic *server_FTM_SupportedPowerRange_Chr; // Supported Power Levels, read, optional
// ---------------------------------------------------------------------------------------
/**
 * Fitness Machine Control Point opcodes 
 * 
 * LSO: uint8 Op Code
 * MSO: 0..18 octets Parameters
 */
const uint8_t ftmcpRequestControl = 0x00;
const uint8_t ftmcpReset = 0x01;
const uint8_t ftmcpSetTargetSpeed = 0x02;
const uint8_t ftmcpSetTargetInclination = 0x03;
const uint8_t ftmcpSetTargetResistanceLevel = 0x04;
const uint8_t ftmcpSetTargetPower = 0x05;
const uint8_t ftmcpSetTargetHeartRate = 0x06;
const uint8_t ftmcpStartOrResume = 0x07;
const uint8_t ftmcpStopOrPause = 0x08;
const uint8_t ftmcpSetTargetedExpendedEngery = 0x09;
const uint8_t ftmcpSetTargetedNumberOfSteps = 0x0A;
const uint8_t ftmcpSetTargetedNumberOfStrided = 0x0B;
const uint8_t ftmcpSetTargetedDistance = 0x0C;
const uint8_t ftmcpSetTargetedTrainingTime = 0x0D;
const uint8_t ftmcpSetTargetedTimeInTwoHeartRateZones = 0x0E;
const uint8_t ftmcpSetTargetedTimeInThreeHeartRateZones = 0x0F;
const uint8_t ftmcpSetTargetedTimeInFiveHeartRateZones = 0x10;
const uint8_t ftmcpSetIndoorBikeSimulationParameters = 0x11;
const uint8_t ftmcpSetWheelCircumference = 0x12;
const uint8_t ftmcpSetSpinDownControl = 0x13;
const uint8_t ftmcpSetTargetedCadence = 0x14;

/** 
 * The Fitness Machine Control Point data type structure 
 * 
 */
const uint8_t FTM_CONTROL_POINT_DATALEN = 19; // Control point consists of 1 opcode (byte) and maximum 18 bytes as parameters
// This ftmcp_data_t structure represents the control point data. The first octet represents the opcode of the request
// followed by a parameter array of maximum 18 octects
typedef struct __attribute__( ( packed ) )
{
  uint8_t OPCODE;
  uint8_t OCTETS[(FTM_CONTROL_POINT_DATALEN-1)];
} ftmcp_data_t;

typedef union // The union type automatically maps the bytes member array to the ftmcp_data_t structure member values
{
  ftmcp_data_t values;
  uint8_t bytes[FTM_CONTROL_POINT_DATALEN];
} ftmcp_data_ut;

// Fitness Machine Control Point Data variable
ftmcp_data_ut server_FTM_Control_Point_Data;

/* Control Point: Common Response Buffers
const uint8_t ftmcpResponseCode = 0x80;
const uint8_t ftmcpRespConfirm = 0x01; // Ok!
const uint8_t ftmcpRespUnknown = 0x02; // Unknown OpCode
unsigned char ftmcpRespConfirmBuffer[3] = {ftmcpResponseCode, 0x00, ftmcpRespConfirm};
unsigned char ftmcpRespUnknownBuffer[3] = {ftmcpResponseCode, 0x00, ftmcpRespUnknown};
*/

// Global variables for decoding of Control Point: INDOOR BIKE DATA RESISTANCE PARAMETERS
float wind_speed = 0;       // meters per second, resolution 0.001
float grade = 0;            // percentage, resolution 0.01
float crr = 0;              // Coefficient of rolling resistance, resolution 0.0001
float cw = 0;               // Wind resistance Kg/m, resolution 0.01;
// ----------------------------------------------------------------------------------------------

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

// --------------------------------------------------------------------------------------
BLEClient* pClient_FTMS = nullptr;
BLEAdvertisedDevice* myDevice = nullptr;
BLEScan* pBLEScan = nullptr;
BLEServer* pServer = nullptr;
NimBLEAdvertising *pAdvertising = nullptr;
TaskHandle_t TaskWriteWithResponseHandle = NULL;

// These variables are handled in loop() to start sort of Callback functions
boolean doClientConnectCall = false;
boolean RestartScanningOnDisconnect = false;
boolean DoCallClientEnable = false;
boolean DoCallClientDisable = false;

// Values used to enable or disable notifications/indications
const uint8_t notificationOff[] = {0x0, 0x0};
const uint8_t notificationOn[] = {0x1, 0x0};
const uint8_t indicationOff[] = {0x0, 0x0};
const uint8_t indicationOn[] = {0x2, 0x0};
// ---------------------------------------------------------------------------------------
#define CONTROL_POINT_TIME_SPAN 2000    // Time span for sending Control Point data
unsigned long TimeInterval = 0;
// ---------------------------------------------------------------------------------
// Client Connect and Disconnect callbacks defined
class client_Connection_Callbacks:public BLEClientCallbacks {
  void onConnect(BLEClient* pClient);
  void onDisconnect(BLEClient* pClient);
  bool onConnParamsUpdateRequest(BLEClient* pClient, ble_gap_upd_params *params);  
};

// Server Connect and Disconnect callbacks defined
class server_Connection_Callbacks:public BLEServerCallbacks {
  void onConnect(BLEServer* pServer, ble_gap_conn_desc* desc);
  void onDisconnect(BLEServer* pServer, ble_gap_conn_desc* desc);
  void onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc);
};
// Handler class for Server Multi Characteristic action(s) defined
class CharacteristicCallbacks:public NimBLECharacteristicCallbacks {
/*  We only define onSubscribe !!!
    void onRead(NimBLECharacteristic* pCharacteristic);
    void onWrite(NimBLECharacteristic* pCharacteristic);
    void onNotify(NimBLECharacteristic* pCharacteristic);    
    void onStatus(NimBLECharacteristic* pCharacteristic, Status status, int code);
*/    
    void onSubscribe(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue);
};    
// Define CharacteristicCallbacks instance(s) globally to use for multiple Server Characteristics 
static CharacteristicCallbacks server_Multi_Chr_Callbacks;

void ConvertMacAddress(char *fullAddress, uint8_t addr[6], bool NativeFormat);
void server_setupGA(void);
void server_setupDIS(void);
void server_setupHRM(void);
void server_setupNUS(void);
void server_setupFTMS(void);
void server_setupCSC(void);
void server_setupCPS(void);
void server_startADV(void);
void client_Start_Scanning(void);
bool client_Connect_Callback(void);
// ---------------------------------------------------------------------------------

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
  while (!Serial) delay(10); 
  Serial.flush();
  delay(1000); // Give Serial I/O time to settle
#endif 
  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("ESP32 NimBLE MITM supporting: CPS, CSC, HBM and FTMS");
  DEBUG_PRINTLN("------------------ Version 01.5 --------------------");
  DEBUG_PRINTLN(THISDEVICENAME);
  delay(200);
  BLEDevice::init(THISDEVICENAME); // Give the device a Shortname  
  // Start the Server-side now!
  pServer = BLEDevice::createServer();
  //Setup callbacks onConnect and onDisconnect
  pServer->setCallbacks(new server_Connection_Callbacks());
  // Set server auto-restart advertise on
  pServer->advertiseOnDisconnect(true); 
  // Server setup
  DEBUG_PRINTLN("Configuring the default Generic Access Service");
  server_setupGA();
  DEBUG_PRINTLN("Configuring the Server Nordic Uart Service");  
  server_setupNUS();
  DEBUG_PRINTLN("Configuring the Server Device Information Service");
  server_setupDIS();
  DEBUG_PRINTLN("Configuring the Server Cycle Power Service");
  server_setupCPS();
  DEBUG_PRINTLN("Configuring the Server Cadence and Speed Service");  
  server_setupCSC();
  DEBUG_PRINTLN("Configuring the Server Fitness Machine Service");  
  server_setupFTMS();
  DEBUG_PRINTLN("Configuring the Server Heart Rate Service");  
  server_setupHRM();
  DEBUG_PRINTLN("Setting up the Server advertising payload(s)");
  server_startADV();
  DEBUG_PRINTLN("Server is advertising: CPS, CSC and FTMS");    
    
  // Start the Client-side!
  client_Start_Scanning();
  if(doClientConnectCall) {
    doClientConnectCall = false;
    bool dummy = client_Connect_Callback();
  }
  if(!Trainer.IsConnected) {
    DEBUG_PRINTLN(">>> Failed to connect Trainer! Reset ESP32 and try again!");
    while(1) {delay(100);}
  }
} // End of setup.

void ConvertMacAddress(char *fullAddress, uint8_t addr[6], bool NativeFormat)
{ // Display byte by byte in HEX 
  if(NativeFormat) { // Unaltered: in Little Endian machine-representation
    sprintf(fullAddress, "%02X:%02X:%02X:%02X:%02X:%02X", addr[0], addr[1], addr[2], \
      addr[3], addr[4], addr[5], HEX);   
  } else { // Altered: In reversed order
    sprintf(fullAddress, "%02X:%02X:%02X:%02X:%02X:%02X", addr[5], addr[4], addr[3], \
      addr[2], addr[1], addr[0], HEX);       
  }
};

void client_HR_Measurement_Notify_Callback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  // Client HR Measurement data is tranferred to the Server (Zwift)
  // NO TREATMENT OF RESPONSE !!!!!
  if((Laptop.IsConnected)) { 
    server_HR_Measurement_Chr->setValue(pData, length);
    server_HR_Measurement_Chr->notify(); // Just pass on and process later!
  }
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
            server_DIS_ManufacturerName_Chr->setValue(client_DIS_Manufacturer_Str); // Tranfer/Update the value to the server side
            DEBUG_PRINTF(" -> Client Reads Manufacturer Name: [%s]\n", client_DIS_Manufacturer_Str.c_str());
          }            
      }     
      pRemote_DIS_ModelNumber_Chr = pRemote_DeviceInformation_Service->getCharacteristic(UUID16_CHR_MODEL_NUMBER_STRING);       
      if ( pRemote_DIS_ModelNumber_Chr != nullptr ) {
          if(pRemote_DIS_ModelNumber_Chr->canRead()) {
            client_DIS_ModelNumber_Str = pRemote_DIS_ModelNumber_Chr->readValue();
            server_DIS_ModelNumber_Chr->setValue(client_DIS_ModelNumber_Str); // Tranfer/Update the value to the server side
            DEBUG_PRINTF(" -> Client Reads Model Number:      [%s]\n", client_DIS_ModelNumber_Str.c_str());
          }
      }  
      pRemote_DIS_SerialNumber_Chr = pRemote_DeviceInformation_Service->getCharacteristic(UUID16_CHR_SERIAL_NUMBER_STRING);       
      if ( pRemote_DIS_SerialNumber_Chr != nullptr ) {
          if(pRemote_DIS_SerialNumber_Chr->canRead()) {
            client_DIS_SerialNumber_Str = pRemote_DIS_SerialNumber_Chr->readValue();
            server_DIS_SerialNumber_Chr->setValue(client_DIS_SerialNumber_Str); // Tranfer/Update the value to the server side
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
            int RespErr = ble_svc_gap_device_name_set((const char*)client_GA_DeviceName_Str.c_str()); // Tranfer/Update the value to the server side    
            DEBUG_PRINTF(" -> Client Reads Device Name:   [%s]\n", client_GA_DeviceName_Str.c_str());
          }            
      }     
      pRemote_GA_Appearance_Chr = pRemote_GenericAccess_Service->getCharacteristic(UUID16_CHR_APPEARANCE);       
      if ( pRemote_GA_Appearance_Chr != nullptr ) {
          if(pRemote_GA_Appearance_Chr->canRead()) {
            client_GA_Appearance_Value = pRemote_GA_Appearance_Chr->readUInt16();
            int RespErr = ble_svc_gap_device_appearance_set(client_GA_Appearance_Value); // Tranfer/Update the value to the server side
            DEBUG_PRINTF(" -> Client Reads Appearance:    [%d]\n", client_GA_Appearance_Value);
          }
      }     
    return true;     
}

void client_CP_Measurement_Notify_Callback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) 
{
  // Client CP Measurement data is tranferred to the Server
  // NO TREATMENT OF COMMAND !!!
  if((Laptop.IsConnected)) {   
    server_CP_Measurement_Chr->setValue(pData, length);
    server_CP_Measurement_Chr->notify(); // Just pass on and process later!
  }
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
      server_CP_Feature_Chr->setValue(client_CP_Feature_Flags); // Tranfer/Update the value to the server side
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
        server_CP_Location_Chr->setValue(&client_CP_Location_Value, 1); // Tranfer/Update the value to the server side
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
    return true;    
}

void client_CSC_Measurement_Notify_Callback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify)
{
  // Client CSC Measurement data is tranferred to the Server
  // NO TREATMENT OF COMMAND !!!
  if((Laptop.IsConnected)) {   
    server_CSC_Measurement_Chr->setValue(pData, length);
    server_CSC_Measurement_Chr->notify(); // Just pass on and process later!
  }  
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
        server_CSC_Feature_Chr->setValue(client_CSC_Feature_Flags); // Tranfer/Update the value to the server side
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
        server_CSC_Location_Chr->setValue(&client_CSC_Location_Value, 1); // Tranfer/Update the value to the server side
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
        server_HR_Location_Chr->setValue(&client_HR_Location_Value, 1); // Tranfer/Update the value to the server side
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
    return true;    
}

void client_FTM_TrainingStatus_Notify_Callback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify)
{
  // Client FTM Training Status data is tranferred to the Server
  // NO TREATMENT OF COMMAND !!!
  if((Laptop.IsConnected)) { 
    server_FTM_TrainingStatus_Chr->setValue(pData, length);
    server_FTM_TrainingStatus_Chr->notify(); // Just pass on and process later!
  }
#ifdef DEBUG_FTM_TRAININGSTATUS
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
  // Client FTM Status data is tranferred to the Server
  // NO TREATMENT OF COMMAND !!!
  if((Laptop.IsConnected)) { 
    server_FTM_Status_Chr->setValue(pData, length);
    server_FTM_Status_Chr->notify(); // Just pass on and process later!
  }  
#ifdef DEBUG_FTM_STATUS
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
  // Client FTM Indoor Bike Data is tranferred to the Server
  // NO TREATMENT OF COMMAND !!!
  if((Laptop.IsConnected)) { 
    server_FTM_IndoorBikeData_Chr->setValue(pData, length);
    server_FTM_IndoorBikeData_Chr->notify(); // Just pass on and process later!
  }  
#ifdef DEBUG_FTM_INDOORBIKEDATA
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
#ifdef DEBUG_DECODE_IBD
  uint8_t offset = 0;
  uint16_t flags = 0;
  memcpy(&flags, &IBDDataBuf[offset], 2); // Tranfer/Update buffer fields to variable
  offset += 2;  // UINT16
  if ((flags & 1) == 0) { // Inst. Speed is present (!) if flag (bit 0) is set to 0 (null)
    //  More Data --> Instantaneous Speed 0.01 
    uint16_t inst_Speed = 0;
    memcpy(&inst_Speed, &IBDDataBuf[offset], 2); // Tranfer/Update buffer fields to variable
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
  // The receipt of Control Point settings is acknowledged by the trainer: handle it
  // Send Client's Response message to the Server
  // NO TREATMENT OF COMMAND !!!
  if((Laptop.IsConnected)) { 
    server_FTM_ControlPoint_Chr->setValue(pData, length);
    server_FTM_ControlPoint_Chr->indicate(); // Just pass on and process later!
    client_ControlPoint_Response = true; // Should be set now!    
  }  
#ifdef DEBUG_FTM_CONTROLPOINT_RESPONSE
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
        server_FTM_Feature_Chr->setValue(client_FTM_Feature_Str); // Tranfer/Update the value to the server side
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
        server_FTM_SupportedResistanceLevelRange_Chr->setValue(client_FTM_SupportedResistanceLevelRange_Str); // Tranfer/Update the value to the server side
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
        server_FTM_SupportedPowerRange_Chr->setValue(client_FTM_SupportedPowerRange_Str); // Tranfer/Update the value to the server side
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
      pRemote_FTM_ControlPoint_Chr->registerForNotify(client_FTM_ControlPoint_Indicate_Callback, true, true); // Notifications true means Indicate is set false!!!!
    } else {
      DEBUG_PRINTLN("Mandatory Client_FTM_ControlPoint_Chr: Cannot Indicate!");
      return false; // Mandatory when service is present
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
return true;
} // end Fitness Machine SVC

// This is NOT really a Callback --> It should have been implemented that way (see for instance Adafruit Bluefruit BLE library), 
// however, now it is called from loop() ... a poor man's solution!
bool client_Connect_Callback(void) {
    // Every time we want to connect to a Server a NEW Client is created !!
    pClient_FTMS = BLEDevice::createClient(); 
    pClient_FTMS->setClientCallbacks(new client_Connection_Callbacks());
    // Connect to the FTMS BLE Server.
    pClient_FTMS->connect(myDevice);
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
  // When the client/trainer is RECONNECTING we need to enable/indicate all Remote Client Char's again!
  if(Laptop.IsConnected) {
    //Do NOT(!) allow for any possible delay (Regularly this is handled in loop() with DoCallClientEnable = true)
    client_Set_All_NotificationIndication(true);
  }
  // ----------------------------------------------------------------------------------------------*/
  Trainer.IsConnected = true;
  client_ControlPoint_Response = true; // Should be set now!  
return true;
};

/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
/*
 * Called for each advertising BLE server.
*/
  void onResult(BLEAdvertisedDevice* advertisedDevice) {
    //DEBUG_PRINT("Advertising Device-> ");
    //DEBUG_PRINTLN(advertisedDevice.toString().c_str());
    // We have found a server device, now see if it contains the FTMS service we are looking for.
    if (advertisedDevice->haveServiceUUID() && advertisedDevice->isAdvertisingService(UUID16_SVC_FITNESS_MACHINE)) {
      NimBLEAddress MyAddress = advertisedDevice->getAddress();
      uint8_t RemoteAddress[6] = {};
      memcpy(&RemoteAddress, MyAddress.getNative(), 6);
      DEBUG_PRINTLN("Found advertising Peripheral with FTMS enabled! See data:");
      DEBUG_PRINTLN(advertisedDevice->toString().c_str());
      // OK Server has FTMS service exposed, now check for right mac adress 
      if ( !(memcmp((void*)RemoteAddress, Trainer.PeerAddress, 6) == 0) ) {
        char fullMacAddress[18] = {}; //
        ConvertMacAddress(fullMacAddress, RemoteAddress, false); // true -> Native representation!
        DEBUG_PRINTF("Keep Scanning! Unknown Trainer Mac Address: [%s]\n", fullMacAddress);
        return;
      }      
      BLEDevice::getScan()->stop();
      myDevice = advertisedDevice;
      /* Connect to the FTMS BLE Server -> Sorry you can't do that here!!! --------------------------------
      ** pClient_FTMS->connect(myDevice);  NOT ALLOWED TO CALL CONNECT --> CAUSES FATAL ERROR !!!! ???? */  
      doClientConnectCall = true;         // Work around via loop()           
    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks

void client_Connection_Callbacks::onConnect(BLEClient* pClient) {
    Trainer.PeerName = myDevice->getName().c_str();
    Trainer.conn_handle = pClient_FTMS->getConnId();
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
    ConvertMacAddress(fullMacAddress, Trainer.PeerAddress, false); // true -> Native representation!
    DEBUG_PRINTF("ESP32 Client connected to Server device with Name: [%s] MAC Address: [%s] Handle: [%d] MTU: [%d]\n", \
                                                          Trainer.PeerName.c_str(), fullMacAddress, Trainer.conn_handle, max_payload); 
#endif
    /** We can use the connection handle here to ask for different connection parameters.
     *  Args: connection handle, min connection interval, max connection interval, latency and supervision timeout.
     *  Units; Min/Max Intervals: 1.25 millisecond increments --> between: 24 (30 ms) resp. 48 (60 ms)
     *  Latency: number of intervals allowed to skip. --> keep zero
     *  Timeout: 10 millisecond increments. --> Try 400 (4000 ms)
     */ 
    //pClient_FTMS->updateConnParams(pClient_FTMS->getConnId(), 24, 48, 0, 400);
    //DEBUG_PRINTLN("Client Updates Connection Parameters -> Min Interval: [24] Max Interval: [48] Latency: [0] Supervision Timeout: [400]");     
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
    Trainer.IsConnected = false;
    Trainer.conn_handle = BLE_HS_CONN_HANDLE_NONE; 
    char fullMacAddress[18] = {}; //
    ConvertMacAddress(fullMacAddress, Trainer.PeerAddress, false); // true -> Native representation!
    DEBUG_PRINTF("Client Disconnected from Server device with Name: [%s] Mac Address: [%s]!\n",  Trainer.PeerName.c_str(), fullMacAddress); 
    RestartScanningOnDisconnect = true;
    //  It is an option to disconnect also the Server, however NOT necessary!! We choose not!
    //  if(Laptop.IsConnected) pServer->disconnect(Laptop.conn_handle);
};

void client_Start_Scanning(void)
{
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
   DEBUG_PRINTLN("Client Starts Scanning for Server Device with CPS, CSC and FTMS!");  
  //pBLEScan->start(5, false); // Scan for 5 seconds only
  pBLEScan->start(0, false);
}

// Handler class for Server Multi Characteristic actions limited to onSubscribe
void CharacteristicCallbacks::onSubscribe(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue) {
          std::string str = "Central (" + Laptop.PeerName + "/Zwift) Updated CCCD -->";
        if(subValue == 0) {
            str += " Notify/Indicate Disabled for Char:";
        }else if(subValue == 1) {
            str += " Notify Enabled for Char:";
        } else if(subValue == 2) {
            str += " Indicate Enabled for Char:";
        } else if(subValue == 3) {
            str += " Notify & Indicate Enabled for Char:";
        }
        DEBUG_PRINTF("%s", str.c_str());
        str = std::string(pCharacteristic->getUUID()).c_str();
        DEBUG_PRINTF(" [%s]\n", str.c_str());
};

void server_startADV(void)
{
    // Prepare for advertising
    /** Optional: set the transmit power, default is 3db */
#ifdef ESP_PLATFORM
    NimBLEDevice::setPower(ESP_PWR_LVL_P9); /** +9db */
#else
    NimBLEDevice::setPower(9); /** +9db */
#endif
    pAdvertising = NimBLEDevice::getAdvertising(); 
    pAdvertising->addServiceUUID(UUID16_SVC_CYCLING_POWER);
    pAdvertising->addServiceUUID(UUID16_SVC_CYCLING_SPEED_AND_CADENCE);
    pAdvertising->addServiceUUID(UUID16_SVC_FITNESS_MACHINE);
    pAdvertising->setAppearance(client_GA_Appearance_Value);
    /*
    pAdvertising->setName(client_GA_DeviceName_Str);
    */
    DEBUG_PRINTF("Setting Appearance in Advertised data to [%d]\n", client_GA_Appearance_Value);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinInterval(32); // in 0.625ms units, 0 = use default.
    pAdvertising->setMaxInterval(244); // in 0.625ms units, 0 = use default.
    BLEDevice::startAdvertising();    
    // Start Advertising 
}

void server_Connection_Callbacks::onConnect(BLEServer* pServer, ble_gap_conn_desc *desc) {
    // Get some connection parameters of the peer device.
    uint16_t serverConnectionHandle = desc->conn_handle;
    uint16_t serverConnectionInterval = desc->conn_itvl;   // Connection interval   
    uint16_t serverConnectionLatency = desc->conn_latency; // Connection latency
    uint16_t serverConnectionSupTimeout = desc->supervision_timeout;   // Connection supervision timeout
    uint8_t RemoteAddress[6];
    memcpy(&RemoteAddress, NimBLEAddress(desc->peer_id_addr).getNative(), 6);
    char fullMacAddress[18] = {}; 
    ConvertMacAddress(fullMacAddress, RemoteAddress, false); // true -> Native format!
    BLEDevice::stopAdvertising();
    DEBUG_PRINTF("Server Connection Parameters -> Interval: [%d] Latency: [%d] Supervision Timeout: [%d]\n",serverConnectionInterval, \
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
    // Who has been exactly connected?
    // [1] Laptop is connecting
    if (memcmp(RemoteAddress, Laptop.PeerAddress, 6) == 0 ) { // Check Laptop MAC address
      // Laptop/PC is connecting !
      memcpy(&Laptop.PeerAddress, RemoteAddress, 6);     
      Laptop.conn_handle = serverConnectionHandle;       
      Laptop.IsConnected = true;
      DEBUG_PRINTF("Central (%s/Zwift) has to set CP/CSC/FTMS CCCD Notify/Indicate (enable) and start....\n", Laptop.PeerName.c_str());
      DoCallClientEnable = true; // Laptop is connected allow now for the trainer to send data
      return; // We are done here!
    }
    // [2] Smartphone is connecting
    Smartphone.conn_handle = serverConnectionHandle;
    Smartphone.IsConnected = true;
    memcpy(Smartphone.PeerAddress, RemoteAddress, 6);
    DEBUG_PRINTF("Central (%s/Simcline App) has to set NUS CCCD 'Notify' (enable) and start....\n", Smartphone.PeerName.c_str());
    /* Alternative for exclusively connecting to Laptop!!
    if ( !(memcmp(RemoteAddress, Laptop.PeerAddress, 6) == 0) ) {
        DEBUG_PRINTLN("ERROR >>> Forced Server Disconnect: Unknown Laptop Mac Address!");      
        pServer->disconnect(desc->conn_handle);        
        return; // Failed
    } 
    */
};

void server_Connection_Callbacks::onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc) {
        DEBUG_PRINTF("MTU updated: %u for connection ID: %u\n", MTU, desc->conn_handle);           
};

void server_Connection_Callbacks::onDisconnect(BLEServer* pServer, ble_gap_conn_desc *desc) {
    uint32_t count = pServer->getConnectedCount();
    // Get some Disconnection parameters of the peer device.
    uint16_t serverConnectionHandle = desc->conn_handle;
    uint8_t RemoteAddress[6] = {};
    memcpy(&RemoteAddress, NimBLEAddress(desc->peer_id_addr).getNative(), 6);
    char fullMacAddress[18] = {}; //
    ConvertMacAddress(fullMacAddress, RemoteAddress, false); // true -> Native format!
    if (Laptop.conn_handle == serverConnectionHandle ) { // Laptop/Desktop is disconnected
      Laptop.conn_handle = BLE_HS_CONN_HANDLE_NONE;
      Laptop.IsConnected = false;
      DEBUG_PRINTF("ESP32 Server disconnected from Central (%s) Conn handle: [%d] Mac Address: [%s]\n", Laptop.PeerName.c_str(), serverConnectionHandle, fullMacAddress);
      if(Trainer.IsConnected) DoCallClientDisable = true; // Tell the client not to send data!
    }
    if (Smartphone.conn_handle == serverConnectionHandle ) { // Smartphone is disconnected
      Smartphone.conn_handle = BLE_HS_CONN_HANDLE_NONE;
      Smartphone.IsConnected = false;
      DEBUG_PRINTF("ESP32 Server disconnected from Central (%s) Conn handle: [%d] Mac Address: [%s]\n", Smartphone.PeerName.c_str(), serverConnectionHandle, fullMacAddress);
    }
    DEBUG_PRINTLN(" --> ESP32 Server is advertising again!");
    // NimBLe does auto advertise after disconnect 
};

void server_setupCPS(void)
{
   server_CyclingPower_Service = pServer->createService(UUID16_SVC_CYCLING_POWER);
    server_CP_Measurement_Chr = server_CyclingPower_Service->createCharacteristic(UUID16_CHR_CYCLING_POWER_MEASUREMENT, 
                                                                            NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    server_CP_Measurement_Chr->setCallbacks(&server_Multi_Chr_Callbacks); //NIMBLE 
    server_CP_Feature_Chr = server_CyclingPower_Service->createCharacteristic(UUID16_CHR_CYCLING_POWER_FEATURE, 
                                                                            NIMBLE_PROPERTY::READ);
    // Set server CP Feature Flags field                                                                            
    server_CP_Feature_Chr->setValue(client_CP_Feature_Flags);                                                                            
    server_CP_Location_Chr = server_CyclingPower_Service->createCharacteristic(UUID16_CHR_SENSOR_LOCATION, 
                                                                            NIMBLE_PROPERTY::READ);
    // Set server_CP_Location for sensor
    server_CP_Location_Chr->setValue(&client_CP_Location_Value, 1);
    server_CyclingPower_Service->start();   
}

void server_setupCSC(void)
{
   server_CyclingSpeedCadence_Service = pServer->createService(UUID16_SVC_CYCLING_SPEED_AND_CADENCE);
    server_CSC_Measurement_Chr = server_CyclingSpeedCadence_Service->createCharacteristic(UUID16_CHR_CSC_MEASUREMENT, 
                                                                            NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    server_CSC_Measurement_Chr->setCallbacks(&server_Multi_Chr_Callbacks); //NIMBLE 
    server_CSC_Feature_Chr = server_CyclingSpeedCadence_Service->createCharacteristic(UUID16_CHR_CSC_FEATURE, 
                                                                            NIMBLE_PROPERTY::READ);
    // Set server CSC Feature Flags field                                                                            
    server_CSC_Feature_Chr->setValue(client_CSC_Feature_Flags);                                                                            
    server_CSC_Location_Chr = server_CyclingSpeedCadence_Service->createCharacteristic(UUID16_CHR_SENSOR_LOCATION, 
                                                                            NIMBLE_PROPERTY::READ);
    // Set server_CSC_Location for sensor
    server_CSC_Location_Chr->setValue(&client_CSC_Location_Value, 1);
    server_CyclingSpeedCadence_Service->start();    
}

std::string ftmcpData; // Defined global to easely passing std::string 

void TaskWriteWithResponse(void *parameter) {
  // Just pass on and process later! 
  if( !pRemote_FTM_ControlPoint_Chr->writeValue(ftmcpData, true) ) { // true -> WithResponse (fatal if trainer is not responding: Guru paniced!!)
      pClient_FTMS->disconnect();
      DEBUG_PRINTLN(">>> Error: NOT responding to FTM Control Point -> Write Value!");
  }
  vTaskDelete(TaskWriteWithResponseHandle);
};

class server_FTM_ControlPoint_Chr_callback: public BLECharacteristicCallbacks {
void onWrite(BLECharacteristic *pCharacteristic) {
  if(!client_ControlPoint_Response) {  // skip the next Write action --> out of write-response-synch !! 
    DEBUG_PRINTLN("-> Server Rec'd Raw Control Point Data --> SKIPPED: Out of Synch!");
    return;
  }
  ftmcpData = server_FTM_ControlPoint_Chr->getValue();
  uint8_t ftmcpDataLen = ftmcpData.length();
  // Server FTM Control Point data is tranferred to the Client
  // NO TREATMENT OF COMMAND !!!
  // write with response !!! writeValue(string, bool response = false);
  if(Trainer.IsConnected) { 
    xTaskCreate(&TaskWriteWithResponse, "Write w Response", 4096, (void *)NULL, 10, &TaskWriteWithResponseHandle); //2048
    client_ControlPoint_Response = false;  
  } 
  //
  memset(server_FTM_Control_Point_Data.bytes, 0, sizeof(server_FTM_Control_Point_Data.bytes));
  // Display the raw request packet
  // Transfer the contents of data to server_FTM_Control_Point_Data.bytes
  for (int i = 0; i < ftmcpDataLen; i++) {
      server_FTM_Control_Point_Data.bytes[i] = ftmcpData[i];
  }
#ifdef DEBUG_FTM_CONTROLPOINT_OPCODE_DATA  
  /// Decodes an incoming Fitness Machine Control Point request
  DEBUG_PRINTF(" --> Raw FTM Control Point Data [len: %d] ", ftmcpDataLen);
  DEBUG_PRINTF("[OpCode: %02X] [Values: ", server_FTM_Control_Point_Data.values.OPCODE, HEX);
  for (int i=0; i<ftmcpDataLen; i++) { 
    DEBUG_PRINTF("%02X ", server_FTM_Control_Point_Data.values.OCTETS[i], HEX); 
  }
  DEBUG_PRINTLN("]");
#endif    
    switch(server_FTM_Control_Point_Data.values.OPCODE) {
    case ftmcpRequestControl: {
      DEBUG_PRINTLN("Request Control of Machine!");
      break;
    }
    case ftmcpStartOrResume: {
      DEBUG_PRINTLN("Start or Resume Machine!");
      break;
    }
    case ftmcpStopOrPause: {
      DEBUG_PRINTLN("Stop or Pause Machine, Parameter: Stop!");
      break;
    }
    case ftmcpSetIndoorBikeSimulationParameters: {
      // Short is 16 bit signed, so the windspeed is converted from two bytes to signed value. Highest bit is sign bit
      short ws = (server_FTM_Control_Point_Data.values.OCTETS[0] << 8) + server_FTM_Control_Point_Data.values.OCTETS[1]; 
      wind_speed = ws / 1000.0;
      // Short is 16 bit signed, so a negative grade is correctly converted from two bytes to signed value. Highest bit is sign bit
      short gr = (server_FTM_Control_Point_Data.values.OCTETS[3] << 8) + server_FTM_Control_Point_Data.values.OCTETS[2]; 
      grade = gr / 100.0;
      crr = server_FTM_Control_Point_Data.values.OCTETS[4] / 10000.0;
      cw = server_FTM_Control_Point_Data.values.OCTETS[5] / 100.0;
      // Remember, if debugging with Zwift, that these values are divided by 2 if in normal 50% settings!
      DEBUG_PRINTLN("Set Indoor Bike Simulation Parameters!");
      DEBUG_PRINT("Wind speed (1000): "); DEBUG_PRINT(wind_speed);
      DEBUG_PRINT(" | Grade (100): "); DEBUG_PRINT(grade);
      DEBUG_PRINT(" | Crr (10000): "); DEBUG_PRINT(crr);
      DEBUG_PRINT(" | Cw (100): "); DEBUG_PRINTLN(cw);
      break;
    }
    case ftmcpReset: {
      DEBUG_PRINTLN("Reset Machine!");
      break;
    }
    case ftmcpSetTargetResistanceLevel:
    case ftmcpSetTargetSpeed:
    case ftmcpSetTargetInclination:
    case ftmcpSetTargetPower:
    case ftmcpSetTargetHeartRate:
    case ftmcpSetTargetedExpendedEngery:
    case ftmcpSetTargetedNumberOfSteps:
    case ftmcpSetTargetedNumberOfStrided:
    case ftmcpSetTargetedDistance:
    case ftmcpSetTargetedTrainingTime:
    case ftmcpSetTargetedTimeInTwoHeartRateZones:
    case ftmcpSetTargetedTimeInThreeHeartRateZones:
    case ftmcpSetTargetedTimeInFiveHeartRateZones:
    case ftmcpSetWheelCircumference:
    case ftmcpSetSpinDownControl:
    case ftmcpSetTargetedCadence: 
    {
      DEBUG_PRINTLN("Unresolved OpCode!");
      break;
    }
    } // switch
  } // onWrite
  
  void onSubscribe(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue) {
        std::string str = "Central (" + Laptop.PeerName + "/Zwift) Updated CCCD -->";
        if(subValue == 0) {
            str += " Notify/Indicate Disabled for Char:";
        }else if(subValue == 1) {
            str += " Notify Enabled for Char:";
        } else if(subValue == 2) {
            str += " Indicate Enabled for Char:";
        } else if(subValue == 3) {
            str += " Notify & Indicate Enabled for Char:";
        }
        DEBUG_PRINTF("%s", str.c_str());
        str = std::string(pCharacteristic->getUUID()).c_str();
        DEBUG_PRINTF(" [%s]\n", str.c_str());
  }; // onSubscribe

};

void server_setupFTMS(void)
{   
    server_FitnessMachine_Service = pServer->createService(UUID16_SVC_FITNESS_MACHINE);
    server_FTM_Feature_Chr = server_FitnessMachine_Service->createCharacteristic(UUID16_CHR_FITNESS_MACHINE_FEATURE, 
                                                                            NIMBLE_PROPERTY::READ);
    server_FTM_Feature_Chr->setValue(client_FTM_Feature_Str);

    server_FTM_Status_Chr = server_FitnessMachine_Service->createCharacteristic(UUID16_CHR_FITNESS_MACHINE_STATUS, 
                                                                            NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    server_FTM_Status_Chr->setCallbacks(&server_Multi_Chr_Callbacks); //NIMBLE    
    server_FTM_IndoorBikeData_Chr = server_FitnessMachine_Service->createCharacteristic(UUID16_CHR_INDOOR_BIKE_DATA, 
                                                                            NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    server_FTM_IndoorBikeData_Chr->setCallbacks(&server_Multi_Chr_Callbacks); //NIMBLE                                                                                                                                                           
    server_FTM_SupportedResistanceLevelRange_Chr = server_FitnessMachine_Service->createCharacteristic(UUID16_CHR_SUPPORTED_RESISTANCE_LEVEL_RANGE, 
                                                                            NIMBLE_PROPERTY::READ); 
    server_FTM_SupportedResistanceLevelRange_Chr->setValue(client_FTM_SupportedResistanceLevelRange_Str);
                                                                                 
    server_FTM_SupportedPowerRange_Chr = server_FitnessMachine_Service->createCharacteristic(UUID16_CHR_SUPPORTED_POWER_RANGE, 
                                                                            NIMBLE_PROPERTY::READ);
    server_FTM_SupportedPowerRange_Chr->setValue(client_FTM_SupportedPowerRange_Str); 
 
    server_FTM_TrainingStatus_Chr = server_FitnessMachine_Service->createCharacteristic(UUID16_CHR_TRAINING_STATUS, 
                                                                            NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    server_FTM_TrainingStatus_Chr->setCallbacks(&server_Multi_Chr_Callbacks); //NIMBLE 
    server_FTM_ControlPoint_Chr = server_FitnessMachine_Service->createCharacteristic(UUID16_CHR_FITNESS_MACHINE_CONTROL_POINT, 
                                                                            NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::INDICATE);
    server_FTM_ControlPoint_Chr->setCallbacks(new server_FTM_ControlPoint_Chr_callback);
    server_FitnessMachine_Service->start();      
}

class server_NUS_Rxd_Chr_callback: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    // Read data received over NUS Rxd from Mobile Phone
#ifdef DEBUG
    std::string NusRxdData = server_NUS_Rxd_Chr->getValue();
    uint8_t NusRxdDataLen = NusRxdData.length();  // Get the actual length of data bytes
    // Display the raw packet data in actual length
    DEBUG_PRINTF(" -> Server NUS Rxd Data [%d][%s]\n", NusRxdDataLen, NusRxdData.c_str());
#endif
  };
}; 

void server_setupNUS(void)
{
    server_NordicUart_Service = pServer->createService(UUID_NUS_SERVICE);
    server_NUS_Rxd_Chr = server_NordicUart_Service->createCharacteristic(UUID_NUS_CHR_RXD, 
                                                                            NIMBLE_PROPERTY::WRITE_NR); // Write with No response !!
    server_NUS_Rxd_Chr->setCallbacks(new server_NUS_Rxd_Chr_callback()); 
    server_NUS_Txd_Chr = server_NordicUart_Service->createCharacteristic(UUID_NUS_CHR_TXD, 
                                                                            NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    server_NUS_Txd_Chr->setCallbacks(&server_Multi_Chr_Callbacks); //NIMBLE
    server_NordicUart_Service->start();
}

void server_setupGA(void)
{
  // Set the Generic Access Appearance value from default: [0] --> Unknown to [1152] --> Generic Cycling
    int RespErr = ble_svc_gap_device_appearance_set(client_GA_Appearance_Value);
    if(RespErr == 0) {
      DEBUG_PRINTF("Successfully Set Generic Access Appearance Chr value to:  [%d] Generic Cycling\n", client_GA_Appearance_Value); 
    } else {
      DEBUG_PRINTLN("Unable to Set Generic Access Appearance Chr value!");      
    } 
  // Set Generic Access Device Name Chr to a value
    RespErr = ble_svc_gap_device_name_set((const char*)client_GA_DeviceName_Str.c_str());
    if(RespErr == 0) {
      DEBUG_PRINTF("Successfully Set Generic Access Device Name Chr value to: [%s]\n", client_GA_DeviceName_Str.c_str()); 
    } else {
      DEBUG_PRINTLN("Unable to Set Generic Access Device Name Chr value!");      
    } 
}

void server_setupDIS(void)
{
    server_DeviceInformation_Service = pServer->createService(UUID16_SVC_DEVICE_INFORMATION);
    server_DIS_ModelNumber_Chr = server_DeviceInformation_Service->createCharacteristic(UUID16_CHR_MODEL_NUMBER_STRING, 
                                                                            NIMBLE_PROPERTY::READ);
    server_DIS_ModelNumber_Chr->setValue(client_DIS_ModelNumber_Str);
    server_DIS_SerialNumber_Chr = server_DeviceInformation_Service->createCharacteristic(UUID16_CHR_SERIAL_NUMBER_STRING, 
                                                                            NIMBLE_PROPERTY::READ);
    server_DIS_SerialNumber_Chr->setValue(client_DIS_SerialNumber_Str);
    server_DIS_ManufacturerName_Chr = server_DeviceInformation_Service->createCharacteristic(UUID16_CHR_MANUFACTURER_NAME_STRING, 
                                                                            NIMBLE_PROPERTY::READ);                                                                                        
    server_DIS_ManufacturerName_Chr->setValue(client_DIS_Manufacturer_Str);
/*
    server_DIS_Firmware_Chr = server_DeviceInformation_Service->createCharacteristic(UUID16_CHR_FIRMWARE_REVISION_STRING, 
                                                                            NIMBLE_PROPERTY::READ);
    server_DIS_Hardware_Chr = server_DeviceInformation_Service->createCharacteristic(UUID16_CHR_HARDWARE_REVISION_STRING, 
                                                                            NIMBLE_PROPERTY::READ);
    server_DIS_Software_Chr = server_DeviceInformation_Service->createCharacteristic(UUID16_CHR_SOFTWARE_REVISION_STRING, 
                                                                            NIMBLE_PROPERTY::READ);
    server_DIS_Hardware_Chr->setValue(client_DIS_Hardware_Str);
    server_DIS_Firmware_Chr->setValue(client_DIS_Firmware_Str);
    server_DIS_Software_Chr->setValue(client_DIS_Software_Str);
*/
    server_DeviceInformation_Service->start();
}

/*
    Field #1 - Flags (byte)
        Bit 0   - Heart Rate Value Format
                    0 = uint8
                    1 = uint16
        Bit 1-2 - Sensor Contact Status
                    0 - Sensor Contact feature is not supported in the current connection
                    1 - Sensor Contact feature is not supported in the current connection
                    2 - Sensor Contact feature is supported, but contact is not detected
                    3 - Sensor Contact feature is supported and contact is detected
        Bit 3   - Energy Expended Status
                    0 = Energy Expended field is not present
                    1 = Energy Expended field is present. Units: kilo Joules
        Bit 3   - RR-Interval bit
                    0 = RR-Interval values are not present.
                    1 = One or more RR-Interval values are present.
        Bit 5-7 - Reserved
    Field #2 - Heart Rate Measurement Value (uint8)
    Field #3 - Heart Rate Measurement Value (uint16)
    Field #4 - Energy Expended (uint16)
    Field #5 - RR-Interval (uint16)
    // Flags = Format uint16 and contact supported and detected
    //byte HR_MeasurementFlags = 0b00000101;
*/

void server_setupHRM(void)
{
    server_HeartRate_Service = pServer->createService(UUID16_SVC_HEART_RATE);
    server_HR_Measurement_Chr = server_HeartRate_Service->createCharacteristic(UUID16_CHR_HEART_RATE_MEASUREMENT, 
                                                                            NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    server_HR_Measurement_Chr->setCallbacks(&server_Multi_Chr_Callbacks); //NIMBLE
    server_HR_Location_Chr = server_HeartRate_Service->createCharacteristic(UUID16_CHR_BODY_SENSOR_LOCATION, 
                                                                            NIMBLE_PROPERTY::READ);
    // Set server_HR_Location for sensor
    server_HR_Location_Chr->setValue(&client_HR_Location_Value, 1);
    server_HeartRate_Service->start();  
}

void client_Set_All_NotificationIndication(bool IsEnable)
{   
if(IsEnable) { // Enable Client Char's
  if( pRemote_FTM_ControlPoint_Chr != nullptr) { // Check: Is it exposed?
            pRemote_FTM_ControlPoint_Chr->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)indicationOn, 2, true); 
          }
  if ( pRemote_HR_Measurement_Chr != nullptr ) { // ERROR NO BOOL = TRUE !!!!!!!!!!
            pRemote_HR_Measurement_Chr->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true); 
          }
  if( pRemote_FTM_TrainingStatus_Chr != nullptr) { // Check: Is it exposed?
            pRemote_FTM_TrainingStatus_Chr->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true); 
          }
  if( pRemote_FTM_IndoorBikeData_Chr != nullptr) { // Check: Is it exposed?
            pRemote_FTM_IndoorBikeData_Chr->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true); 
          }
  if( pRemote_FTM_Status_Chr != nullptr) { // Check: Is it exposed?
            pRemote_FTM_Status_Chr->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true); 
          }
  if ( pRemote_CSC_Measurement_Chr != nullptr ) {
            pRemote_CSC_Measurement_Chr->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true); 
          }          
  if ( pRemote_CP_Measurement_Chr != nullptr ) {
            pRemote_CP_Measurement_Chr->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true); 
          }                     
  DEBUG_PRINTLN("All Client (Trainer) Characteristics are Notify/Indicate Enabled!");
  } else { // Disable Client Char's
  if( pRemote_FTM_ControlPoint_Chr != nullptr) { // Check: Is it exposed?
            pRemote_FTM_ControlPoint_Chr->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)indicationOff, 2, true); 
          }
  if ( pRemote_HR_Measurement_Chr != nullptr ) { // ERROR BOOL false insteadof TRUE !!!!!!!!!!
            pRemote_HR_Measurement_Chr->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOff, 2, true); 
          }
  if( pRemote_FTM_TrainingStatus_Chr != nullptr) { // Check: Is it exposed?
            pRemote_FTM_TrainingStatus_Chr->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOff, 2, true); 
          }
  if( pRemote_FTM_IndoorBikeData_Chr != nullptr) { // Check: Is it exposed?
            pRemote_FTM_IndoorBikeData_Chr->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOff, 2, true); 
          }  
  if( pRemote_FTM_Status_Chr != nullptr) { // Check: Is it exposed?
            pRemote_FTM_Status_Chr->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOff, 2, true); 
          }  
  if ( pRemote_CSC_Measurement_Chr != nullptr ) {
            pRemote_CSC_Measurement_Chr->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOff, 2, true); 
          }                      
  if ( pRemote_CP_Measurement_Chr != nullptr ) {
            pRemote_CP_Measurement_Chr->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOff, 2, true); 
          }
  DEBUG_PRINTLN("All Client (Trainer) Characteristics are Notify/Indicate Disabled!");
  }
} // end

void loop() { // loop() is used to start sort of Callback functions
 // If the flag "DoCallClientEnable" is true, we enable Notify and Indicate on ALL Client Char's of BLE server!
  if(Trainer.IsConnected && DoCallClientEnable) {
    DoCallClientEnable = false; 
    client_Set_All_NotificationIndication(true);
  }
 // If the flag "DoCallClientDisable" is true, we disable Notify and Indicate on ALL Client Char's of BLE server!
  if(Trainer.IsConnected && DoCallClientDisable) {
    DoCallClientDisable = false; 
    client_Set_All_NotificationIndication(false);
  }
  // If the flag "doClientConnectCall" is true, we connect to the BLE server!
  if (doClientConnectCall) {
    doClientConnectCall = false;
    bool dummy = client_Connect_Callback();
  } // doClientConnectCall
  // If the flag "RestartScanningOnDisconnect" is true, we start scanning for a (new) BLE Server!  
  if(RestartScanningOnDisconnect) {
        //DEBUG_PRINTLN("Trying to set indicate off!");
        //server_FTM_ControlPoint_Chr->getDescriptorByUUID(BLEUUID((uint16_t)0x2902))->setValue((uint8_t*)indicationOff, 2);    
        pBLEScan->clearResults();   // delete results from BLEScan buffer to release memory    
        DEBUG_PRINTLN("Client Starts Scanning again for Server Device with CPS, CSC and FTMS!");
        RestartScanningOnDisconnect = false;        
        pBLEScan->start(0, false);
  } 
 delay(200);  // DO NOT REMOVE or Task watchdog will be triggered!!!   
} // End of loop
