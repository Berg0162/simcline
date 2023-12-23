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
 *             This code should work with all indoor cycling trainers that fully support,
 *        Fitness Machine Service, Cycling Power Service and Cycling Speed & Cadence Service
 * ------------------------------------------------------------------------------------------------------
 *
 *  The code links a BLE Server (a peripheral to Zwift) and a BLE Client (a central to the Trainer) with a bridge 
 *  in between, the ESP32 being man-in-the-middle (MITM). The ESP32 is an integral part of the Simcline,
 *  that interprets the exchanged road grade and moves the front wheel up and down with the change in inclination.
 *  The ESP32-bridge can control, filter and alter the bi-directional interchanged data!
 *  The client-side (central) scans and connects with the trainer relevant services: FTMS, CPS and CSC. It collects 
 *  all cyling data of the services and passes these on to the server-side....  
 *  The client-side supplies the indoor trainer with target and resistance control data.
 *  The server-side (peripheral) advertises and enables connection with cycling apps like Zwift and collects the app's  
 *  control commands, target and resistance data. It passes these on to the client-side....  
 *  The server-side supplies the app with the generated cycling data in return. 
 *  
 *  The client plus server (MITM) are transparent to the indoor trainer as well as to the training app Zwift or alike!
 *  
 *  Requirements: Zwift app or alike, operational Simcline and a FTMS/CPS/CSC/HBM supporting indoor trainer
 *  0) Upload and Run this code on the Simcline (i.c. ESP32 board)
 *  1) Start the Serial Monitor to catch debugging info and check the Oled display
 *  2) The code will do basic testing of mechanical parts and sensors
 *  3) Start/Power On the indoor trainer  
 *  4) Simcline and trainer (with <name>) will pair as reported in the output
 *  5) Start Zwift on your computer or tablet and wait....
 *  6) Search on the Zwift pairing screens for the Simcline a.k.a. "Sim <name>"
 *  7) Pair: Power, Cadence and Controllable one after another with "Sim <name>"
 *  8) Optionally one can pair as well devices for heartrate and/or steering (Sterzo)
 *  9) Start the default Zwift ride or any ride you wish
 * 10) Make Serial Monitor output window visible on top of the Zwift window 
 * 11) Hop on the bike: do the work and feel resistance change with the road
 * 12) Inspect the info presented by Serial Monitor.....
 *  
 *  Your trainer's device <name> is modified by the Simcline code to "Sim <name>", to allow for a clear distinction 
 *  between the Simcline (simulating your trainer) and your original trainer, when advertising the trainer's services!
 *  You will notice this only when connecting to Zwift on the pairing screens! Notice: Zwift extends device names with
 *  additional numbers for extra identification!
 *  
 */
/*
Version 1.1
Changed Stack Depth values from 2048 to 4096 for Server Control Point Indicate (modified) and Write w Response
Version 1.2
Inserted check (boolean) on Control Point Write-Response out of synch...
Version 1.3 
Cycling Speed Cadence Service changed to NOT Mandatory
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
//#define DEBUG_FTM_TRAININGSTATUS// If defined allows for parsing the Training Status Data
//#define DEBUG_FTM_STATUS        // If defined allows for parsing the Machine Status Data
#define DEBUG_FTM_CONTROLPOINT_RESPONSE     // If defined allows for parsing the Data
#define DEBUG_FTM_CONTROLPOINT_OPCODE_DATA  // If defined allows for parsing and decoding Data
//#define MOVEMENTDEBUG 
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

#define 	BLE_APPEARANCE_GENERIC_CYCLING   1152

#include <NimBLEDevice.h>
// We need this for setting the Server-side Generic Access Char's --> Appearance and DeviceName
#include <nimble/nimble/host/services/gap/include/services/gap/ble_svc_gap.h>

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
#define UUID16_CHR_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS BLEUUID((uint16_t)0x2A04)
#define UUID16_CHR_CENTRAL_ADDRESS_RESOLUTION                 BLEUUID((uint16_t)0x2AA6)
BLERemoteService* pRemote_GenericAccess_Service;
BLERemoteCharacteristic* pRemote_GA_Appearance_Chr; // Read
uint16_t client_GA_Appearance_Value = BLE_APPEARANCE_GENERIC_CYCLING;  // Default decimal: 1152 -> Generic Cycling
BLERemoteCharacteristic* pRemote_GA_DeviceName_Chr;// Read, Write
std::string client_GA_DeviceName_Str = "ESP32";

// Client Service Device Information --------------------------------------------------
#define UUID16_SVC_DEVICE_INFORMATION                         BLEUUID((uint16_t)0x180A)
#define UUID16_CHR_MODEL_NUMBER_STRING                        BLEUUID((uint16_t)0x2A24)
#define UUID16_CHR_SERIAL_NUMBER_STRING                       BLEUUID((uint16_t)0x2A25)
#define UUID16_CHR_FIRMWARE_REVISION_STRING                   BLEUUID((uint16_t)0x2A26)
#define UUID16_CHR_HARDWARE_REVISION_STRING                   BLEUUID((uint16_t)0x2A27)
#define UUID16_CHR_SOFTWARE_REVISION_STRING                   BLEUUID((uint16_t)0x2A28)
#define UUID16_CHR_MANUFACTURER_NAME_STRING                   BLEUUID((uint16_t)0x2A29)
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
//std::string client_DIS_ModelNumber_Str = "ESP32 Feather V2";
BLECharacteristic *server_DIS_SerialNumber_Chr;      // Read
//std::string client_DIS_SerialNumber_Str = "12345";
BLECharacteristic *server_DIS_Firmware_Chr;          // Read
std::string client_DIS_Firmware_Str = "12345";
BLECharacteristic *server_DIS_Hardware_Chr;          // Read
std::string client_DIS_Hardware_Str = "12345";
BLECharacteristic *server_DIS_Software_Chr;          // Read
std::string client_DIS_Software_Str = "12345";
BLECharacteristic *server_DIS_ManufacturerName_Chr;  // Read
//std::string client_DIS_Manufacturer_Str = "Adafruit Industries";
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
uint8_t client_CP_Location_Value = {0x0C};                    //          --> rear wheel !
BLERemoteCharacteristic* pRemote_CP_ControlPoint_Chr;   // Indicate, Write
bool client_ControlPoint_Response = true;               // CP Write-Response synch test
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
      
const char* client_Sensor_Location_Str[] = { "Other", "Top of shoe", "In shoe", "Hip", 
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
// Libraries for use of I2C devices (Oled and VL6180X distance sensor)
#include <SPI.h>
#include <Wire.h>
// Necessary libraries for use of Oled display(s)
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// Additional splash screen bitmap and icon(s) for Oled
#include "Adafruit_SSD1306_Icons.h" // needs to be in the SAME (!) directory
// Declarations for an SSD1306 128x64 display connected to I2C (SDA, SCL pins)
#define SCREEN_WIDTH 128            // SSD1306-OLED display width, in pixels
#define SCREEN_HEIGHT 64            // SSD1306-OLED display height, in pixels
#define OLED_RESET -1               // No reset pin on this OLED display
#define OLED_I2C_ADDRESS 0x3C       // I2C Address of OLED display
// Declare the display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// Declare Global var for OLED Display selection 1 (Cycling data) or 2 (Road Grade)
uint8_t OledDisplaySelection = 2; // default Road Grade to show

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
#define RGVMIN 19500 // -5%  // Always is RGVMIN < 20000 (flat road level)
// Raw Grade Value Maximally (Mechanically: the highest position of wheel axis) 22000 is equiv. of 20% uphill road grade
#define RGVMAX 22000 // 20%  // +20% // Always is RGVMAX > 20000 (flat road level)
//------------------------------------------------- WARNING --------------------------------------------------------------

// Correction for measuring plane difference and midth wheel axis position (1 cm offset is an MEASUREOFFSET of about 40)
#define MEASUREOFFSET 50 // about 1.25 cm
// Raw Grade Value Minimally (Mechanically: the lowest position of wheel axis)  19000 is equiv. of 10% downhill road grade
// These values are derived from the above RGVMIN and RGVMAX settings
#define RGVMIN_GRADE (20000-RGVMIN)/100 // Notice: positive value of the Minimal downhill grade! 
#define RGVMAX_GRADE (RGVMAX-20000)/100 // Notice: positive value of the Maximal uphill grade!
// Besides what is mechanically possible there are also limits in what is physically pleasant
// Keep the following aRGVMin and aRGVMax values within the limits of the mechanically feasible values of above !!!
// DEFAULT Minimally Allowed Raw Grade Value that should not be exceeded: -5%! -> Descent grade Limit
int aRGVmin = 19500;
// DEFAULT Maximally Allowed Raw Grade Value that should not be exceeded: 15%! -> Ascent grade limit
int aRGVmax = 21500;
// Value for a flat road equals 0% grade or a RGV of 20000; result needs to be corrected for the measure offset
long RawgradeValue = (20000 - MEASUREOFFSET);
int GradeChangeFactor = 100; // 100% means no effect, 50% means only halved up/down steps --> Road Grade Change Factor
// The Grade Percentage of a road is defined as a measure of the road's steepness as it rises and falls along its route
float gradePercentValue = 0;
//-----------------------------------------------------------------

/* ------------------------------------------------------------------------------------------------
 * Warning I/O Pins can have identical board position but different I/O Pin declarations for 
 * connection with the pins of the Motor driver board
 * ADAFRUIT_FEATHER_ESP32_V2 is nearly pin compatible with ARDUINO_NRF52840_FEATHER
*/
#ifdef ADAFRUIT_FEATHER_ESP32_V2
#define actuatorOutPin1 A0   // --> A0/P0.02 connected to pin IN2 of the DRV8871 Motor Driver board
#define actuatorOutPin2 A1   // --> A1/P0.03 connected to pin IN1 of the DRV8871 Motor Driver board
#endif

// -------------------------- WARNING ------------------------------------------------------------
// The following VL6180X sensor values are a 100% construction specific and
// should be experimentally determined, when the Actuator AND the VL6180X sensor are mounted!
// ------>>>> Test manually and use example/test sketches that go with the VL6180X sensor! <<<<---
// Microswitches should limit physically/mechanically the upper and lower position of the Actuator!
// The microswitches are mechanically controlled, and NOT by the software --> should be fail safe!
// Notice that unrestricted movement at the boundaries can damage the Actuator and/or construction!
// The following values are respected by the software and will (in normal cases!) never be exceeded!
#define MINPOSITION 270 // 265 // VL6180X highest value top microswitch activated to mechanically stop operation
#define MAXPOSITION 470 // 535 // VL6180X lowest value bottom microswitch activated to mechanically stop operation

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
bool IsBasicMotorFunctions = false; // Mechanical motor functions
// ----------------------xControlUpDownMovement task definitions ------------------
SemaphoreHandle_t xSemaphore = NULL;
TaskHandle_t ControlTaskHandle = NULL;
// Set Arduino IDE Tools Menu --> Events Run On: "Core 1"
// Set Arduino IDE Tools Menu --> Arduino Runs On: "Core 1"
// Run xControlUpDownMovement on "Core 0"
const BaseType_t xControlCoreID = 0;
void xControlUpDownMovement(void* arg); 
// --------------------------------------------------------------------------------
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
// Handler class for Server CPS, CSC, HRM, FTMS Characteristics action(s) defined
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

bool getPRSdata(void);
void setPRSdata(void);
void ConvertMacAddress(char *fullAddress, uint8_t addr[6], bool NativeFormat);
#ifdef EMA_ALPHA
int16_t EMA_TargetPositionFilter(int16_t current_value);
#endif
void ShowIconsOnTopBar(void);
void ShowOnOledLarge(const char* Line1, const char* Line2, const char* Line3, uint16_t Pause);
void ShowSlopeTriangleOnOled(void);
void SetManualGradePercentValue(void);
void SetNewRawGradeValue(float RoadGrade);
void SetNewActuatorPosition(void);

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

  Serial.begin(115200);
  while ( !Serial ) 
      delay(10); 

/* The Feather ESP32 V2 has a NEOPIXEL_I2C_POWER pin that must be pulled HIGH
 * to enable power to the STEMMA QT port. Without it, the QT port will not work!
 */
#ifdef ADAFRUIT_FEATHER_ESP32_V2
  // Turn on the I2C power on Stemma connector by pulling pin HIGH.
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
#endif

  DEBUG_PRINTLN("ESP32 NimBLE SIMCLINE supporting: CPS, CSC, HBM and FTMS");
  DEBUG_PRINTLN("-------------------- Version 01.2 ----------------------");
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
    display.display(); // Acknowledge Adafruit rights, license and efforts
    delay(500); // show some time
  }
  // Ready to show our own SIMCLINE splash screen
  display.clearDisplay(); // clean the oled screen
  display.setTextColor(SSD1306_WHITE);
  display.drawBitmap(24, 0, Mountain_bw_79x64, 79, 64, 1);
  display.display();
  delay(2000); // Take somewhat more time.....
  //Show Name and SW version on Oled
  ShowOnOledLarge("SIMCLINE", "FTMS", "v01.0", 500);
  // Initialize Lifter Class data, variables, test and set to work !
  lift.Init(actuatorOutPin1, actuatorOutPin2, MINPOSITION, MAXPOSITION, BANDWIDTH);
  // Test Actuator and VL8106X for proper functioning
  ShowOnOledLarge("Testing", "Up & Down", "Functions", 100);
  if (!lift.TestBasicMotorFunctions()) {
    ShowOnOledLarge("Testing", "Functions", "Failed!", 500);
    IsBasicMotorFunctions = false; // Not working properly
    DEBUG_PRINTLN("Simcline >> ERROR << Basic Motor Funtions are NOT working!!");
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
  // Initialize NimBLE with maximum connections as Peripheral = 1, Central = 1
  BLEDevice::init("ESP32"); // Shortname    
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
  //BLEDevice::stopAdvertising(); 
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
  //BLEDevice::startAdvertising(); 
  //DEBUG_PRINTLN("Server is advertising: CPS, CSC and FTMS"); 
} // End of setup.

// LittleFS --------------------------------------------------
bool getPRSdata(void) { // aRGVmax, aRGVmin, GradeChangeFactor -> PRSdata
  if (LittleFS.exists(PRS_FILENAME)) {
    File file = LittleFS.open(PRS_FILENAME, FILE_READ);
    if (file) {    
      uint32_t readLen;
      uint8_t buffer[LITTLEFS_BLOCK_SIZE+1] = { 0 };
      readLen = file.read(buffer, LITTLEFS_BLOCK_SIZE);
      buffer[readLen] = 0; // set the values to the pointed by variables
      sscanf((char*)buffer, "%d %d %d %d", &aRGVmax, &aRGVmin, &GradeChangeFactor, &OledDisplaySelection);
      DEBUG_PRINT(F("ESP32 internally Got persistent storage from: /littlefs/PRSdata -> "));
      DEBUG_PRINTF("Max: %d Min: %d Perc.: %d Displ.: %d\n", aRGVmax, aRGVmin, GradeChangeFactor, OledDisplaySelection);
      file.close();
      return true;
    }    
  }
  return false;
}

void setPRSdata(void) { // aRGVmax, aRGVmin, GradeChangeFactor -> PRSdata
  uint8_t buffer[LITTLEFS_BLOCK_SIZE+1] = { 0 };
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

void ShowIconsOnTopBar(void) {
  // Show Icons on Top Bar
  if (Trainer.IsConnected) { // show icon
    display.drawBitmap(112, 0, power_icon16x16, 16, 16, 1);
  }
  if (Laptop.IsConnected) { // show icon
    display.drawBitmap(0, 0, zwift_icon16x16, 16, 16, 1);
  }
  if (Smartphone.IsConnected) { // show icon Phone
    display.drawBitmap(0, 0, mobile_icon16x16, 16, 16, 1);
  }
}

void ShowOnOledLarge(const char* Line1, const char* Line2, const char* Line3, uint16_t Pause) {
  // Clear and set Oled to display 3 line info -> centered
  int pos = 1;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  // ShowIconsOnTopBar();
  display.setTextSize(2);  // Large characters 11 pixels wide
  if (Line1) {
    pos = round( (127 - (12 * strlen(Line1))) / 2 );
    display.setCursor(pos, 2); // 16
    display.print(Line1);
  }
  if (Line2) {
    pos = round( (127 - (12 * strlen(Line2))) / 2 );
    display.setCursor(pos, 22); // 16
    display.print(Line2);
  }
  if (Line3) {
    pos = round( (127 - (12 * strlen(Line3))) / 2 );
    display.setCursor(pos, 44); // 16
    display.print(Line3);
  }
  display.display();
  delay(Pause);  // Pause indicated time in ms
}

void ShowSlopeTriangleOnOled(void) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  ShowIconsOnTopBar();
  display.setCursor(102, 10); //26
  display.setTextSize(2);
  display.print(F("%"));
  char tmp[7];
  dtostrf(gradePercentValue, 5, 1, tmp); // show sign only if negative
  display.setCursor(10, 6); // 22
  display.setTextSize(3);
  display.print(tmp);
  // The following calculations give more "weight" to lower grade values
  // (like: 1.2% or 0.5%), these will occur more often in practice and are not well
  // displayable at 128*64! --> 64 * 64 = 4096 and this value should not be
  // exceeded (4096/20) = 204.8
  int pos = 64 - int(sqrt(abs(204 * gradePercentValue))); // cast to int to get rid of decimals only now!
  if (gradePercentValue > 0) {
    display.fillTriangle( 1, 63, 127, 63, 127, pos, SSD1306_INVERSE);
  } else {
    display.fillTriangle( 127, 63, 1, 63, 0, pos, SSD1306_INVERSE);
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

  exponential_average = int16_t( (EMA_ALPHA * (uint32_t)current_value + (100 - EMA_ALPHA) * (uint32_t)exponential_average) / 100 );
  return exponential_average;
}
#endif

void SetManualGradePercentValue(void) 
{
  gradePercentValue = float( (RawgradeValue - 20000 + MEASUREOFFSET) )/100;
  SetNewActuatorPosition();
}

void SetNewRawGradeValue(float RoadGrade)
{
        // ----- Recalculate to relevant values for this project ------
        // Take into account the allowed Increase Percentage of the inclination
        // 100% has no effect, 50% means every increase or decrease is divided by 2
        // --> the increase or decrease of inclination is in 50% smaller steps...
        gradePercentValue = RoadGrade;
        RawgradeValue = (long)(RoadGrade*100) + 20000;
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
        DEBUG_PRINTF("Set Simcline to Percentage: %02.1f %% RawgradeValue: %05d \n", gradePercentValue, RawgradeValue);
}

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

// ----------------------------------------------------------------------------------

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
    // Obtain a reference to the remote CSC service.
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
        DEBUG_PRINT(" -> Client Reads CSC Location Sensor:");
        DEBUG_PRINTF(" Loc#: %d %s\n", client_CSC_Location_Value, client_Sensor_Location_Str[client_CSC_Location_Value]);
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
        // Body sensor location value is 8 bit
        const char* body_str[] = { "Other", "Chest", "Wrist", "Finger", "Hand", "Ear Lobe", "Foot" };
        DEBUG_PRINT(" -> Client Reads HR Location Sensor:");
        DEBUG_PRINTF(" Loc#: %d %s\n", client_HR_Location_Value, body_str[client_HR_Location_Value]);
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
  // ----------------------------------------------------------------------------------------------
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
  // scan to run for ## seconds.
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
          String str = "Central Updated CCCD -->";
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
    DEBUG_PRINTF("Setting Appearance in Advertised data to [%d]\n", client_GA_Appearance_Value);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinInterval(144); // 32 in 0.625ms units, 0 = use default.
    pAdvertising->setMaxInterval(244); // 244 in 0.625ms units, 0 = use default.
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
      DoCallClientEnable = true;
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
    DEBUG_PRINTF("Central (%s/Zwift) updated MTU to: [%u] for connection ID: %u\n", Laptop.PeerName.c_str(), MTU, desc->conn_handle);           
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
  // write with response !!! writeValue(string, bool response = true);
  if(Trainer.IsConnected) { 
    xTaskCreate(&TaskWriteWithResponse, "Write w Response", 4096, (void *)NULL, 1, &TaskWriteWithResponseHandle); //2048
    client_ControlPoint_Response = false;  
  } 
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
#ifdef DEBUG_FTM_CONTROLPOINT_OPCODE_DATA
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
    case ftmcpReset: {
      DEBUG_PRINTLN("Reset Machine!");
      break;
    }
#endif

    case ftmcpSetIndoorBikeSimulationParameters: {
#ifdef DEBUG_FTM_CONTROLPOINT_OPCODE_DATA
      // Short is 16 bit signed, so the windspeed is converted from two bytes to signed value. Highest bit is sign bit
      short ws = (server_FTM_Control_Point_Data.values.OCTETS[0] << 8) + server_FTM_Control_Point_Data.values.OCTETS[1]; 
      wind_speed = ws / 1000.0;
#endif
      // Short is 16 bit signed, so a negative grade is correctly converted from two bytes to signed value. Highest bit is sign bit
      short gr = (server_FTM_Control_Point_Data.values.OCTETS[3] << 8) + server_FTM_Control_Point_Data.values.OCTETS[2]; 
      grade = (float)(gr/100.0);
#ifdef DEBUG_FTM_CONTROLPOINT_OPCODE_DATA
      crr = server_FTM_Control_Point_Data.values.OCTETS[4] / 10000.0;
      cw = server_FTM_Control_Point_Data.values.OCTETS[5] / 100.0;
      // Remember, if debugging with Zwift, that these values are divided by 2 if in normal 50% settings!
      DEBUG_PRINTLN("Set Indoor Bike Simulation Parameters!");
      DEBUG_PRINT("Wind speed (1000): "); DEBUG_PRINT(wind_speed);
      DEBUG_PRINT(" | Grade (100): "); DEBUG_PRINT(grade);
      DEBUG_PRINT(" | Crr (10000): "); DEBUG_PRINT(crr);
      DEBUG_PRINT(" | Cw (100): "); DEBUG_PRINTLN(cw);
#endif
      SetNewRawGradeValue(grade);
      SetNewActuatorPosition();
      ShowSlopeTriangleOnOled();  
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
    case ftmcpSetTargetedCadence: {
#ifdef DEBUG_FTM_CONTROLPOINT_OPCODE_DATA      
      DEBUG_PRINTLN("Unresolved OpCode!");
#endif
      break;
      }
    } // switch
  } // onWrite
  
  void onSubscribe(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue) {
        String str = "Central Updated CCCD -->";
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
    server_FTM_ControlPoint_Chr->setCallbacks(new server_FTM_ControlPoint_Chr_callback());
    server_FitnessMachine_Service->start();      
}

class server_NUS_Rxd_Chr_callback: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    // Read data received over NUS Rxd from Mobile Phone
    std::string NusRxdData = server_NUS_Rxd_Chr->getValue();
    uint8_t NusRxdDataLen = NusRxdData.length();  // Get the actual length of data bytes
    // Display the raw packet data in actual length
    DEBUG_PRINTF(" -> Server Rec'd NUS Rxd Data [%d][%s]\n", NusRxdDataLen, NusRxdData.c_str());
    // The following routines parse and process the incoming commands
    // Every NusRxdData packet starts with a '!' otherwise corrupt/invalid
    if (NusRxdData[0] != '!') {
      DEBUG_PRINTLN(F("-> Error: RXD-packet does not start with a '!'"));
      return; // invalid NusRxdData packet: do not further parse and process
    }
  // RXpacket buffer has IdCode = "S"
  if (NusRxdData[1] == 'S') { // Settings packet
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
    iMin = constrain(iMin, 0, RGVMIN_GRADE); // Notice: positive value!
    aRGVmin = map(iMin, RGVMIN_GRADE, 0, RGVMIN, 20000);
    // set Road Grade Change Factor
    GradeChangeFactor = iPerc;
    // set OledDisplaySelection
    OledDisplaySelection = iDispl;
    // LittleFS for persistent storage of these values
    setPRSdata();
    // LittleFS --------------------------------------
    DEBUG_PRINT(F(" Settings: Max: ")); DEBUG_PRINT(iMax);
    DEBUG_PRINT(F(" Min: ")); DEBUG_PRINT(iMin);
    DEBUG_PRINT(F(" Perc: ")); DEBUG_PRINT(iPerc);
    DEBUG_PRINT(F(" Displ: ")); DEBUG_PRINTLN(iDispl);
    // Confirm to the PHONE: settings rcvd and set to persistent
    DEBUG_PRINTF("Server Sends NUS TXD Confirm message: Done!\n");
    server_NUS_Txd_Chr->setValue("!SDone!;");
    server_NUS_Txd_Chr->notify();
    return; // Settings rcvd and set to persistent
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
  DEBUG_PRINTF("\nServer Sends NUS TXD Error message: Out of Order!\n");
  }; // onWrite
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
    DEBUG_PRINTF("Server Sends NUS TXD Persistent settings to Phone: [%s]\n", (char*)TXpacketBuffer);
}

// Handler class for Server NUS Txd Characteristic actions limited to onSubscribe
class server_NUS_Txd_Callback: public NimBLECharacteristicCallbacks {
/*  We only define onSubscribe !!!
    void onRead(NimBLECharacteristic* pCharacteristic);
    void onWrite(NimBLECharacteristic* pCharacteristic);
    void onNotify(NimBLECharacteristic* pCharacteristic);    
    void onStatus(NimBLECharacteristic* pCharacteristic, Status status, int code);
*/  
  void onSubscribe(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue) {
          String str = "Central Updated CCCD -->";
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

        if(subValue == 1) {
          server_NUS_Txd_Persistent_Settings();
        }
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
    server_NUS_Txd_Chr->setCallbacks(new server_NUS_Txd_Callback()); //NIMBLE
    server_NordicUart_Service->start();
}

void Construct_Dev_Name(void)
{
  const char prefix[] = {'S', 'i', 'm', ' '}; // #4 Chars
  size_t len = client_GA_DeviceName_Str.length(); // Len of null terminated char array
  int MaxLen = (MAX_PAYLOAD - sizeof(prefix) - 1); // 1 less for null terminating char
  if (len > MaxLen) {
    len = MaxLen;
  }
  int pos = MaxLen;
  if (len > 0) {
    // pfound points to the first occurence of " " (blank space char)
    char *pfound = strstr((const char*)client_GA_DeviceName_Str.c_str(), " ");
    if (pfound != NULL) {
      pos = int(pfound - (char*)client_GA_DeviceName_Str.c_str());  // Convert to position in DevName
    }
  }
  if ( pos > MaxLen ) {
    pos = MaxLen;  // Stay within char array allocated memory!
  }
  memmove((void*)&client_GA_DeviceName_Str.c_str()[sizeof(prefix)], (void*)&client_GA_DeviceName_Str.c_str()[0], (size_t)pos); // Make space: shift to the right
  memcpy((void*)&client_GA_DeviceName_Str.c_str()[0], &prefix, (size_t)sizeof(prefix)); // Insert prefix at begin of DevName
  client_GA_DeviceName_Str[(pos + sizeof(prefix))] = 0; // Make null terminated char array at new position, skip rest!
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
    Construct_Dev_Name(); // Convert Device name to "Sim DevName"
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
    server_DIS_SerialNumber_Chr = server_DeviceInformation_Service->createCharacteristic(UUID16_CHR_SERIAL_NUMBER_STRING, 
                                                                            NIMBLE_PROPERTY::READ);
    server_DIS_Firmware_Chr = server_DeviceInformation_Service->createCharacteristic(UUID16_CHR_FIRMWARE_REVISION_STRING, 
                                                                            NIMBLE_PROPERTY::READ);
    server_DIS_Hardware_Chr = server_DeviceInformation_Service->createCharacteristic(UUID16_CHR_HARDWARE_REVISION_STRING, 
                                                                            NIMBLE_PROPERTY::READ);
    server_DIS_Software_Chr = server_DeviceInformation_Service->createCharacteristic(UUID16_CHR_SOFTWARE_REVISION_STRING, 
                                                                            NIMBLE_PROPERTY::READ);
    server_DIS_ManufacturerName_Chr = server_DeviceInformation_Service->createCharacteristic(UUID16_CHR_MANUFACTURER_NAME_STRING, 
                                                                            NIMBLE_PROPERTY::READ);                                                                                        
    // Set Device Information varariables   
    server_DIS_ModelNumber_Chr->setValue(client_DIS_ModelNumber_Str);
    server_DIS_SerialNumber_Chr->setValue(client_DIS_SerialNumber_Str);
    server_DIS_Hardware_Chr->setValue(client_DIS_Hardware_Str);
    server_DIS_Firmware_Chr->setValue(client_DIS_Firmware_Str);
    server_DIS_Software_Chr->setValue(client_DIS_Software_Str);
    server_DIS_ManufacturerName_Chr->setValue(client_DIS_Manufacturer_Str);
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
  if ( pRemote_HR_Measurement_Chr != nullptr ) {
            pRemote_HR_Measurement_Chr->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2); 
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
  if ( pRemote_HR_Measurement_Chr != nullptr ) { 
            pRemote_HR_Measurement_Chr->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOff, 2, false); 
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
