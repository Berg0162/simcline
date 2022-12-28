/*********************************************************************
 This is programming code for the nRF52 based Bluefruit BLE boards
 
 The code uses heavily the Adafruit supplied Bluefruit BLE libraries !!
 Adafruit invests time and resources providing open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text must be included in any redistribution
*********************************************************************/

/* 
 *  This Feather-nRF52840 tested code scans for the CPS, CSC and FTMS
 *  that the trainer is advertising, it tries to connect and then 
 *  enables .....
 *  
 *  Requirements: FTMS trainer and Feather nRF52 board
 *  1) Upload and Run this code on the Feather-nRF52
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

// -------------------------------------------------------------------------------------------
// COMPILER DIRECTIVE to allow/suppress DEBUG messages that help debugging...
// Uncomment general "#define DEBUG" to activate
#define DEBUG
// Include these debug utility macros in all cases!
#include "DebugUtils.h"
//
#ifdef DEBUG
#define DEBUG_IBD               // If defined allows for parsing the Indoor Bike Data
#define DEBUG_HBM               // If defined allows for parsing the Heart Beat Data
//#define DEBUG_CP_MEASUREMENT  // If defined allows for presentation of Cycling Power Data
//#define DEBUG_CSC_MEASUREMENT // If defined allows for presentation of Cadence & Speed Data
#endif
// --------------------------------------------------------------------------------------------

#include <bluefruit.h>

const uint8_t MAX_PAYLOAD = 20; // Max 20 byte data size for single packet BLE transfer
uint16_t client_Connection_Handle = BLE_CONN_HANDLE_INVALID;
// ----------- Some trainers need extra time to process a SVC/Char-discovery polls! -----------------
// --> delay(poll_delay) calls are inserted in the critical parts of the 'client connect' code!
// If calls to detect which SVC/Char's the trainer exposes are too quickly sequenced, time-out-induced 
// errors occur during the process! This manifests in erroneous responses about the presence of 
// exposed SVC/Char's and empty (zero) readings of Char values! Result: connection fails!
// - Zwift Hub will NOT pass 100%-error-free with a value lower than 250 ms !!
const unsigned long poll_delay = 0;  // Unit is milliseconds
//---------------------------------------------------------------------------------------------------

/* Generic Access
#define UUID16_SVC_GENERIC_ACCESS                             0x1800
#define UUID16_CHR_DEVICE_NAME                                0x2A00
#define UUID16_CHR_APPEARANCE                                 0x2A01
#define UUID16_CHR_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS 0x2A04
#define UUID16_CHR_CENTRAL_ADDRESS_RESOLUTION                 0x2AA6
*/
BLEClientService        client_GenericAccess_Service(UUID16_SVC_GENERIC_ACCESS);
BLEClientCharacteristic client_GA_Appearance_Chr(UUID16_CHR_APPEARANCE); // Read
uint16_t client_GA_Appearance_Value = 0;
BLEClientCharacteristic client_GA_DeviceName_Chr(UUID16_CHR_DEVICE_NAME);// Read, Write
unsigned char client_GA_DeviceName_Data[MAX_PAYLOAD] = {};

/* Cycling Power Service
 * CP Service: 0x1818  
 * CP Characteristic: 0x2A63 (Measurement)
 * CP Characteristic: 0x2A65 (Feature)
 * CP Characteristic: 0x2A5D (Location)
 * CP Characteristic: 0x2A66 (Control Point)
 */
BLEClientService        client_CyclingPower_Service(UUID16_SVC_CYCLING_POWER);
BLEClientCharacteristic client_CP_Measurement_Chr(UUID16_CHR_CYCLING_POWER_MEASUREMENT);    // Notify, Read
BLEClientCharacteristic client_CP_Feature_Chr(UUID16_CHR_CYCLING_POWER_FEATURE);            // Read
uint32_t client_CP_Feature_Flags = 0;
BLEClientCharacteristic client_CP_Location_Chr(UUID16_CHR_SENSOR_LOCATION);                 // Read
uint8_t client_CP_Location_Value = 0; // UINT8
BLEClientCharacteristic client_CP_ControlPoint_Chr(UUID16_CHR_CYCLING_POWER_CONTROL_POINT); // Indicate, Write

const char* client_CP_Feature_Str[] = { 
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
      
const char* client_Sensor_Location_Str[] = { "Other", "Top of shoe", "In shoe", "Hip", "Front wheel", "Left crank", "Right crank", "Left pedal",
    "Right pedal", "Front hub", "Rear dropout", "Chainstay", "Rear wheel", "Rear hub", "Chest", "Spider", "Chain ring"};

/*
 * Cycling Speed and Cadence Service
 * CSC Service:   0x1816 
 * CSC Measurement Characteristic:  0x2A5B
 * CSC Feature Characteristic:      0x2A5C  
 */
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

const char* client_CSC_Feature_Str[] = {"Wheel rev supported", "Crank rev supported", "Multiple locations supported"};

BLEClientService        client_CyclingSpeedCadence_Service(UUID16_SVC_CYCLING_SPEED_AND_CADENCE);
BLEClientCharacteristic client_CSC_Measurement_Chr(UUID16_CHR_CSC_MEASUREMENT);                   // Notify, Read
BLEClientCharacteristic client_CSC_Feature_Chr(UUID16_CHR_CSC_FEATURE);                           // Read
const uint8_t CSC_FEATURE_FIXED_DATALEN = 2; // UINT16
uint16_t client_CSC_Feature_Flags = 0;
BLEClientCharacteristic client_CSC_Location_Chr(UUID16_CHR_SENSOR_LOCATION);                      // Read
uint8_t client_CSC_Location_Value = 0; 

/* Client Service Device Information
#define UUID16_SVC_DEVICE_INFORMATION                         0x180A
#define UUID16_CHR_MODEL_NUMBER_STRING                        0x2A24
#define UUID16_CHR_SERIAL_NUMBER_STRING                       0x2A25
#define UUID16_CHR_FIRMWARE_REVISION_STRING                   0x2A26
#define UUID16_CHR_HARDWARE_REVISION_STRING                   0x2A27
#define UUID16_CHR_SOFTWARE_REVISION_STRING                   0x2A28
#define UUID16_CHR_MANUFACTURER_NAME_STRING                   0x2A29
*/
BLEClientService        client_DIS_Service(UUID16_SVC_DEVICE_INFORMATION);
BLEClientCharacteristic client_DIS_ManufacturerName_Chr(UUID16_CHR_MANUFACTURER_NAME_STRING);  // Read
char client_DIS_Manufacturer_Str[MAX_PAYLOAD] = {};
BLEClientCharacteristic client_DIS_ModelNumber_Chr(UUID16_CHR_MODEL_NUMBER_STRING);       // Read
char client_DIS_ModelNumber_Str[MAX_PAYLOAD] = {};
BLEClientCharacteristic client_DIS_SerialNumber_Chr(UUID16_CHR_SERIAL_NUMBER_STRING);      // Read
char client_DIS_SerialNumber_Str[MAX_PAYLOAD] = {};

/*
 * Fitness Machine Service, uuid 0x1826 or 00001826-0000-1000-8000-00805F9B34FB
#define UUID16_SVC_FITNESS_MACHINE                            0x1826
#define UUID16_CHR_FITNESS_MACHINE_FEATURE                    0x2ACC
#define UUID16_CHR_INDOOR_BIKE_DATA                           0x2AD2
#define UUID16_CHR_TRAINING_STATUS                            0x2AD3
#define UUID16_CHR_SUPPORTED_SPEED_RANGE                      0x2AD4
#define UUID16_CHR_SUPPORTED_INCLINATION_RANGE                0x2AD5
#define UUID16_CHR_SUPPORTED_RESISTANCE_LEVEL_RANGE           0x2AD6
#define UUID16_CHR_SUPPORTED_HEART_RATE_RANGE                 0x2AD7
#define UUID16_CHR_SUPPORTED_POWER_RANGE                      0x2AD8
#define UUID16_CHR_FITNESS_MACHINE_CONTROL_POINT              0x2AD9
#define UUID16_CHR_FITNESS_MACHINE_STATUS                     0x2ADA
 */
BLEClientService client_FitnessMachine_Service(UUID16_SVC_FITNESS_MACHINE); // FTM Service
// Service characteristics exposed by FTM Service
BLEClientCharacteristic client_FTM_Feature_Chr(UUID16_CHR_FITNESS_MACHINE_FEATURE); //  Fitness Machine Feature, mandatory, read
const uint8_t FTM_FEATURE_FIXED_DATALEN = 8;
uint8_t client_FTM_Feature_Data[FTM_FEATURE_FIXED_DATALEN];
#ifdef DEBUG
// ---------------------Fitness Machine Features (bytes 1-4):
const char* client_FTM_Feature_Str_One[32] = {
"Average Speed Supported", "Cadence Supported", "Total Distance Supported", "Inclination Supported",
"Elevation Gain Supported", "Pace Supported", "Step Count Supported", "Resistance Level Supported",
"Stride Count Supported", "Expended Energy Supported", "Heart Rate Measurement Supported", "Metabolic Equivalent Supported",
"Elapsed Time Supported", "Remaining Time Supported", "Power Measurement Supported", "Force on Belt and Power Output Supported",
"User Data Retention Supported", "Reserved for Future Use","","","","","","","","","","","","","",""};
// ---------------------Fitness Machine Target Setting Features (bytes 5-8) :
const char* client_FTM_Feature_Str_Two[32] = {
"Speed Target Setting Supported", "Inclination Target Setting Supported", "Resistance Target Setting Supported", "Power Target Setting Supported",
"Heart Rate Target Setting Supported", "Targeted Expended Energy Configuration Supported", "Targeted Step Number Configuration Supported",
"Targeted Stride Number Configuration Supported", "Targeted Distance Configuration Supported", "Targeted Training Time Configuration Supported",
"Targeted Time in Two Heart Rate Zones Configuration Supported", "Targeted Time in Three Heart Rate Zones Configuration Supported",
"Targeted Time in Five Heart Rate Zones Configuration Supported", "Indoor Bike Simulation Parameters Supported", "Wheel Circumference Configuration Supported",
"Spin Down Control Supported", "Targeted Cadence Configuration Supported", "Reserved for Future Use","","","","","","","","","","","","","",""};
#endif
BLEClientCharacteristic client_FTM_IndoorBikeData_Chr(UUID16_CHR_INDOOR_BIKE_DATA); //  Indoor Bike Data, optional, notify
BLEClientCharacteristic client_FTM_TrainingStatus_Chr(UUID16_CHR_TRAINING_STATUS); //  Training Status, optional, read & notify
BLEClientCharacteristic client_FTM_SupportedResistanceLevelRange_Chr(UUID16_CHR_SUPPORTED_RESISTANCE_LEVEL_RANGE); // Supported Resistance Level, read, optional
const uint8_t FTM_SRLR_FIXED_DATALEN = 6;
uint8_t client_FTM_SupportedResistanceLevelRange_Data[FTM_SRLR_FIXED_DATALEN];
BLEClientCharacteristic client_FTM_SupportedPowerRange_Chr(UUID16_CHR_SUPPORTED_POWER_RANGE); // Supported Power Levels, read, optional
const uint8_t FTM_SPR_FIXED_DATALEN = 6;
uint8_t client_FTM_SupportedPowerRange_Data[FTM_SPR_FIXED_DATALEN];
BLEClientCharacteristic client_FTM_ControlPoint_Chr(UUID16_CHR_FITNESS_MACHINE_CONTROL_POINT); //  Fitness Machine Control Point, optional, write & indicate
BLEClientCharacteristic client_FTM_Status_Chr(UUID16_CHR_FITNESS_MACHINE_STATUS); //  Fitness Machine Status, mandatory, notify

/* HRM Service Definitions
 * Heart Rate Monitor Service:  0x180D
 * Heart Rate Measurement Char: 0x2A37 (Mandatory)
 * Body Sensor Location Char:   0x2A38 (Optional)
 */
BLEClientService        client_HeartRate_Service(UUID16_SVC_HEART_RATE);
BLEClientCharacteristic client_HR_Measurement_Chr(UUID16_CHR_HEART_RATE_MEASUREMENT);
BLEClientCharacteristic client_HR_Location_Chr(UUID16_CHR_BODY_SENSOR_LOCATION);
uint8_t client_HR_Location_Value = 1; //Chest
#define HRM_MEASUREMENT_DATALEN 4

const unsigned long TimeSpan = (poll_delay*30 + 3000);  // Unit in millis, Time to wait for client_connect_callback to finish properly
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

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb
  DEBUG_PRINTLN(" Feather nRF52 Client/Central: CPS, CSC, HBM and FTMS");
  DEBUG_PRINTLN("------------------- Version 03.1 --------------------");
  DEBUG_PRINTF("Discover polling delay setting: [%d]\n", poll_delay);
#endif
// ------------------------------------------------------
#if defined(ARDUINO_NRF52840_FEATHER)
// Allow for extra memory usage the nRF52840 has plenty!
    DEBUG_PRINTLN("Setting NRF52840 BLE configuration parameters!");
//    Bluefruit.configUuid128Count(2); // 1 Service and 2 Char
//    Bluefruit.configCentralBandwidth(BANDWIDTH_HIGH);
  Bluefruit.configCentralConn(BLE_GATT_ATT_MTU_DEFAULT, BLE_GAP_EVENT_LENGTH_DEFAULT * 2, BLE_GATTS_HVN_TX_QUEUE_SIZE_DEFAULT * 2, BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT);
//    Bluefruit.configPrphBandwidth(BANDWIDTH_HIGH);
//    Bluefruit.configPrphConn(BLE_GATT_ATT_MTU_DEFAULT, BLE_GAP_EVENT_LENGTH_DEFAULT * 2, BLE_GATTS_HVN_TX_QUEUE_SIZE_DEFAULT * 2, BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT2);
#endif
// -----------------------------------------------------------
#if defined(ARDUINO_NRF52832_FEATHER)
// Squeeze the memory to a minimum... to avoid nRF52832 out off memory errors
//    Bluefruit.configUuid128Count(2); // 1 Service and 1 Char CPWT
//    Bluefruit.configPrphBandwidth(BANDWIDTH_LOW); 
//    Bluefruit.configCentralBandwidth(BANDWIDTH_LOW);
//    Bluefruit.configAttrTableSize(1024); // 1024
#endif
// -----------------------------------------------------------

  // Initialize Bluefruit module
  DEBUG_PRINTLN("Initialise the Bluefruit nRF52 module: Client (Central)");
  // begin (Peripheral = 0, Central = 1)
  Bluefruit.begin(0, 1);
  char name[32];
  memclr(name, sizeof(name));
  Bluefruit.getName(name, sizeof(name));
  DEBUG_PRINTF("Board name: [%s]\n", name);
  // Supported tx_power values depending on mcu:
  // - nRF52832: -40dBm, -20dBm, -16dBm, -12dBm, -8dBm, -4dBm, 0dBm, +3dBm and +4dBm.
  // - nRF52840: -40dBm, -20dBm, -16dBm, -12dBm, -8dBm, -4dBm, 0dBm, +2dBm, +3dBm, +4dBm, +5dBm, +6dBm, +7dBm and +8dBm.
  Bluefruit.setTxPower(4); // See above for supported values: +4dBm
  // --------------------------------------------------------------
  Setup_Client_FTMS();
  Setup_Client_CPS();
  Setup_Client_CSC();
  Setup_Client_DIS();
  Setup_Client_HRM();
  Client_Start_Scanning();
  while (Bluefruit.Scanner.isRunning()) { // do nothing else but scanning....
  }
  TimeInterval = millis() + TimeSpan; // ADD just enough time delay
  // Wait TimeSpan for client_connect_callback to finish
  while( (millis() < TimeInterval) ) { 
    yield();
  }
  // Scanning is finished! Are we connected?
  // If NO connection is established -> Stop! 
  if( !(Bluefruit.connected()) ) { 
    DEBUG_PRINTLN("Stopped!");
    return; 
  } 
  // Try to enable Client (Trainer) CPS, CSC and FTMS data streams...
  Client_Enable_Notify_Indicate(); 
  if( !(Bluefruit.connected()) ) { // Are we meanwhile disconnected?
    DEBUG_PRINTLN("Stopped!");
    return;
  }
  // After successful enable notify/indicate of all mandatory Char's continue!  
//  DEBUG_PRINTLN("Client (Central) is Up and Running!");
  // Set first timing for sending first set of Control Point Data
  TimeInterval = millis() + CONTROL_POINT_TIME_SPAN;
}

void Setup_Client_HRM(void)
{
  // Initialize HRM client
  client_HeartRate_Service.begin();
  // Initialize client characteristics of HRM.
  // Note: Client Char will be added to the last service that is begin()ed.
  client_HR_Location_Chr.begin();
  // set up callback for receiving measurement
  client_HR_Measurement_Chr.setNotifyCallback(client_HR_Measurement_Notify_callback);
  client_HR_Measurement_Chr.begin(); 
  DEBUG_PRINTLN("HRMS and Chars 'initialized'");
}

/**
 * Hooked callback that triggered when a measurement value is sent from peripheral
 * @param chr   Pointer client characteristic that even occurred,
 *              in this example it should be client_HR_Measurement_Chr
 * @param data  Pointer to received data
 * @param len   Length of received data
 */
void client_HR_Measurement_Notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
  // Measurement contains of Flags byte, measurement (8 or 16 bit) and optional fields
#ifdef DEBUG_HBM
  uint8_t HRDataLen = (uint8_t)len;
  uint8_t HRDataBuf[HRDataLen] = {};
  // Transfer first the contents of data to buffer (array of chars)
  DEBUG_PRINTF(" -> Client Rec'd Raw Heart Rate Measurement Data: [%d] [ ", len);
  for (int i = 0; i <  sizeof(HRDataBuf); i++) {
      HRDataBuf[i] = *data++;
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

void Setup_Client_FTMS(void) 
{  
  // Initialize client FTM Service
  client_FitnessMachine_Service.begin();
  // Initialize client FTM Feature characteristic
  client_FTM_Feature_Chr.begin();
  // Initialize client FTM Training Status characteristic
  client_FTM_TrainingStatus_Chr.setNotifyCallback(client_FTM_TrainingStatus_Notify_callback);
  client_FTM_TrainingStatus_Chr.begin(); 
  // Initialize client FTM Supported Power Range characteristic
  client_FTM_SupportedPowerRange_Chr.begin();
  // Initialize client FTM Supported Resistance Level Range characteristic
  client_FTM_SupportedResistanceLevelRange_Chr.begin();
  // Initialize client FTM Indoor Bike Data characteristic
  client_FTM_IndoorBikeData_Chr.setNotifyCallback(client_FTM_IndoorBikeData_Notify_callback);
  client_FTM_IndoorBikeData_Chr.begin();
  // Initialize client FTM Control Point characteristic
  // For receiving Control Point Responses
  client_FTM_ControlPoint_Chr.setIndicateCallback(client_FTM_ControlPoint_Indicate_callback);
  client_FTM_ControlPoint_Chr.begin();
  // Initialize client FTM Status characteristic
  client_FTM_Status_Chr.setNotifyCallback(client_FTM_Status_Notify_callback);
  client_FTM_Status_Chr.begin();
  DEBUG_PRINTLN("FTMS and Chars 'initialized'");
}

void Setup_Client_CPS(void)
{  
  // Initialize CPS client
  client_CyclingPower_Service.begin();
  // Initialize CP Feature characteristics of client_CyclingPower_Service.
  client_CP_Feature_Chr.begin();
  // Initialize CP sensor location characteristics of client_CyclingPower_Service.
  client_CP_Location_Chr.begin();
  // set up callback for receiving measurement
  client_CP_Measurement_Chr.setNotifyCallback(client_CP_Measurement_Chr_notify_callback);
  client_CP_Measurement_Chr.begin();
   // Initialize Control Point and set up Indicate callback for receiving responses (indicate!)
  client_CP_ControlPoint_Chr.setIndicateCallback(client_CP_ControlPoint_Chr_indicate_callback);
  client_CP_ControlPoint_Chr.begin();
  DEBUG_PRINTLN("CPS and Chars 'initialized'");
}

void Setup_Client_CSC(void)
{
  // Initialize CSC client
  client_CyclingSpeedCadence_Service.begin();
  // Initialize client characteristics of CSC.
  client_CSC_Location_Chr.begin();
  // Initialize CSC Feature characteristics of client_CSC.
  client_CSC_Feature_Chr.begin();
  // set up callback for receiving measurement
  client_CSC_Measurement_Chr.setNotifyCallback(client_CSC_Measurement_Chr_notify_callback);
  client_CSC_Measurement_Chr.begin();
  DEBUG_PRINTLN("CSCS and Chars 'initialized'");
}  

void Setup_Client_DIS(void)
{
  // Initialize client Generic Access Service
  client_GenericAccess_Service.begin();
  // Initialize some characteristics of the Generic Access Service.
  client_GA_DeviceName_Chr.begin();
  client_GA_Appearance_Chr.begin();  
  DEBUG_PRINTLN("GA and Chars 'initialized'");
  // Initialize client Device Information Service
  client_DIS_Service.begin();
  // Initialize some characteristics of the Device Information Service.
  client_DIS_ManufacturerName_Chr.begin();
  client_DIS_ModelNumber_Chr.begin();
  client_DIS_SerialNumber_Chr.begin();
  DEBUG_PRINTLN("DIS and Chars 'initialized'");
}  

void loop()
{
 if ( Bluefruit.connected() ) 
 {
  // If time is there, send test values of Indoor Bike Simulation Parameters to 
  // the Trainer's FTM Control Point to drive the FTM...
  if(millis() > TimeInterval) 
  {
    if(ControlPointMessageCount > 31) { ControlPointMessageCount = 0; } // start all over again!
    uint8_t CPData[8] = {};
    DEBUG_PRINT("Client sends Indoor Bike Simulation Parameters to Trainer's FTM Control Point: [ ");
    // Transfer multidimensional test data to CPData buffer
    for (int i = 0; i < sizeof(CPData); i++) {
      CPData[i] = ControlPointData[ControlPointMessageCount][i];
      DEBUG_PRINTF("%02X ", CPData[i], HEX);
    }
    DEBUG_PRINTLN("] ");
    client_FTM_ControlPoint_Chr.write_resp(CPData, 8);
    TimeInterval = millis() + CONTROL_POINT_TIME_SPAN;
    ControlPointMessageCount++;
  } // TimeInterval
 } // Bluefruit connected
} // end loop

void Client_Start_Scanning(void)
{
  /* Start Central Scanning
   * - Disable/Enable auto scan if disconnected
   * - Interval = 100 ms, window = 50 ms
   * - FilterUuid only accept ...
   * - Start(timeout) with timeout = 0 will scan forever (until connected)
   */

  // Client/Central Callbacks defined
  Bluefruit.Central.setDisconnectCallback(client_disconnect_callback);
  Bluefruit.Central.setConnectCallback(client_connect_callback);
  Bluefruit.Scanner.setRxCallback(client_scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(false); // default -> true !!! in test stage we do not want to RESTART -> false
  Bluefruit.Scanner.filterRssi(-80);   // Only invoke callback when RSSI >= -80 dBm so get close to your device !!
  Bluefruit.Scanner.setInterval(160, 80); // in units of 0.625 ms
  // only invoke callback if one of these services are advertised --> check for FTMS only later
  Bluefruit.Scanner.filterUuid(UUID16_SVC_CYCLING_POWER, UUID16_SVC_CYCLING_SPEED_AND_CADENCE, UUID16_SVC_FITNESS_MACHINE); 
  Bluefruit.Scanner.useActiveScan(true);
  Bluefruit.Scanner.start(0);  // 0 = Don't stop scanning or n = after n/100 seconds
  DEBUG_PRINTLN("Start Scanning for CPS, CSC and FTMS!");
  delay(100); // To show print message !

}

void Client_Enable_Notify_Indicate(void)
{
// Reaching here means we are ready to go, let's enable Chars with Indicate/Notify
// -------------------------  Enable FTMS Notify and Indicate --------------------------------------- 
    DEBUG_PRINTLN("Enable Notify/Indicate of relevant Peripheral (Trainer) Characteristics");
if(client_FTM_ControlPoint_Chr.discovered()) {
    if ( client_FTM_ControlPoint_Chr.enableIndicate() ) { // MANDATORY
      DEBUG_PRINTLN("Ready to receive Client FTM Control Point Response Messages");
    } else {
      DEBUG_PRINTLN(">>> Couldn't enable indicate for Client FTM Control Point Characteristic.");
      DEBUG_PRINTLN("FTMS (trainer) is controlled by another Client (Training App)!");
      Bluefruit.disconnect(client_Connection_Handle);
      return;
    }
}
if(client_FTM_Status_Chr.discovered()) {
    if ( client_FTM_Status_Chr.enableNotify() ) { // MANDATORY
      DEBUG_PRINTLN("Ready to receive Client FTM Status values");
    } else {
      DEBUG_PRINTLN(">>> Couldn't enable notify for Client FTM Status Characteristic.");
      DEBUG_PRINTLN("FTMS (trainer) is controlled by another Client (Training App)!");
      Bluefruit.disconnect(client_Connection_Handle);
      return;
    }
}
if(client_FTM_TrainingStatus_Chr.discovered()) {
    if ( client_FTM_TrainingStatus_Chr.enableNotify() ) { // NOT MANDATORY
      DEBUG_PRINTLN("Ready to receive Client FTM Training Status values");
    } else {
      DEBUG_PRINTLN(">>> Couldn't enable notify for Client FTM Training Status Characteristic.");
    }
}
if(client_FTM_IndoorBikeData_Chr.discovered()) {
    if ( client_FTM_IndoorBikeData_Chr.enableNotify() ) { // MANDATORY
      DEBUG_PRINTLN("Ready to receive Client FTM Indoor Bike Data values");
    } else {
      DEBUG_PRINTLN(">>> Couldn't enable notify for Client FTM Indoor Bike Data Characteristic.");
      DEBUG_PRINTLN("FTMS (trainer) is controlled by another Client (Training App)!");
      Bluefruit.disconnect(client_Connection_Handle);
      return;
    }
}
// -------------------------  Enable FTMS Notify and Indicate ---------------------------------------  
// ---------------------  Enable CP and CSC Notify and Indicate ------------------------------------
if(client_CP_ControlPoint_Chr.discovered()) {
    if ( client_CP_ControlPoint_Chr.enableIndicate() ) { // NOT MANDATORY
      DEBUG_PRINTLN("Ready to receive Client CP Control Point Responses");
    } else {
      DEBUG_PRINTLN(">>> Couldn't enable indicate for Client CP Control Point Characteristic.");
    }
}
if(client_CP_Measurement_Chr.discovered()) {
    if ( client_CP_Measurement_Chr.enableNotify() ) { // MANDATORY
      DEBUG_PRINTLN("Ready to receive Client CP Measurement values");
    } else {
      DEBUG_PRINTLN(">>> Couldn't enable notify for Client CP Measurement Characteristic.");
      DEBUG_PRINTLN("FTMS (trainer) is controlled by another Client (Training App)!");
      Bluefruit.disconnect(client_Connection_Handle);
      return;
    }
}
if(client_CSC_Measurement_Chr.discovered()) {
    if ( client_CSC_Measurement_Chr.enableNotify() ) { // MANDATORY
      DEBUG_PRINTLN("Ready to receive Client CSC Measurement values");
    } else  {
      DEBUG_PRINTLN(">>> Couldn't enable notify for Client CSC Measurement Characteristic.");
      DEBUG_PRINTLN("FTMS (trainer) is controlled by another Client (Training App)!");
      Bluefruit.disconnect(client_Connection_Handle);
      return;
    }
}
if(client_HR_Measurement_Chr.discovered()) {
    if ( client_HR_Measurement_Chr.enableNotify() ) { // MANDATORY
      DEBUG_PRINTLN("Ready to receive Client HR Measurement values");
    } else {
      DEBUG_PRINTLN(">>> Couldn't enable notify for HRM Measurement Characteristic");
      DEBUG_PRINTLN("FTMS (trainer) is controlled by another Client (Training App)!");
      Bluefruit.disconnect(client_Connection_Handle);
      return;
    }
}
// -------------------------  Enable CP, CSC, HR Notify and Indicate --------------------------------
}

/**
 * Hooked callback that triggered when a value is sent
 * @param chr   Pointer client characteristic that event occurred,
 * @param data  Pointer to received data
 * @param len   Length of received data
 */
void client_FTM_TrainingStatus_Notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
#ifdef DEBUG
  uint8_t SDataLen = (uint8_t)len;
  uint8_t SDataBuf[SDataLen] = {};
  // Transfer first the contents of data to buffer (array of chars)
  DEBUG_PRINTF(" -> Client Rec'd Raw FTM Training Status Data: [%d] [ ", len);
  for (int i = 0; i <  sizeof(SDataBuf); i++) {
      SDataBuf[i] = *data++;
      DEBUG_PRINTF("%02X ", SDataBuf[i], HEX);
  }
  DEBUG_PRINTLN("] ");
#endif
}

/**
 * Hooked callback that triggered when a value is sent
 * @param chr   Pointer client characteristic that event occurred,
 * @param data  Pointer to received data
 * @param len   Length of received data
 */
void client_FTM_IndoorBikeData_Notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
#ifdef DEBUG
  uint8_t IBDDataLen = (uint8_t)len;
  uint8_t IBDDataBuf[IBDDataLen] = {};
  // Transfer first the contents of data to buffer (array of chars)
  DEBUG_PRINTF(" -> Client Rec'd Raw FTM Indoor Bike Data: [%d] [ ", len);
  for (int i = 0; i < sizeof(IBDDataBuf); i++) {
      IBDDataBuf[i] = *data++;
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


/**
 * Hooked callback that triggered when a value is sent
 * @param chr   Pointer client characteristic that event occurred!
 * @param data  Pointer to received data
 * @param len   Length of received data
 */
void client_FTM_ControlPoint_Indicate_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{   
#ifdef DEBUG
  uint8_t RespBufferLen = (uint8_t)len;
  uint8_t RespBuffer[RespBufferLen] = {}; // It is max 6 bytes long
  // Transfer first the contents of data to buffer (array of chars)
  DEBUG_PRINT(" -> Client Rec'd Raw FTM Control Point Response Data: [ "); 
  for (int i = 0; i < sizeof(RespBuffer); i++) {
      RespBuffer[i] = *data++;
      DEBUG_PRINTF("%02X ", RespBuffer[i], HEX);
  }
  DEBUG_PRINTLN("]");
#endif
}

/**
 * Hooked callback that triggered when a value is sent
 * @param chr   Pointer client characteristic that event occurred,
 * @param data  Pointer to received data
 * @param len   Length of received data
 */
void client_FTM_Status_Notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
#ifdef DEBUG
  uint8_t SDataLen = (uint8_t)len;
  uint8_t SDataBuf[SDataLen] = {};
  // Transfer first the contents of data to buffer (array of chars)
  DEBUG_PRINTF(" -> Client Rec'd Raw FTM Machine Status Data: [%d] [ ", len);
  for (int i = 0; i < sizeof(SDataBuf); i++) {
      SDataBuf[i] = *data++;
      DEBUG_PRINTF("%02X ", SDataBuf[i], HEX);
  }
  DEBUG_PRINTLN("] ");
#endif
}

 // Find certain uuid in the data of the received advertising packet
bool checkForUuidPresent(const uint16_t uuid, const uint8_t* reportData, uint8_t reportDataLen)
{
  // Enter uuid in printed format like 0x1826 for UUID16_SVC_FITNESS_MACHINE
  // uuid is internally stored in Little Endian
  for (int i = 0; i < (reportDataLen); i++) { // step 1: never miss out a position!
    if( memcmp(&uuid, (reportData+i), 2) == 0) {
      return true;
    }
  }
  return false; 
}

/**
 * Callback invoked when scanner pick up an advertising data
 * @param report Structural advertising data
 */
void client_scan_callback(ble_gap_evt_adv_report_t* report)
{
  // Since we configure the scanner with filterUuid(CPS, CSC, FTMS)
  // scan_callback is invoked for devices with usually CPS service advertised.
  // However, we only do business with FTMS enabled Trainer types so check 
  // for UUID16_SVC_FITNESS_MACHINE to be present, if not --> keep scanning!
  if (!checkForUuidPresent(UUID16_SVC_FITNESS_MACHINE, report->data.p_data, report->data.len)) {
    return; // Keep scanning for FTMS trainer !!
    }
#ifdef DEBUG  
  DEBUG_PRINTLN("Found advertising Peripheral with FTMS service!, see the Raw Data packet:");
  DEBUG_PRINTLN(F("Timestamp MAC Address       Rssi Data"));
  DEBUG_PRINTF("%09d ", millis());
  // MAC is in little endian --> print reverse
  Serial.printBufferReverse(report->peer_addr.addr, 6, ':');
  DEBUG_PRINT(F(" "));
  DEBUG_PRINT(report->rssi);
  DEBUG_PRINT(F("  "));
  Serial.printBuffer(report->data.p_data, report->data.len, '-');
  DEBUG_PRINTLN();
#endif
  Bluefruit.Central.connect(report);
}

#ifdef DEBUG
void PrintPeerAddress(uint8_t addr[6])
{
  for (int i = 1; i < 6; i++) {
      // Display byte by byte in HEX reverse: little Endian
      DEBUG_PRINTF("%02X:",addr[(6-i)], HEX);
  }
   DEBUG_PRINTF("%02X ",addr[0], HEX);
}
#endif

/**
 * Callback invoked when an connection is established
 * @param conn_handle
 */
void client_connect_callback(uint16_t conn_handle)
{
  char peripheral_name[MAX_PAYLOAD] = { 0 };
  uint8_t peripheral_addr[6] = {0};
  
  client_Connection_Handle = conn_handle;
  
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);
  connection->getPeerName(peripheral_name, sizeof(peripheral_name));
  ble_gap_addr_t peer_address = connection->getPeerAddr();
  memcpy(peripheral_addr, peer_address.addr, 6);
#ifdef DEBUG
  Bluefruit.printInfo();
/*
  uint16_t ConnectionInterval = 0;
  uint16_t SlaveLatency = 0;
  uint16_t SupervisionTimeout = 0;
  ConnectionInterval = connection->getConnectionInterval();
  DEBUG_PRINTF("ConnectionInterval: [%d]\n", ConnectionInterval);
  SlaveLatency = connection->getSlaveLatency();
  DEBUG_PRINTF("SlaveLatency:       [%d]\n", SlaveLatency);
  SupervisionTimeout = connection->getSupervisionTimeout();
  DEBUG_PRINTF("SupervisionTimeout: [%d]\n", SupervisionTimeout);
*/
  DEBUG_PRINTF("Feather nRF52 (Central) connected to Trainer (Peripheral) device: [%s] MAC Address: ", peripheral_name);
  PrintPeerAddress(peripheral_addr);
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("Now checking all Client Services and Characteristics!");
  DEBUG_PRINTLN("If Mandatory Services Fail --> the Client will disconnect!");
  DEBUG_PRINTLN("Checking Generic Access and Device Information!");
#endif 
  // test--------------
  // Wait some extra time for client to fully settle after connection!
  delay(poll_delay);
  // test--------------

  if( !client_GA_connect_callback(conn_handle) ) {
    Bluefruit.disconnect(conn_handle);
    return;    
  }
  if( !client_DIS_connect_callback(conn_handle) ) {
    Bluefruit.disconnect(conn_handle);
    return;    
  }
  if( !client_FTM_connect_callback(conn_handle) ) {
    Bluefruit.disconnect(conn_handle);
    return;    
  }
  if( !client_CP_connect_callback(conn_handle) ){
    Bluefruit.disconnect(conn_handle);
    return;
  }
  if( !client_CSC_connect_callback(conn_handle) ) {
    Bluefruit.disconnect(conn_handle);
    return;
  }
  if( !client_HR_connect_callback(conn_handle) ) {
    Bluefruit.disconnect(conn_handle);
    return;
  }
} // End client_connect_callback

// ---------------------------- GA and DIS SERVICE ------------------------------------------
bool client_GA_connect_callback(uint16_t conn_handle)
{
  delay(poll_delay);
  // If Generic Access is not found then go on.... NOT FATAL !
  if ( client_GenericAccess_Service.discover(conn_handle) ) {
     DEBUG_PRINT(F("Found Client Generic Access\n"));
  } else {
     DEBUG_PRINT(F("Not Found Client Generic Access\n"));
     return true;
  }
  delay(poll_delay);
  if ( client_GA_DeviceName_Chr.discover() ) {
     client_GA_DeviceName_Chr.read(client_GA_DeviceName_Data, sizeof(client_GA_DeviceName_Data));
     DEBUG_PRINTF(" -> Client Reads Device Name:   [%s]\n", client_GA_DeviceName_Data);
  } 
  delay(poll_delay);   
  if ( client_GA_Appearance_Chr.discover() ) {
      client_GA_Appearance_Value = client_GA_Appearance_Chr.read16();
      DEBUG_PRINTF(" -> Client Reads Appearance:    [%d]\n", client_GA_Appearance_Value);
  }     
  return true;
}

bool client_DIS_connect_callback(uint16_t conn_handle)
{
   delay(poll_delay);    
  // If DIS is not found then go on.... NOT FATAL !
  if ( client_DIS_Service.discover(conn_handle) ) {
    DEBUG_PRINTLN(F("Found Client Device Information"));
  } else {
    DEBUG_PRINTLN(F("Not Found Client Device Information!"));
    return true;
  }
  //  1
  delay(poll_delay);
  if ( client_DIS_ManufacturerName_Chr.discover() ) {
    // read and print out Manufacturer
    if ( client_DIS_ManufacturerName_Chr.read(client_DIS_Manufacturer_Str, sizeof(client_DIS_Manufacturer_Str)) ) {
        DEBUG_PRINTF(" -> Client Reads Manufacturer:  [%s]\n", client_DIS_Manufacturer_Str);
    }
  } // 1
  delay(poll_delay);
  //  2
  if ( client_DIS_ModelNumber_Chr.discover() ) {
    // read and print out Model Number
    if ( client_DIS_ModelNumber_Chr.read(client_DIS_ModelNumber_Str, sizeof(client_DIS_ModelNumber_Str)) ) { 
      DEBUG_PRINTF(" -> Client Reads Model Number:  [%s]\n", client_DIS_ModelNumber_Str);
     }
  } // 2
  delay(poll_delay);
  //  3
  if ( client_DIS_SerialNumber_Chr.discover() ) {
    // read and print out Serial Number
    if ( client_DIS_SerialNumber_Chr.read(client_DIS_SerialNumber_Str, sizeof(client_DIS_SerialNumber_Str)) ) {
      DEBUG_PRINTF(" -> Client Reads Serial Number: [%s]\n", client_DIS_SerialNumber_Str);
    }
  } // 3
  return true;
}
// ---------------------------- END GA and DIS SERVICE ------------------------------------------------

// -----------------------------FTM SERVICE ------------------------------------------------------------
bool client_FTM_connect_callback(uint16_t conn_handle)
{
  DEBUG_PRINT("Discovering Mandatory Client Fitness Machine (FTM) Service ... ");
  delay(poll_delay);
  // If FTM is not found, disconnect, resume scanning, and return
  if ( client_FitnessMachine_Service.discover(conn_handle) )
  {
#ifdef DEBUG
    BLEConnection* conn = Bluefruit.Connection(conn_handle);
    uint16_t max_payload = conn->getMtu()-3;
    uint16_t data_length = conn->getDataLength();
    DEBUG_PRINT("Found it! ");
    DEBUG_PRINTF("FTMS Max Payload: %d Data Length: %d\n", max_payload, data_length);
#endif
  } else {
    DEBUG_PRINTLN("Not Found!"); 
    DEBUG_PRINTLN("Disconnecting since Client FTM Service is mandatory!");
    return false;
  }
  DEBUG_PRINT("Discovering Client FTM Control Point Characteristic ... ");
  delay(poll_delay);
  // If FTM Control Point is not found, disconnect, resume scanning, and return
  if ( client_FTM_ControlPoint_Chr.discover() )
  {
    DEBUG_PRINTLN("Found it!");
  } else {
    DEBUG_PRINTLN("Not Found!"); 
    DEBUG_PRINTLN("Disconnecting since Client FTM Control Point Characteristic is mandatory!");
    return false;
  }
  DEBUG_PRINT("Discovering Client FTM Status Characteristic ... ");
  delay(poll_delay);
  // If FTM Status is not found, disconnect, resume scanning, and return
  if ( client_FTM_Status_Chr.discover() )
  {
    DEBUG_PRINTLN("Found it!");
  } else {
    DEBUG_PRINTLN("Not Found!"); 
    DEBUG_PRINTLN("Disconnecting since Client FTM Status Characteristic is mandatory!");
    return false;
  }
  DEBUG_PRINT("Discovering Client FTM Indoor Bike Data Characteristic ... ");
  delay(poll_delay);
  // FTM Indoor Bike Data is not mandatory
  if ( client_FTM_IndoorBikeData_Chr.discover() )
  {
    DEBUG_PRINTLN("Found it!");
  } else {
    DEBUG_PRINTLN("Not Found!"); 
    DEBUG_PRINTLN("Disconnecting since Client FTM Indoor Bike Data Characteristic is mandatory!");
    return false;
  }
  DEBUG_PRINT("Discovering Client FTM Feature Characteristic ... ");
  delay(poll_delay);
  // If FTM Feature is not found, disconnect, resume scanning, and return
  if ( client_FTM_Feature_Chr.discover() )
  {
    DEBUG_PRINTLN("Found it!");
    // Read FTM Feature Data
    client_FTM_Feature_Chr.read(client_FTM_Feature_Data, 8);
#ifdef DEBUG
    DEBUG_PRINT(" -> Client Reads Raw FTM Feature bytes: [8] [ ");
    for (int i = 0; i < sizeof(client_FTM_Feature_Data); i++) {
        DEBUG_PRINTF("%02X ", client_FTM_Feature_Data[i], HEX);
    } // for
    DEBUG_PRINTLN("] ");
    DEBUG_PRINTLN("- Fitness Machine Features:");
    // Load 32-bit client_CP_Feature_Chr value
    uint32_t client_FTM_Feature_Flags_One;
    memcpy(&client_FTM_Feature_Flags_One, &client_FTM_Feature_Data, 4);  
    for (int i = 0; i < sizeof(client_FTM_Feature_Str_One); i++) {
      if ( client_FTM_Feature_Flags_One & (1 << i) )
        {
          DEBUG_PRINTLN(client_FTM_Feature_Str_One[i]);
        }
      }
    DEBUG_PRINTLN("- Target Setting Features:");
    // Load 32-bit client_CP_Feature_Chr value
    uint32_t client_FTM_Feature_Flags_Two;
    memcpy(&client_FTM_Feature_Flags_Two, &client_FTM_Feature_Data[4], 4);
    for (int i = 0; i < sizeof(client_FTM_Feature_Str_Two); i++) {
      if ( client_FTM_Feature_Flags_Two & (1 << i) )
        {
          DEBUG_PRINTLN(client_FTM_Feature_Str_Two[i]);
        }
      }
#endif
  } else {
    DEBUG_PRINTLN("Disconnecting since Client FTM Feature Characteristic is mandatory!");
    return false;
  }
  DEBUG_PRINT("Discovering Client FTM Training Status Characteristic ... ");
  delay(poll_delay);
  // FTM Training Status is NOT MANDATORY
  if ( client_FTM_TrainingStatus_Chr.discover() )
  {
    DEBUG_PRINTLN("Found it!");
  } else {
    DEBUG_PRINTLN("Not Found! Not Mandatory");
  }
  DEBUG_PRINT("Discovering Client FTM Supported Resistance Level Range Characteristic ... ");
  delay(poll_delay);
  // FTM SupportedResistanceLevelRange is not mandatory!
  if ( client_FTM_SupportedResistanceLevelRange_Chr.discover() )
  {
    DEBUG_PRINTLN("Found it!");
    // Read Supported Resistance Level Range Data
    client_FTM_SupportedResistanceLevelRange_Chr.read(client_FTM_SupportedResistanceLevelRange_Data, 6);
#ifdef DEBUG
    DEBUG_PRINT(" -> Client Reads Raw FTM Supported Resistance Level Range bytes: [6] [ ");
    for (int i = 0; i < sizeof(client_FTM_SupportedResistanceLevelRange_Data); i++) {
        DEBUG_PRINTF("%02X ", client_FTM_SupportedResistanceLevelRange_Data[i], HEX);
    } // for
    DEBUG_PRINTLN("] ");
#endif
  } else {
    DEBUG_PRINTLN("Not Found! NOT mandatory!"); 
  }
  DEBUG_PRINT("Discovering Client FTM Supported Power Range Characteristic ... ");
  delay(poll_delay); 
  // FTM SupportedPowerRange is not mandatory!
  if ( client_FTM_SupportedPowerRange_Chr.discover() )
  {
    DEBUG_PRINTLN("Found it!");
    // Read Supported Resistance Level Range values
    client_FTM_SupportedPowerRange_Chr.read(client_FTM_SupportedPowerRange_Data, 6);
#ifdef DEBUG
    DEBUG_PRINT(" -> Client Reads Raw FTM Supported Power Range bytes: [6] [ ");
    for (int i = 0; i < sizeof(client_FTM_SupportedPowerRange_Data); i++) {
        DEBUG_PRINTF("%02X ", client_FTM_SupportedPowerRange_Data[i], HEX);
    } // for
    DEBUG_PRINTLN("] ");
#endif
  } else {
    DEBUG_PRINTLN("Not Found! NOT mandatory!"); 
  }
  return true;
}
// ---------------------------- End FTM SERVICE ---------------------------------------------

// ---------------------------- CP SERVICE --------------------------------------------------
bool client_CP_connect_callback(uint16_t conn_handle)
{
  DEBUG_PRINT("Discovering Client Cycling Power (CP) Service ... ");
  delay(poll_delay);
  // If CPS is not found, disconnect, resume scanning, and return
  if ( client_CyclingPower_Service.discover(conn_handle) )
  {
#ifdef DEBUG
    BLEConnection* conn = Bluefruit.Connection(conn_handle);
    uint16_t max_payload = conn->getMtu()-3;
    uint16_t data_length = conn->getDataLength();
    DEBUG_PRINT("Found it! ");
    DEBUG_PRINTF("CPS Max Payload: %d Data Length: %d\n", max_payload, data_length);
#endif
  } else {
    DEBUG_PRINTLN("Not Found!"); 
    DEBUG_PRINTLN("Disconnecting since Client Cyling Power Service is mandatory!");
    // MANDATORY so disconnect since we couldn't find the service
    return false;
  }
  DEBUG_PRINT("Discovering Client CP Measurement characteristic ... ");
  delay(poll_delay);
  if ( client_CP_Measurement_Chr.discover() )
  {
    DEBUG_PRINTLN("Found it!");
  } else {
    // Measurement chr is mandatory, if it is not found (valid), then disconnect
    DEBUG_PRINTLN("Not Found!");  
    DEBUG_PRINTLN("Disconnecting since Client CP Measurement Characteristic is mandatory!");
    return false;
  }
  DEBUG_PRINT("Discovering Client CP Feature characteristic ... ");
  delay(poll_delay);
  if ( client_CP_Feature_Chr.discover() )
  {
    DEBUG_PRINTLN("Found it!");
// Configure the Cycle Power Feature characteristic
// Properties = Read
// Min Len    = 1
// Max Len    = 32
//    B0:3    = UINT8 - Cycling Power Feature (MANDATORY)
//      b0    = Pedal power balance supported; 0 = false, 1 = true
//      b1    = Accumulated torque supported; 0 = false, 1 = true
//      b2    = Wheel revolution data supported; 0 = false, 1 = true
//      b3    = Crank revolution data supported; 0 = false, 1 = true
//      b4    = Extreme magnatudes supported; 0 = false, 1 = true
//      b5    = Extreme angles supported; 0 = false, 1 = true
//      b6    = Top/bottom dead angle supported; 0 = false, 1 = true
//      b7    = Accumulated energy supported; 0 = false, 1 = true
//      b8    = Offset compensation indicator supported; 0 = false, 1 = true
//      b9    = Offset compensation supported; 0 = false, 1 = true
//      b10   = Cycling power measurement characteristic content masking supported; 0 = false, 1 = true
//      b11   = Multiple sensor locations supported; 0 = false, 1 = true
//      b12   = Crank length adj. supported; 0 = false, 1 = true
//      b13   = Chain length adj. supported; 0 = false, 1 = true
//      b14   = Chain weight adj. supported; 0 = false, 1 = true
//      b15   = Span length adj. supported; 0 = false, 1 = true
//      b16   = Sensor measurement context; 0 = force, 1 = torque
//      b17   = Instantaineous measurement direction supported; 0 = false, 1 = true
//      b18   = Factory calibrated date supported; 0 = false, 1 = true
//      b19   = Enhanced offset compensation supported; 0 = false, 1 = true
//   b20:21   = Distribtue system support; 0 = legacy, 1 = not supported, 2 = supported, 3 = RFU
//   b22:32   = Reserved
  
  // Read 32-bit client_CP_Feature_Chr value
  client_CP_Feature_Flags = client_CP_Feature_Chr.read32();
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
  for (int i = 0; i < sizeof(client_CP_Feature_Str); i++) {
    if ( client_CP_Feature_Flags & (1 << i) )
      {
       DEBUG_PRINTLN(client_CP_Feature_Str[i]);
      }
    }
#endif
  } else {
    DEBUG_PRINTLN("NOT Found!");
    DEBUG_PRINTLN("Disconnecting since Client CP Feature Characteristic is mandatory!");
    return false;
  }
  DEBUG_PRINT("Discovering Client CP Control Point characteristic ... ");
  delay(poll_delay);
  if ( client_CP_ControlPoint_Chr.discover() )
  {
    // CP Control Point chr is not mandatory
    DEBUG_PRINTLN("Found it!");  
  } else {
    DEBUG_PRINTLN("Not Found! NOT Mandatory!");  
  }
  DEBUG_PRINT("Discovering Client CP Sensor Location characteristic ... ");
  delay(poll_delay);
  if ( client_CP_Location_Chr.discover() )
  {
    DEBUG_PRINTLN("Found it!");
  // The Sensor Location characteristic
  // Properties = Read
  // Min Len    = 1
  // Max Len    = 1
  //  B0:1      = UINT8 - Sensor Location
    // Read 8-bit client CP sensor location value
    client_CP_Location_Value = client_CP_Location_Chr.read8();   
    DEBUG_PRINT(" -> Client Reads CP Location Sensor: ");
    DEBUG_PRINTF("Loc#: %d %s\n", client_CP_Location_Value, client_Sensor_Location_Str[client_CP_Location_Value]);
  } else {
    DEBUG_PRINTLN("NOT Found! NOT Mandatory!");
  }
  return true;
}
// ---------------------------- END CP SERVICE ------------------------------------------------

// ---------------------------- CSC SERVICE --------------------------------------------------
bool client_CSC_connect_callback(uint16_t conn_handle) 
{
  DEBUG_PRINT("Discovering Cycling Speed and Cadence (CSC) Service ... ");
  delay(poll_delay);
  if ( client_CyclingSpeedCadence_Service.discover(conn_handle) ) //   UUID16_SVC_CYCLING_SPEED_AND_CADENCE
  {
#ifdef DEBUG
    BLEConnection* conn = Bluefruit.Connection(conn_handle);
    uint16_t max_payload = conn->getMtu()-3;
    uint16_t data_length = conn->getDataLength();
    DEBUG_PRINT("Found it! ");
    DEBUG_PRINTF("CSCS Max Payload: %d Data Length: %d\n", max_payload, data_length);
#endif
  } else {
    DEBUG_PRINTLN("Not Found!"); 
    DEBUG_PRINTLN("Disconnecting since CSC Service is mandatory!");
    return false; // NO CSC -> end of client_CSC_connect_callback !!   
  }
  // Test for client CSC Characteristics when the client CSC Service is existing
  DEBUG_PRINT("Discovering Client CSC Measurement CHR ... ");
  delay(poll_delay);
  if ( client_CSC_Measurement_Chr.discover() ) //   UUID16_CHR_CSC_MEASUREMENT
  {
    DEBUG_PRINTLN("Found it! ");
  } else {
    DEBUG_PRINTLN("Not Found!"); 
    DEBUG_PRINTLN("Disconnecting since Client CSC Measurement CHR is mandatory!");
    return false;    
  }
  DEBUG_PRINT("Discovering Client CSC Feature CHR ... ");   
  delay(poll_delay);
  if ( client_CSC_Feature_Chr.discover() ) //   UUID16_CHR_CSC_FEATURE
  {
    DEBUG_PRINTLN("Found it!");
    // Read sensor CSC Feature value in 16 bit
    client_CSC_Feature_Flags = client_CSC_Feature_Chr.read16();
#ifdef DEBUG
    uint8_t cscfcData[CSC_FEATURE_FIXED_DATALEN] = { (uint8_t)(client_CSC_Feature_Flags & 0xff), (uint8_t)(client_CSC_Feature_Flags >> 8) }; //  Little Endian Representation
    DEBUG_PRINTF(" -> Client Reads Raw CSC Feature bytes: [2] [ ");
    for (int i = 0; i < sizeof(cscfcData); i++) {
      DEBUG_PRINTF("%02X ", cscfcData[i], HEX);
    }
    DEBUG_PRINTLN("] ");
    for (int i = 0; i < sizeof(client_CSC_Feature_Str); i++) {
    if ( (client_CSC_Feature_Flags & (1 << i)) != 0 )
      {
       DEBUG_PRINTLN(client_CSC_Feature_Str[i]);
      }
    }
#endif
  } else {
      DEBUG_PRINTLN("Not Found!");
      DEBUG_PRINTLN("Disconnecting since Client CSC Feature CHR is mandatory!");
      return false;
  }
  DEBUG_PRINT("Discovering Client CSC Location CHR ... ");
  delay(poll_delay);
  if ( client_CSC_Location_Chr.discover() ) //   UUID16_CHR_SENSOR_LOCATION
  {
    DEBUG_PRINTLN("Found it!");
    // Read 16-bit client CSC sensor location value
    client_CSC_Location_Value = client_CSC_Location_Chr.read8();
    DEBUG_PRINT(" -> Client Reads CSC Location Sensor: ");
    DEBUG_PRINTF("Loc#: %d %s\n", client_CSC_Location_Value, client_Sensor_Location_Str[client_CSC_Location_Value]);
  } else {
      DEBUG_PRINTLN("Not Found! NOT Mandatory!");
  }
  return true;
}
// ---------------------------- CSC SERVICE --------------------------------------------------

bool client_HR_connect_callback(uint16_t conn_handle)
{
  DEBUG_PRINT("Discovering Heart Rate Measurement (HR) Service ... ");
  delay(poll_delay);
  if ( client_HeartRate_Service.discover(conn_handle) ) //   UUID16_SVC_HRM
  {
#ifdef DEBUG
    BLEConnection* conn = Bluefruit.Connection(conn_handle);
    uint16_t max_payload = conn->getMtu()-3;
    uint16_t data_length = conn->getDataLength();
    DEBUG_PRINT("Found it! ");
    DEBUG_PRINTF("HRMS Max Payload: %d Data Length: %d\n", max_payload, data_length);
#endif
  } else {
    DEBUG_PRINTLN("Not Found! HR Service is Not Mandatory!");
    return true; // NO HRM -> end of client_HR_connect_callback !!   
  }
  DEBUG_PRINT("Discovering HR Measurement characteristic ... ");
  delay(poll_delay);
  if ( client_HR_Measurement_Chr.discover() )
  {
    DEBUG_PRINTLN("Found it!");
  } else {
    // Is Mandatory when HR Service is found!
    DEBUG_PRINTLN("Not Found!"); 
    DEBUG_PRINTLN("Disconnecting since Client HR Measurement CHR is mandatory!");
    return false;
  }
  // Body Sensor Location is optional, print out the location in text if present
  DEBUG_PRINT("Discovering HR Sensor Location characteristic ... ");
  delay(poll_delay);
  if ( client_HR_Location_Chr.discover() )
    {
     // Read 8-bit client_HR_Location_Chr value from peripheral
     client_HR_Location_Value = client_HR_Location_Chr.read8();
#ifdef DEBUG
    DEBUG_PRINTLN("Found it!");
    // Body sensor location value is 8 bit
    const char* body_str[] = { "Other", "Chest", "Wrist", "Finger", "Hand", "Ear Lobe", "Foot" };
    DEBUG_PRINT(" -> Client Reads HR Location Sensor:");
    DEBUG_PRINTF(" Loc#: %d %s\n", client_HR_Location_Value, body_str[client_HR_Location_Value]);
#endif
  } else {
    DEBUG_PRINTLN("Not Found! Not Mandatory!");
  }
  return true;
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void client_disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  client_Connection_Handle = BLE_CONN_HANDLE_INVALID;
  DEBUG_PRINT("Client Disconnected, reason = 0x"); DEBUG_PRINTLN(reason, HEX);
  DEBUG_PRINTLN(">>> Restart the Feather nRF52 Client for a new run! <<<");
}

/**
 * Hooked callback that triggered when a measurement value is sent from peripheral
 * @param chr   Pointer client characteristic that even occurred,
 *              in this example it should be cpmc
 * @param data  Pointer to received data
 * @param len   Length of received data
 */
void client_CP_Measurement_Chr_notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
#ifdef DEBUG_CP_MEASUREMENT
uint8_t buffer[len]= {}; /*{     (uint8_t)(cpmcDef & 0xff), (uint8_t)(cpmcDef >> 8),  // flags 
                             (uint8_t)(powerOut & 0xff), (uint8_t)(powerOut >> 8),  // inst. power 
                                                                                0,  // bal
                                                                             0, 0,  // torque
                                                                       0, 0, 0, 0,  // cum. rev
                                                                             0, 0,  // wheel time
                                                                             0, 0,  // cum. crank
                                                                             0, 0,  // crank time
                                                                             0, 0,  // max force
                                                                             0, 0,  // min force
                                                                             0, 0,  // max tor
                                                                             0, 0,  // min tor
                                                                             0, 0,  // max ang
                                                                             0, 0,  // min ang
                                                                             0, 0,  // tdc
                                                                             0, 0,  // bdc
                                                                             0, 0 }; // total energy
*/
  // Transfer first the contents of data to buffer (array of chars)
  DEBUG_PRINTF(" -> Client Rec'd Raw CP Data: [%d] [ ", len); 
  for (int i = 0; i < sizeof(buffer); i++) {
    if ( i <= sizeof(buffer)) {
      buffer[i] = *data++;
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

/**
 * Hooked callback that triggered when a measurement value is sent from peripheral
 * @param chr   Pointer client characteristic that even occurred,
 *              in this example it should be client_CP_ControlPoint_Chr
 * @param data  Pointer to received data
 * @param len   Length of received data
 */
void client_CP_ControlPoint_Chr_indicate_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
/*
static const unsigned MAX_CONTROL_BYTES  = 5;
const uint8_t RESPONSE_SUCCESS = 1;
const uint8_t RESPONSE_OP_CODE_NOT_SUPPORTED = 2;
const uint8_t RESPONSE_INVALID_PARAMETER = 3;
const uint8_t RESPONSE_OPERATION_FAILED = 4;
const uint8_t OPCODE_RESPONSE_CODE = 32;
Len = 3
cpcpData[0] = Response Code = 0x20 // 32 Decimal
cpcpData[1] = OpCode
cpcpData[2] = responseValue
Len = 5
cpcpData[3] = (uint8_t)(responseParameter & 0xFF);
cpcpData[4] = (uint8_t)(responseParameter >> 8);
*/

// Handle the response message
  
#ifdef DEBUG
  uint8_t cpcpDataLen = (uint8_t)len;
  uint8_t cpcpData[cpcpDataLen]= {}; 
  // Transfer first the contents of data to buffer (array of chars)
  DEBUG_PRINTF(" -> Client Rec'd Raw CP Control Point Data: [%d] [ ", len); 
  for (int i = 0; i < sizeof(cpcpData); i++) {
      cpcpData[i] = *data++;
      DEBUG_PRINTF("%02X ", cpcpData[i], HEX);
  }
  DEBUG_PRINT("] ");
#endif
}

/**
 * Hooked callback that triggered when a measurement value is sent from peripheral
 * @param chr   Pointer client characteristic that even occurred,
 *              in this example it should be cpmc
 * @param data  Pointer to received data
 * @param len   Length of received data
 */
void client_CSC_Measurement_Chr_notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
#ifdef DEBUG_CSC_MEASUREMENT
  uint8_t buffer[len]= {}; 
  // Transfer first the contents of data to buffer (array of chars)
  DEBUG_PRINTF(" -> Client Rec'd Raw CSC Data: [%d] [ ", len); 
  for (int i = 0; i < len; i++) {
    if ( i <= sizeof(buffer)) {
      buffer[i] = *data++;
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
        
