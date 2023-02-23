/*********************************************************************
 This is programming code for the nRF52 based Bluefruit BLE boards
 
 The code uses heavily the Adafruit supplied Bluefruit BLE libraries !!
 Adafruit invests time and resources providing open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

        MIT license, check LICENSE for more information
        All text must be included in any redistribution
*********************************************************************/

/* -----------------------------------------------------------------------------------------------------
 *             This code should work with all indoor cycling trainers that fully support,
 *        Fitness Machine Service, Cycling Power Service and Cycling Speed & Cadence Service
 * ------------------------------------------------------------------------------------------------------
 *
 *  The code links a BLE Server (a peripheral to Zwift) and a BLE Client (a central to the Trainer) with a bridge 
 *  in between, the Feather nRF52 being man-in-the-middle (MITM). The Feather is an integral part of the Simcline,
 *  that interprets the exchanged road grade and moves the front wheel up and down with the change in inclination.
 *  The nRF52-bridge can control, filter and alter the bi-directional interchanged data!
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
 *  0) Upload and Run this code on the Simcline (i.c. Feather nRF52)
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

// -------------------------------------------------------------------------------------------
// COMPILER DIRECTIVE to allow/suppress SERIAL.PRINT messages that help debugging...
// Uncomment general "#define DEBUG" to activate
#define DEBUG
// Include these debug utility macros in all cases!
#include "DebugUtils.h"
//
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

#include <bluefruit.h>

const uint8_t MAX_PAYLOAD = 20; // Max 20 byte data size for single packet BLE transfer
// ----------- Some trainers need extra time to process a SVC/Char-discovery polls! -----------------
// --> delay(poll_delay) calls are inserted in the critical parts of the 'client connect' code!
// If calls to detect which SVC/Char's the trainer exposes are too quickly sequenced, time-out-induced 
// errors occur during the process! This manifests in erroneous responses about the presence of 
// exposed SVC/Char's and empty (zero) readings of Char values! Result: connection fails!
// - Zwift Hub will NOT pass 100%-error-free with a value lower than 250 ms !!
const unsigned long poll_delay = 0;  // Unit is milliseconds
//---------------------------------------------------------------------------------------------------

// Struct containing Device info to administer dis/connected devices
typedef struct
{
  uint8_t PeerAddress[6];
  char PeerName[MAX_PAYLOAD];
  uint16_t conn_handle;
  bool IsConnected;
} Device_info_t;
// -----------------------------------------------------------------
// Your hardware MAC/DEVICE ADDRESSES
// Laptop/Desktop Device Address that runs Zwift, in printed format: [00:01:02:03:04:05]
// Little Endian: in reversed order !!!!
#define LAPTOPADDRESS {0x05,0x04,0x03,0x02,0x01,0x00}
// Trainer FTMS enabled Device Address, in printed format: [00:01:02:03:04:05]
// Little Endian: in reversed order !!!!
#define TRAINERADDRESS {0x05,0x04,0x03,0x02,0x01,0x00}
// -----------------------------------------------------------------
// Initialize connectable device registration
  Device_info_t Trainer    = {TRAINERADDRESS, {0x00}, BLE_CONN_HANDLE_INVALID, false};
  Device_info_t Laptop     = { LAPTOPADDRESS, {0x00}, BLE_CONN_HANDLE_INVALID, false};
  Device_info_t Smartphone = {        {0x00}, {0x00}, BLE_CONN_HANDLE_INVALID, false};
// ----------------------------------------------------------------------------------

/* Generic Access
#define UUID16_SVC_GENERIC_ACCESS                             0x1800
#define UUID16_CHR_DEVICE_NAME                                0x2A00
#define UUID16_CHR_APPEARANCE                                 0x2A01
#define UUID16_CHR_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS 0x2A04 ---> not implemented
#define UUID16_CHR_CENTRAL_ADDRESS_RESOLUTION                 0x2AA6 ---> not implemented
*/
BLEClientService        client_GenericAccess_Service(UUID16_SVC_GENERIC_ACCESS); // Optional
BLEClientCharacteristic client_GA_Appearance_Chr(UUID16_CHR_APPEARANCE);         // Read
uint16_t client_GA_Appearance_Value = 0;
BLEClientCharacteristic client_GA_DeviceName_Chr(UUID16_CHR_DEVICE_NAME);        // Read, Write
unsigned char client_GA_DeviceName_Data[MAX_PAYLOAD] = {};

/* Cycling Power Service
 * CP Service: 0x1818  
 * CP Characteristic: 0x2A63 (Measurement)
 * CP Characteristic: 0x2A65 (Feature)
 * CP Characteristic: 0x2A5D (Location)
 * CP Characteristic: 0x2A66 (Control Point)
 */
BLEClientService        client_CyclingPower_Service(UUID16_SVC_CYCLING_POWER);            // Mandatory
BLEClientCharacteristic client_CP_Measurement_Chr(UUID16_CHR_CYCLING_POWER_MEASUREMENT);  // Notify, Read, Mandatory
BLEClientCharacteristic client_CP_Feature_Chr(UUID16_CHR_CYCLING_POWER_FEATURE);          // Read, optional
uint32_t client_CP_Feature_Flags = 0;
const uint8_t CP_FEATURE_DATALEN = 4; // Set MaxLen to 4
const char* client_CP_Feature_Str[] = { 
      "Pedal power balance supported","Accumulated torque supported","Wheel revolution data supported","Crank revolution data supported", \
      "Extreme magnitudes supported","Extreme angles supported","Top/bottom dead angle supported","Accumulated energy supported", \
      "Offset compensation indicator supported","Offset compensation supported","Cycling power measurement characteristic content masking supported", \
      "Multiple sensor locations supported","Crank length adj. supported","Chain length adj. supported","Chain weight adj. supported", \
      "Span length adj. supported","Sensor measurement context","Instantaineous measurement direction supported","Factory calibrated date supported", \
      "Enhanced offset compensation supported" };
BLEClientCharacteristic client_CP_Location_Chr(UUID16_CHR_SENSOR_LOCATION);                 // Read, optional
uint8_t client_CP_Location_Value = 0; // UINT8
const char* client_Sensor_Location_Str[] = { "Other", "Top of shoe", "In shoe", "Hip", "Front wheel", "Left crank", "Right crank", "Left pedal", \
      "Right pedal", "Front hub", "Rear dropout", "Chainstay", "Rear wheel", "Rear hub", "Chest", "Spider", "Chain ring"};
BLEClientCharacteristic client_CP_ControlPoint_Chr(UUID16_CHR_CYCLING_POWER_CONTROL_POINT); // Indicate, Write, optional
const uint16_t CP_CONTROL_POINT_DATALEN = 5;

/*
 * Cycling Speed and Cadence Service
 * CSC Service:   0x1816 
 * CSC Measurement Characteristic:  0x2A5B
 * CSC Feature Characteristic:      0x2A5C  
 * CSC Location Characteristic:     0x2A5D
 * CSC Control Point Characteristic:0x2A55 ---> not implemented
 */
BLEClientService        client_CyclingSpeedCadence_Service(UUID16_SVC_CYCLING_SPEED_AND_CADENCE); // Mandatory
BLEClientCharacteristic client_CSC_Measurement_Chr(UUID16_CHR_CSC_MEASUREMENT);                   // Notify, Read, Mandatory
BLEClientCharacteristic client_CSC_Feature_Chr(UUID16_CHR_CSC_FEATURE);                           // Read, optional
const uint8_t CSC_FEATURE_FIXED_DATALEN = 2; // UINT16
uint16_t client_CSC_Feature_Flags = 0;
const char* client_CSC_Feature_Str[] = {"Wheel rev supported", "Crank rev supported", "Multiple locations supported"};
BLEClientCharacteristic client_CSC_Location_Chr(UUID16_CHR_SENSOR_LOCATION);                      // Read, optional
uint8_t client_CSC_Location_Value = 0; 
// Shared with CPS --> client_Sensor_Location_Str[]

/* Service Device Information
#define UUID16_SVC_DEVICE_INFORMATION                         0x180A
#define UUID16_CHR_MODEL_NUMBER_STRING                        0x2A24
#define UUID16_CHR_SERIAL_NUMBER_STRING                       0x2A25
#define UUID16_CHR_FIRMWARE_REVISION_STRING                   0x2A26 ---> not implemented
#define UUID16_CHR_HARDWARE_REVISION_STRING                   0x2A27 ---> not implemented
#define UUID16_CHR_SOFTWARE_REVISION_STRING                   0x2A28 ---> not implemented
#define UUID16_CHR_MANUFACTURER_NAME_STRING                   0x2A29
*/
BLEClientService        client_DIS_Service(UUID16_SVC_DEVICE_INFORMATION);                    // Optional
BLEClientCharacteristic client_DIS_ManufacturerName_Chr(UUID16_CHR_MANUFACTURER_NAME_STRING); // Read
char client_DIS_Manufacturer_Str[MAX_PAYLOAD] = {};
BLEClientCharacteristic client_DIS_ModelNumber_Chr(UUID16_CHR_MODEL_NUMBER_STRING);           // Read
char client_DIS_ModelNumber_Str[MAX_PAYLOAD] = {};
BLEClientCharacteristic client_DIS_SerialNumber_Chr(UUID16_CHR_SERIAL_NUMBER_STRING);         // Read
char client_DIS_SerialNumber_Str[MAX_PAYLOAD] = {};

/* Fitness Machine Service
#define UUID16_SVC_FITNESS_MACHINE                            0x1826
#define UUID16_CHR_FITNESS_MACHINE_FEATURE                    0x2ACC
#define UUID16_CHR_INDOOR_BIKE_DATA                           0x2AD2
#define UUID16_CHR_TRAINING_STATUS                            0x2AD3
#define UUID16_CHR_SUPPORTED_SPEED_RANGE                      0x2AD4 ---> not implemented
#define UUID16_CHR_SUPPORTED_INCLINATION_RANGE                0x2AD5 ---> not implemented
#define UUID16_CHR_SUPPORTED_RESISTANCE_LEVEL_RANGE           0x2AD6
#define UUID16_CHR_SUPPORTED_HEART_RATE_RANGE                 0x2AD7 ---> not implemented
#define UUID16_CHR_SUPPORTED_POWER_RANGE                      0x2AD8
#define UUID16_CHR_FITNESS_MACHINE_CONTROL_POINT              0x2AD9
#define UUID16_CHR_FITNESS_MACHINE_STATUS                     0x2ADA
*/
BLEClientService client_FitnessMachine_Service(UUID16_SVC_FITNESS_MACHINE);         // Mandatory
// Service characteristics exposed by FTM Service
BLEClientCharacteristic client_FTM_Feature_Chr(UUID16_CHR_FITNESS_MACHINE_FEATURE); // Mandatory, Read
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
BLEClientCharacteristic client_FTM_IndoorBikeData_Chr(UUID16_CHR_INDOOR_BIKE_DATA); // Optional, Notify
BLEClientCharacteristic client_FTM_TrainingStatus_Chr(UUID16_CHR_TRAINING_STATUS);  // Optional, Read & Notify
BLEClientCharacteristic client_FTM_SupportedResistanceLevelRange_Chr(UUID16_CHR_SUPPORTED_RESISTANCE_LEVEL_RANGE); // Mandatory, Read
const uint8_t FTM_SRLR_FIXED_DATALEN = 6;
uint8_t client_FTM_SupportedResistanceLevelRange_Data[FTM_SRLR_FIXED_DATALEN];
BLEClientCharacteristic client_FTM_SupportedPowerRange_Chr(UUID16_CHR_SUPPORTED_POWER_RANGE);  // Mandatory, Read
const uint8_t FTM_SPR_FIXED_DATALEN = 6;
uint8_t client_FTM_SupportedPowerRange_Data[FTM_SPR_FIXED_DATALEN];
BLEClientCharacteristic client_FTM_ControlPoint_Chr(UUID16_CHR_FITNESS_MACHINE_CONTROL_POINT); // Mandatory, Write & Indicate
BLEClientCharacteristic client_FTM_Status_Chr(UUID16_CHR_FITNESS_MACHINE_STATUS);              // Mandatory, Notify

/* HRM Service Definitions
 * Heart Rate Monitor Service:  0x180D
 * Heart Rate Measurement Char: 0x2A37 (Mandatory)
 * Body Sensor Location Char:   0x2A38 (Optional)
 */
BLEClientService        client_HeartRate_Service(UUID16_SVC_HEART_RATE);
BLEClientCharacteristic client_HR_Measurement_Chr(UUID16_CHR_HEART_RATE_MEASUREMENT);
BLEClientCharacteristic client_HR_Location_Chr(UUID16_CHR_BODY_SENSOR_LOCATION);
uint8_t client_HR_Location_Value = 1; //Chest

//  ------------------------------------ START of SERVER DEFINITIONS -----------------------------------------------------
/* Cycling Speed and Cadence Service */
BLEService        server_CyclingSpeedCadence_Service = BLEService(UUID16_SVC_CYCLING_SPEED_AND_CADENCE); 
BLECharacteristic server_CSC_Measurement_Chr = BLECharacteristic(UUID16_CHR_CSC_MEASUREMENT);              // Notify, Read
BLECharacteristic server_CSC_Feature_Chr = BLECharacteristic(UUID16_CHR_CSC_FEATURE);                      // Read
BLECharacteristic server_CSC_Location_Chr = BLECharacteristic(UUID16_CHR_SENSOR_LOCATION);                 // Read

/* Cycling Power Service */
BLEService        server_CylingPower_Service = BLEService(UUID16_SVC_CYCLING_POWER);
BLECharacteristic server_CP_Measurement_Chr = BLECharacteristic(UUID16_CHR_CYCLING_POWER_MEASUREMENT);    // Notify, Read
BLECharacteristic server_CP_Feature_Chr = BLECharacteristic(UUID16_CHR_CYCLING_POWER_FEATURE);            // Read
BLECharacteristic server_CP_Location_Chr = BLECharacteristic(UUID16_CHR_SENSOR_LOCATION);                 // Read
BLECharacteristic server_CP_ControlPoint_Chr = BLECharacteristic(UUID16_CHR_CYCLING_POWER_CONTROL_POINT); // Indicate, Write

/* Fitness Machine Service */
BLEService server_FitnessMachine_Service = BLEService(UUID16_SVC_FITNESS_MACHINE);
BLECharacteristic server_FTM_Feature_Chr = BLECharacteristic(UUID16_CHR_FITNESS_MACHINE_FEATURE); // Read
BLECharacteristic server_FTM_IndoorBikeData_Chr = BLECharacteristic(UUID16_CHR_INDOOR_BIKE_DATA); // Notify
BLECharacteristic server_FTM_TrainingStatus_Chr = BLECharacteristic(UUID16_CHR_TRAINING_STATUS);  // Notify, Read
const uint8_t FTM_TRAINING_STATUS_FIXED_DATALEN = 2; // Fixed len
BLECharacteristic server_FTM_SupportedResistanceLevelRange_Chr = BLECharacteristic(UUID16_CHR_SUPPORTED_RESISTANCE_LEVEL_RANGE); // Read
BLECharacteristic server_FTM_SupportedPowerRange_Chr = BLECharacteristic(UUID16_CHR_SUPPORTED_POWER_RANGE);  // Read
BLECharacteristic server_FTM_ControlPoint_Chr = BLECharacteristic(UUID16_CHR_FITNESS_MACHINE_CONTROL_POINT); // Write & Indicate
BLECharacteristic server_FTM_Status_Chr = BLECharacteristic(UUID16_CHR_FITNESS_MACHINE_STATUS);              // Notify
const uint8_t FTM_STATUS_DATALEN = 7; // Max Len was: [3] --> Notice that with de Elite Direto the size is: [7] !!

/* Device Information Service helper class instance */
BLEDis server_bledis;  // Read
unsigned char FirmwareRevStr[] = "0.0.0";
unsigned char HardwareRevStr[] = "0.0.0";
unsigned char SoftwareRevStr[] = "0.0.0";

/* NORDIC UART SERVICE a.k.a. NUS
 * NUS Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 * NUS RXD    : 6E400002-B5A3-F393-E0A9-E50E24DCCA9E
 * NUS TXD    : 6E400003-B5A3-F393-E0A9-E50E24DCCA9E
 */
const uint8_t UUID_NUS_SERVICE[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E};
const uint8_t UUID_NUS_CHR_RXD[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E};
const uint8_t UUID_NUS_CHR_TXD[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E};
BLEService server_NordicUart_Service = BLEService(UUID_NUS_SERVICE);
BLECharacteristic server_NUS_RXD_Chr = BLECharacteristic(UUID_NUS_CHR_RXD); // Read (Receiving Data)
BLECharacteristic server_NUS_TXD_Chr = BLECharacteristic(UUID_NUS_CHR_TXD); // Notify (Sending Data)

// Heart Rate Measurement Server Services and chars
BLEService        server_HeartRate_Service = BLEService(UUID16_SVC_HEART_RATE);
BLECharacteristic server_HR_Measurement_Chr= BLECharacteristic(UUID16_CHR_HEART_RATE_MEASUREMENT);
BLECharacteristic server_HR_Location_Chr   = BLECharacteristic(UUID16_CHR_BODY_SENSOR_LOCATION);
#define HR_MEASUREMENT_DATALEN 4
uint8_t server_HR_Measurement_Data[HR_MEASUREMENT_DATALEN] = {0};

// -----------------------------  The Fitness Machine Control Point data type structure --------------------------------
// ---------------------------- Decoding is done in: server_FTM_ControlPoint_Chr_callback ------------------------------
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
// ----------------------- end of server_FTM_ControlPoint_Chr_callback definitions ----------------------------

// -------------------------------------- END OF SERVER DEFINITIONS -------------------------------------------

// --------------------------------------------------------------------------------
// Global Server variables for decoding of INDOOR BIKE RESISTANCE PARAMETERS
// --------------------------------------------------------------------------------
float wind_speed = 0;       // meters per second, resolution 0.001
float grade = 0;            // percentage, resolution 0.01
float crr = 0;              // Coefficient of rolling resistance, resolution 0.0001
float cw = 0;               // Wind resistance Kg/m, resolution 0.01;
// --------------------------------------------------------------------------------

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
#include <avr/dtostrf.h>

// LittleFS for internal storage of persistent data on the Feather nRF52
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;
// Managing persistence of some data with LittleFile system
// PeRSistent Data  --> PRS data
#define PRS_FILENAME "/prsdata.txt"
File file(InternalFS);
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

// Exponential EMA ALPHA filter definition
// Should be between low (10-40) is maximal and high (50-90) is minimal filtering
#ifndef EMA_ALPHA
#define EMA_ALPHA 60 // Value is in percentage 0-99. 
#endif

// Library code for low level measuring (VL6180X) and controlling UP and down movement
#include "Lifter.h" 

// Decalaration of Lifter Class for control of the low level up/down movement
Lifter lift;
// Global variables for Lifter position control --> RawGradeValue has been defined/set previously to flat road level!!
int16_t TargetPosition = map(RawgradeValue, RGVMIN, RGVMAX, MAXPOSITION, MINPOSITION);
bool IsBasicMotorFunctions = false; // Mechanical motor functions
bool IsControlBusy = false;

const unsigned long TimeSpan = (poll_delay*30 + 3000);  // Unit in millis, Time to wait for client_connect_callback to finish properly
unsigned long TimeInterval = 0;

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb
  DEBUG_PRINTLN("   <<--  SIMCLINE supporting: CPS, CSC, HRM and FTMS   -->>");
  DEBUG_PRINTLN("------------------------- Version 02.0 -------------------------");
  DEBUG_PRINTF("Discover polling delay setting: [%d]\n", poll_delay);
#endif
// ------------------------------------------------------
#if defined(ARDUINO_NRF52840_FEATHER)
// Allow for extra memory usage the nRF52840 has plenty!
    DEBUG_PRINTLN("Setting NRF52840 BLE configuration parameters!");
  Bluefruit.configUuid128Count(3); // 1 Service and 2 Char's NUS TX and RX
//    Bluefruit.configCentralBandwidth(BANDWIDTH_HIGH);
  Bluefruit.configCentralConn(BLE_GATT_ATT_MTU_DEFAULT, BLE_GAP_EVENT_LENGTH_DEFAULT * 2, BLE_GATTS_HVN_TX_QUEUE_SIZE_DEFAULT * 2, BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT);
//    Bluefruit.configPrphBandwidth(BANDWIDTH_HIGH);
  Bluefruit.configPrphConn(BLE_GATT_ATT_MTU_DEFAULT, BLE_GAP_EVENT_LENGTH_DEFAULT * 2, BLE_GATTS_HVN_TX_QUEUE_SIZE_DEFAULT * 2, BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT);
#endif
// -----------------------------------------------------------
#if defined(ARDUINO_NRF52832_FEATHER)
// Squeeze the memory to a minimum... to avoid nRF52832 out off memory errors
  Bluefruit.configUuid128Count(3); // 1 Service and 2 Char NUS TX and RX
  Bluefruit.configPrphBandwidth(BANDWIDTH_LOW); 
  Bluefruit.configCentralBandwidth(BANDWIDTH_LOW);
//    Bluefruit.configAttrTableSize(1024); // 1024
#endif
// -----------------------------------------------------------

  // LittleFS start the Littlefilesystem lib and see if we have persistent data ----
  InternalFS.begin();
  // WARNING --------- Uncomment only when strictly necessary!!! ---------
  /*
  InternalFS.format();
  DEBUG_PRINTLN("Wipe out all persistent data, including file(s)....");
  */
  // WARNING --------- Uncomment only when strictly necessary!!! ---------
  // Get or set (first time only) the values of relevant and crucial variables
  // to persistence, whith the Companion App the user can set these on the fly!
  // get or set the values of aRGVmax, aRGVmin, GradeChangeFactor in PRSdata.
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
    // Is working properly
    IsBasicMotorFunctions = true;
    DEBUG_PRINTLN("Simcline Basic Motor Funtions are working!!");
    // Put Simcline in neutral: flat road position
    // Init EMA filter at first call with flat road position as reference
    TargetPosition = EMA_TargetPositionFilter(TargetPosition); 
    SetNewActuatorPosition();
    ControlUpDownMovement();
  }

  // Initialize Bluefruit with maximum connections as Peripheral = 1, Central = 1
  Bluefruit.begin(1, 1);
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
    yield();
  }
  TimeInterval = millis() + TimeSpan; // ADD just enough time delay
  // Wait TimeSpan for background process: client_connect_callback to finish
  while( (millis() < TimeInterval) ) { 
    yield();
  }

  // Configure and Start the Device Information Service
  DEBUG_PRINTLN("Configuring the Server Device Information Service");
  server_setupDIS();
  // Setup the Cycle Power Service, Speed & Cadence Service and FTMS
  DEBUG_PRINTLN("Configuring the Server Cycle Power Service");
  server_setupCPS();
  DEBUG_PRINTLN("Configuring the Server Cadence and Speed Service");  
  server_setupCSC();
  DEBUG_PRINTLN("Configuring the Server Fitness Machine Service");  
  server_setupFTMS();
  DEBUG_PRINTLN("Configuring the Server HRM Service");  
  server_setupHRM();
  DEBUG_PRINTLN("Configuring the Server NUS Service");  
  server_setupNUS();
  // Setup and start advertising
  DEBUG_PRINTLN("Setting up the Server-side advertising payload(s)");
  server_startADV();
  DEBUG_PRINTLN("Server-side is CPS, CSC and FTMS advertising!");
  while (Bluefruit.Advertising.isRunning()) { // ONLY advertise!
    yield();
  }
/*
  TimeInterval = millis() + 3000; // ADD enough DELAY
  // wait enough time or go on when Server/Peripheral is connected and set!
  while ( (millis() < TimeInterval) || (!Laptop.IsConnected) ) {
    yield();
  }
  DEBUG_PRINTLN("Client- and Server-side are Up and Running!");
*/
}

// LittleFS --------------------------------------------------
bool getPRSdata(void) { // aRGVmax, aRGVmin, GradeChangeFactor -> PRSdata
  file.open(PRS_FILENAME, FILE_O_READ);
  if (file) {
    uint32_t readLen;
    char buffer[64] = { 0 };
    file.seek(1); // always read first record!!
    readLen = file.read(buffer, sizeof(buffer));
    buffer[readLen] = 0; // set the values to the pointed by variables
    sscanf(buffer, "%d %d %d %d", &aRGVmax, &aRGVmin, &GradeChangeFactor, &OledDisplaySelection);
    DEBUG_PRINT(F("Feather internal Get & Set PRSdata to "));
    DEBUG_PRINTF("Max: %d Min: %d Perc.: %d Displ.: %d\n", aRGVmax, aRGVmin, GradeChangeFactor, OledDisplaySelection);
    file.close();
    return true;
  }
  return false;
}

void setPRSdata(void) { // aRGVmax, aRGVmin, GradeChangeFactor -> PRSdata
  if (file.open(PRS_FILENAME, FILE_O_WRITE )) {
    char buffer[64] = { 0 };
    sprintf(buffer, "%d %d %d %d", aRGVmax, aRGVmin, GradeChangeFactor, OledDisplaySelection);
    file.seek(1); // always overwrite first record!!
    file.write(buffer, sizeof(buffer));
    file.close();
    DEBUG_PRINT(F("Feather internal Set new values of PRSdata in persistent storage: "));
    DEBUG_PRINTLN(buffer);
  }
}
// LittleFS --------------------------------------------------


int16_t EMA_TargetPositionFilter(int16_t current_value) {
  static int16_t exponential_average = current_value;

  exponential_average = int16_t( (EMA_ALPHA * (uint32_t)current_value + (100 - EMA_ALPHA) * (uint32_t)exponential_average) / 100 );
  return exponential_average;
}

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
} // ---------------------------------------------------------------

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

void SetNewActuatorPosition(void)
{
  // Handle mechanical movement i.e. wheel position in accordance with Road Inclination
  // Map RawgradeValue ranging from 0 to 40.000 on the
  // TargetPosition (between MINPOSITION and MAXPOSITION) of the Lifter
  // Notice 22000 is equivalent to +20% incline and 19000 to -10% incline
  RawgradeValue = constrain(RawgradeValue, RGVMIN, RGVMAX); // Keep values within the safe range
  TargetPosition = map(RawgradeValue, RGVMIN, RGVMAX, MAXPOSITION, MINPOSITION);
  // EMA filter for smoothing quickly fluctuating Target Position values see: Zwift Titan Groove
  TargetPosition = EMA_TargetPositionFilter(TargetPosition); 
  lift.SetTargetPosition(TargetPosition);
#ifdef MOVEMENTDEBUG
  DEBUG_PRINTF("RawgradeValue: %05d Grade percent: %03.1f%% ", RawgradeValue, gradePercentValue);
  DEBUG_PRINTF("TargetPosition: %03d\n", TargetPosition, DEC);
#endif
}

void ControlUpDownMovement(void) // Move fully controlled to the right position
{
  // Check Position and move Up/Down until target position is reached, 
  // BLE channels can interrupt and change target position on-the-fly !!
 
  if (!IsBasicMotorFunctions) { return; } // do nothing that can damage construction!!
  int OnOffsetAction = 0;
  IsControlBusy = true;
do {
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
  }
} while ( (OnOffsetAction == 1) || (OnOffsetAction == 2) ); // Run the loop until target position is reached!
  IsControlBusy = false;
} // end 

// ----------------------   CLIENT SIDE FUNCTIONS  -------------------------

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
  // Client CP Measurement data is tranferred to the Server (Zwift)
  // NO TREATMENT OF RESPONSE !!!!!
  if (server_HR_Measurement_Chr.notifyEnabled(Laptop.conn_handle)) {
    server_HR_Measurement_Chr.notify(data, len); // Just pass on and process later!
  }
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
  DEBUG_PRINTLN("FTM Service and Chars are 'initialized'");
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
  DEBUG_PRINTLN("CP Service and Chars are 'initialized'");
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
  DEBUG_PRINTLN("CSC Service and Chars are 'initialized'");
}  

void Setup_Client_DIS(void)
{
  // Initialize client Generic Access Service
  client_GenericAccess_Service.begin();
  // Initialize some characteristics of the Generic Access Service.
  client_GA_DeviceName_Chr.begin();
  client_GA_Appearance_Chr.begin();  
  DEBUG_PRINTLN("Generic Access Service and Chars are 'initialized'");
  // Initialize client Device Information Service
  client_DIS_Service.begin();
  // Initialize some characteristics of the Device Information Service.
  client_DIS_ManufacturerName_Chr.begin();
  client_DIS_ModelNumber_Chr.begin();
  client_DIS_SerialNumber_Chr.begin();
  DEBUG_PRINTLN("Device Information Service and Chars are 'initialized'");
}  

void loop()
{
  delay(20);
} // end loop

void Client_Start_Scanning(void)
{
  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Don't use active scan
   * - Filter only accept CP service
   * - Start(timeout) with timeout = 0 will scan forever (until connected)
   */
  // Client/Central Callbacks defined
  Bluefruit.Central.setDisconnectCallback(client_disconnect_callback);
  Bluefruit.Central.setConnectCallback(client_connect_callback);
  Bluefruit.Scanner.setRxCallback(client_scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true); // default is true! In test we do not want to RESTART!
  Bluefruit.Scanner.filterRssi(-80);   // We want to scan only nearby peripherals, so get close to your device !!
  Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms
  // Invoke callback if CPS or CSC or FTMS is advertised
  Bluefruit.Scanner.filterUuid(UUID16_SVC_CYCLING_POWER, UUID16_SVC_CYCLING_SPEED_AND_CADENCE, UUID16_SVC_FITNESS_MACHINE);  
  Bluefruit.Scanner.useActiveScan(true);
  Bluefruit.Scanner.start(0);  // 500 // 0 = Don't stop scanning or n = after n/100 seconds
  DEBUG_PRINTLN("Start Client-side Scanning for CPS, CSC and FTMS!");
  delay(100); // To show print message !
}

/**
 * Hooked callback that triggered when a status value is sent
 * @param chr   Pointer client characteristic  
 * @param data  Pointer to received data
 * @param len   Length of received data
 */
void client_FTM_TrainingStatus_Notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
  // Client Training Status data is tranferred to the Server
  // NO TREATMENT OF COMMAND !!!
  if (server_FTM_TrainingStatus_Chr.notifyEnabled(Laptop.conn_handle)) {
    server_FTM_TrainingStatus_Chr.notify(data, len); // Just pass on and process later! 
  }
#ifdef DEBUG_FTM_TRAININGSTATUS
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
 * Hooked callback that is triggered when an IBD value is sent
 * @param chr   Pointer client characteristic  
 * @param data  Pointer to received data
 * @param len   Length of received data
 */
void client_FTM_IndoorBikeData_Notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
  // Client IBD data is tranferred to the Server
  // NO TREATMENT OF COMMAND !!!
  if (server_FTM_IndoorBikeData_Chr.notifyEnabled(Laptop.conn_handle)) { 
    server_FTM_IndoorBikeData_Chr.notify(data, len); // Just pass on and process later!
  }
#ifdef DEBUG_FTM_INDOORBIKEDATA
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

/**
 * Hooked callback that triggered when a value is sent
 * @param chr   Pointer client characteristic
 * @param data  Pointer to received data
 * @param len   Length of received data
 */
void client_FTM_ControlPoint_Indicate_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{   
  // The receipt of Control Point settings is acknowledged by the trainer: handle it
  // Send Client's Response message to the Server
  // NO TREATMENT OF COMMAND !!!
  if(server_FTM_ControlPoint_Chr.indicateEnabled()) {
    server_FTM_ControlPoint_Chr.indicate(data, len); // Just pass on and process later!
  }
  
#ifdef DEBUG_FTM_CONTROLPOINT_RESPONSE
  uint8_t RespBufferLen = (uint8_t)len;
  uint8_t RespBuffer[RespBufferLen] = {}; // It is max 6 bytes long
  // Transfer first the contents of data to buffer (array of chars)
  DEBUG_PRINT(" -> Client Rec'd Control Point Response: [ "); 
  for (int i = 0; i < sizeof(RespBuffer); i++) {
      RespBuffer[i] = *data++;
      DEBUG_PRINTF("%02X ", RespBuffer[i], HEX);
  }
  DEBUG_PRINTLN("] ");
#endif

}

/**
 * Hooked callback that triggered when a value is sent
 * @param chr   Pointer client characteristic  
 * @param data  Pointer to received data
 * @param len   Length of received data
 */
void client_FTM_Status_Notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
  // Client's Machine Status data is tranferred to the Server
  // NO TREATMENT OF COMMAND !!!
  if (server_FTM_Status_Chr.notifyEnabled(Laptop.conn_handle)) {
    server_FTM_Status_Chr.notify(data, len); // Just pass on and process later!
  }
#ifdef DEBUG_FTM_STATUS
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

 // Byte swap unsigned short
uint16_t swap_uint16( uint16_t val ) 
{
    return (val << 8) | (val >> 8 );
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
  uint8_t Device_Addr[6] = {0};
  if (!checkForUuidPresent(UUID16_SVC_FITNESS_MACHINE, report->data.p_data, report->data.len)) {
    return; // Keep scanning for FTMS trainer !!
  } 
  memcpy(Device_Addr, report->peer_addr.addr, 6);
  if ( !(memcmp(Device_Addr, Trainer.PeerAddress, 6) == 0) ) {
    DEBUG_PRINTLN("No match with required Mac Address -> keep scanning!");
    return; // Keep scanning for the required trainer !!
  }
  // Connect to device only with required services AND device address
  if (Bluefruit.Scanner.isRunning()) { Bluefruit.Scanner.stop(); }
#ifdef DEBUG  
  DEBUG_PRINTLN("Found Advertising Peripheral with FTMS, CPS and CSC! See Raw data packet:");
  DEBUG_PRINTLN(F("Timestamp Addr              Rssi Data"));
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
   Callback invoked when a connection is established
   @param conn_handle
*/
void client_connect_callback(uint16_t conn_handle)
{
  /* Wait for connection to be established */
  while (!Bluefruit.connected()) {
    DEBUG_PRINTLN("Client Waiting for Connection...");
    delay(500);
  }

  char Peer_Name[MAX_PAYLOAD] = {0};
  Trainer.conn_handle = conn_handle;
  Trainer.IsConnected = true;
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);
  connection->getPeerName(Peer_Name, sizeof(Peer_Name));
  memcpy(Trainer.PeerName, Peer_Name, sizeof(Peer_Name));
  ble_gap_addr_t PeerAddr = connection->getPeerAddr(); // Fill BLE Gap struct
  memcpy(Trainer.PeerAddress, PeerAddr.addr, 6); // Copy Peer Address from ble_gap_addr_t struct
  Peer_Name[9] = 0; // Cut off name at 9
  ShowOnOledLarge("Pairing", (const char*)Peer_Name, "Done!", 500);

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
  DEBUG_PRINTF("Feather nRF52 (Central) connected to Trainer (Peripheral) device: [%s] MAC Address: ", Trainer.PeerName);
  PrintPeerAddress(Trainer.PeerAddress);
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
  
/*
 * Callback invoked when a connection is dropped
 * @param conn_handle
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void client_disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  char disc_reason[10] = {0};
  sprintf(disc_reason, "Reason:%2X", reason);

  DEBUG_PRINTF("Client disconnected from Peripheral Device: [%s], reason: [%02X]\n", Trainer.PeerName, reason, HEX);
  Trainer.conn_handle = BLE_CONN_HANDLE_INVALID;
  Trainer.IsConnected = false;
  Laptop.PeerName[9] = 0; // Cut off name at 9
  ShowOnOledLarge("Lost!", (const char*)Trainer.PeerName, (const char*)disc_reason, 500);
  // Default RestartOnDisconnect == true
  DEBUG_PRINTLN("Client is Scanning again!");
}

/**
 * Hooked callback that triggered when a measurement value is sent from peripheral
 * @param chr   Pointer client characteristic   
 *              in this example it should be cpmc
 * @param data  Pointer to received data
 * @param len   Length of received data
 */
void client_CP_Measurement_Chr_notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
  // Client CP Measurement data is tranferred to the Server (Zwift)
  // NO TREATMENT OF RESPONSE !!!!!
  if (server_CP_Measurement_Chr.notifyEnabled(Laptop.conn_handle)) { 
    server_CP_Measurement_Chr.notify(data, len); // Just pass on and process later!
  }
#ifdef DEBUG_CP_MEASUREMENT
  uint8_t buffer[len]= {};
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
 * Hooked callback that triggered when a response value is sent from peripheral
 * @param chr   Pointer client characteristic
 * @param data  Pointer to received data
 * @param len   Length of received data
 */
void client_CP_ControlPoint_Chr_indicate_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
  // Send Client's response message to the Server (Zwift)
  // NO TREATMENT OF RESPONSE !!!!!
  if(server_CP_ControlPoint_Chr.indicateEnabled()) {
    server_CP_ControlPoint_Chr.indicate(data, len); // Just pass on and process later!
  }
/*  
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
*/
}

/**
 * Hooked callback that triggered when a measurement value is sent from peripheral
 * @param chr   Pointer client characteristic   
 * @param data  Pointer to received data
 * @param len   Length of received data
 */
void client_CSC_Measurement_Chr_notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
  // Client CSC Measurement data is transferred to the Server (Zwift)
  // NO TREATMENT OF RESPONSE !!!!!
  if (server_CSC_Measurement_Chr.notifyEnabled(Laptop.conn_handle)) { 
    server_CSC_Measurement_Chr.notify(data, len); // Just pass on and process later!
  }
#ifdef DEBUG_CSC_MEASUREMENT
  uint8_t buffer[len]= {}; 
  // Transfer first the contents of data to buffer (array of chars)
  DEBUG_PRINTF(" -> Client Rec'd Raw CSC Data: [%d] [ ", len); 
  for (int i = 0; i < sizeof(buffer); i++) {
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
  }
  DEBUG_PRINTLN();
#endif
}
// ----------------------   END of CLIENT SIDE FUNCTIONS   -------------------------  
// ----------------------  START of SERVER SIDE FUNCTIONS  ------------------------- 

void server_setupHRM(void)
{
  server_HeartRate_Service.begin();
  // Set the HRM Measurement characteristic
  server_HR_Measurement_Chr.setProperties(CHR_PROPS_NOTIFY);  
  server_HR_Measurement_Chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS); 
  server_HR_Measurement_Chr.setFixedLen(HR_MEASUREMENT_DATALEN);
  server_HR_Measurement_Chr.setCccdWriteCallback(server_cccd_callback);  // Optionally capture CCCD updates
  server_HR_Measurement_Chr.begin();
  // Configure the Sensor Location characteristic
  server_HR_Location_Chr.setProperties(CHR_PROPS_READ);
  server_HR_Location_Chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  server_HR_Location_Chr.setFixedLen(1); // uint8_t
  server_HR_Location_Chr.begin();
  server_HR_Location_Chr.write8(client_HR_Location_Value);  // Set the characteristic
}

void server_setupNUS(void)
{
  server_NordicUart_Service.begin();
  // Add NUS TXD Characteristic 
  server_NUS_TXD_Chr.setProperties(CHR_PROPS_NOTIFY);  // Type "notify"
  server_NUS_TXD_Chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS); // readAccess, NO writeAccess
  server_NUS_TXD_Chr.setMaxLen(MAX_PAYLOAD); // To be on the safe side! 
  server_NUS_TXD_Chr.setCccdWriteCallback(server_cccd_callback);  // Optionally capture CCCD updates
  server_NUS_TXD_Chr.begin();
  
  // Add NUS RXD Characteristic
  server_NUS_RXD_Chr.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP); // Write with No response !!
  server_NUS_RXD_Chr.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN); 
  server_NUS_RXD_Chr.setMaxLen(MAX_PAYLOAD); // Maxlen
  server_NUS_RXD_Chr.setWriteCallback(server_NUS_RXD_Chr_callback);
  server_NUS_RXD_Chr.begin();
}

void server_NUS_RXD_Chr_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  // Read data received over NUS RXD from Mobile Phone
  uint8_t NusRxdDataLen = (uint8_t)len;  // Get the actual length of data bytes and type cast to (uint8_t)
  char NusRxdData[MAX_PAYLOAD+1];        // Data is all ASCII !
  memset(NusRxdData, 0, MAX_PAYLOAD);    // set to zero
  if(NusRxdDataLen > MAX_PAYLOAD) { NusRxdDataLen = MAX_PAYLOAD; } // Check for limit
  memcpy(NusRxdData, data, NusRxdDataLen); // Transfer data to char array
  // Display the raw packet data in actual length
  DEBUG_PRINTF(" -> Server Rec'd NUS RXD Data [%d][%s] ", NusRxdDataLen, NusRxdData);
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
    sscanf(NusRxdData, "!S%d;%d;%d;%d;", &iMax, &iMin, &iPerc, &iDispl);
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
    server_NUS_TXD_Chr.notify("!SDone!;", 8);
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
   // Modification !!
   ControlUpDownMovement();
   return;
  }
  if (NusRxdData[1] == 'D' && IsBasicMotorFunctions) {
    DEBUG_PRINTLN("-> Set motor DOWNward moving!");
    RawgradeValue = RawgradeValue - 100;
//    SetManualGradePercentValue();
//    ShowSlopeTriangleOnOled();
    // Modification !!
//    ControlUpDownMovement();
    return;
  } 
  server_NUS_TXD_Chr.notify("!UOut of Order!;", 16);
  DEBUG_PRINTF("\nServer Sends NUS TXD Error message: Out of Order!\n");
}  

void Construct_Dev_Name(void)
{
  const char prefix[] = {'S','i','m',' '}; // #4 Chars
  size_t len = strlen((const char*)client_GA_DeviceName_Data); // Len of null terminated char array
  int MaxLen = (MAX_PAYLOAD-sizeof(prefix)-1); // 1 less for null terminating char
  if(len > MaxLen) { len = MaxLen; } 
  int pos = MaxLen;
  if (len > 0) {
    // pfound points to the first occurence of " " (blank space char)
    char *pfound = strstr((const char*)client_GA_DeviceName_Data, " "); 
    if (pfound != NULL) { pos = int(pfound - (char*)client_GA_DeviceName_Data); } // Convert to position in DevName
  }
  if( pos > MaxLen ) { pos = MaxLen; } // Stay within char array allocated memory!
  memmove(&client_GA_DeviceName_Data[sizeof(prefix)], &client_GA_DeviceName_Data, pos); // Make space: shift to the right
  memcpy(&client_GA_DeviceName_Data, &prefix, sizeof(prefix)); // Insert prefix at begin of DevName
  client_GA_DeviceName_Data[(pos+sizeof(prefix))] = 0; // Make null terminated char array at new position, skip rest!
}

void server_startADV(void)
{
// Setup and start advertising
  // Set blink rate in advertising mode
  Bluefruit.setConnLedInterval(250); 
  Construct_Dev_Name();
  DEBUG_PRINTF("Setting Server Device Name to:  [%s]\n", client_GA_DeviceName_Data);
  Bluefruit.setName((const char*)client_GA_DeviceName_Data);
  if (Bluefruit.setAppearance(client_GA_Appearance_Value))
  {
    DEBUG_PRINTF("Setting Server Appearance to: [%d]\n", client_GA_Appearance_Value);
  }
  // Set the connect/disconnect callback handlers
  Bluefruit.Periph.setConnectCallback(server_connect_callback);
  Bluefruit.Periph.setDisconnectCallback(server_disconnect_callback);

  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Test include only FTMS Service as defined above
  Bluefruit.Advertising.addService(server_CylingPower_Service);  
  Bluefruit.Advertising.addService(server_CyclingSpeedCadence_Service);
  Bluefruit.Advertising.addService(server_FitnessMachine_Service);
  // No need to advertise NUS, Companion App detects it anyway!
  
  // Include Bluefruit.Advertising.addName and 128-bit uuid(s) result in a packet space problem!!!
  // Use secondary Scan Response packet (optional)
  // if there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
  
  /* 
   * - Enable auto advertising Yes/NO if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
  */
  Bluefruit.Advertising.restartOnDisconnect(true);  // false --> at test stage we do NOT want to auto RESTART
  Bluefruit.Advertising.setInterval(32, 244);       // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);         // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                   // 0 = Don't stop advertising after n seconds  
}

void server_setupDIS(void)
{
  // Configure and Start the Device Information Service
  server_bledis.setManufacturer((const char*)client_DIS_Manufacturer_Str); 
  server_bledis.setModel((const char*)client_DIS_ModelNumber_Str);
  // Notice that the Firmware Revision string is default set to 
  // the value of the Feather-nRF52 Board being used!
  server_bledis.setFirmwareRev((const char*)FirmwareRevStr); 
  server_bledis.setSerialNum((const char*)client_DIS_SerialNumber_Str); 
  server_bledis.setHardwareRev((const char*)HardwareRevStr);
  server_bledis.setSoftwareRev((const char*)SoftwareRevStr); 
  server_bledis.begin();
}

void server_setupFTMS(void)
{
  server_FitnessMachine_Service.begin(); //
  
  // Fitness Machine Feature, mandatory, read
  server_FTM_Feature_Chr.setProperties(CHR_PROPS_READ);
  server_FTM_Feature_Chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  server_FTM_Feature_Chr.setFixedLen(FTM_FEATURE_FIXED_DATALEN);
  server_FTM_Feature_Chr.begin();
  server_FTM_Feature_Chr.write(client_FTM_Feature_Data, FTM_FEATURE_FIXED_DATALEN);

  // Indoor Bike Data, optional, notify
  server_FTM_IndoorBikeData_Chr.setProperties(CHR_PROPS_NOTIFY); 
  server_FTM_IndoorBikeData_Chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS); 
  server_FTM_IndoorBikeData_Chr.setMaxLen(MAX_PAYLOAD); // To be on the safe side, when many features are set! 
  server_FTM_IndoorBikeData_Chr.setCccdWriteCallback(server_cccd_callback);  // Optionally capture CCCD updates
  server_FTM_IndoorBikeData_Chr.begin();

  // Training Status, optional, read & notify
  server_FTM_TrainingStatus_Chr.setProperties(CHR_PROPS_NOTIFY);  
  server_FTM_TrainingStatus_Chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  server_FTM_TrainingStatus_Chr.setFixedLen(FTM_TRAINING_STATUS_FIXED_DATALEN);
  server_FTM_TrainingStatus_Chr.setCccdWriteCallback(server_cccd_callback);  // Optionally capture CCCD updates
  server_FTM_TrainingStatus_Chr.begin();

  // Supported Resistance Level Range, read, optional
  server_FTM_SupportedResistanceLevelRange_Chr.setProperties(CHR_PROPS_READ);
  server_FTM_SupportedResistanceLevelRange_Chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  server_FTM_SupportedResistanceLevelRange_Chr.setFixedLen(FTM_SRLR_FIXED_DATALEN);
  server_FTM_SupportedResistanceLevelRange_Chr.begin();
  server_FTM_SupportedResistanceLevelRange_Chr.write(client_FTM_SupportedResistanceLevelRange_Data, FTM_SRLR_FIXED_DATALEN);

  // Supported Power Range, read, optional
  server_FTM_SupportedPowerRange_Chr.setProperties(CHR_PROPS_READ);
  server_FTM_SupportedPowerRange_Chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  server_FTM_SupportedPowerRange_Chr.setFixedLen(FTM_SPR_FIXED_DATALEN);
  server_FTM_SupportedPowerRange_Chr.begin();
  server_FTM_SupportedPowerRange_Chr.write(client_FTM_SupportedPowerRange_Data, FTM_SPR_FIXED_DATALEN);

  // Fitness Machine Control Point, optional, write 
  server_FTM_ControlPoint_Chr.setProperties(CHR_PROPS_INDICATE | CHR_PROPS_WRITE); 
  server_FTM_ControlPoint_Chr.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  server_FTM_ControlPoint_Chr.setMaxLen(FTM_CONTROL_POINT_DATALEN); // Maxlen of Client written data: (1) OpCode and (FTM_CONTROL_POINT_DATALEN-1) OCTETS
  server_FTM_ControlPoint_Chr.setWriteCallback(server_FTM_ControlPoint_Chr_callback);
  server_FTM_ControlPoint_Chr.setCccdWriteCallback(server_cccd_callback);  // Optionally capture CCCD updates
  server_FTM_ControlPoint_Chr.begin();

  // Fitness Machine Status, mandatory, notify  BLENotify, 
  server_FTM_Status_Chr.setProperties(CHR_PROPS_NOTIFY);  
  server_FTM_Status_Chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS); 
  server_FTM_Status_Chr.setMaxLen(FTM_STATUS_DATALEN); 
  server_FTM_Status_Chr.setCccdWriteCallback(server_cccd_callback); 
  server_FTM_Status_Chr.begin();
}

void server_FTM_ControlPoint_Chr_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  // Server FTM Control Point data is tranferred to the Client
  // NO TREATMENT OF COMMAND !!!
  // Control Point write is documented to be write_resp !!!
  client_FTM_ControlPoint_Chr.write_resp(data, len); // Just pass on and process later! 
  
  uint8_t ftmcpDataLen = (uint8_t)len;
  memset(server_FTM_Control_Point_Data.bytes, 0, sizeof(server_FTM_Control_Point_Data.bytes));
  // Transfer the contents of data to server_FTM_Control_Point_Data.bytes
  for (int i = 0; i < ftmcpDataLen; i++) {
      server_FTM_Control_Point_Data.bytes[i] = *data++;
    }
/* Decodes an incoming Fitness Machine Control Point request */
#ifdef DEBUG_FTM_CONTROLPOINT_OPCODE_DATA
    DEBUG_PRINTF(" -> Server Rec'd Raw FTM Control Point Data [len: %d] ", ftmcpDataLen);
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
      DEBUG_PRINTLN("Set Indoor Bike Simulation Parameters!");
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
  } // End switch
  // Whatever the control point setting: check the actuator's targeted position!
  // Only when NOT busy doing so!
  if(!IsControlBusy) { ControlUpDownMovement(); }
}

void server_setupCPS(void)
{
  // Configure the Cycling Power service
  server_CylingPower_Service.begin();
  
  server_CP_Measurement_Chr.setProperties(CHR_PROPS_NOTIFY); 
  server_CP_Measurement_Chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS); 
  server_CP_Measurement_Chr.setMaxLen(MAX_PAYLOAD); // Will work in most cases!
  server_CP_Measurement_Chr.setCccdWriteCallback(server_cccd_callback);  // Optionally capture CCCD updates
  server_CP_Measurement_Chr.begin();
  
  server_CP_ControlPoint_Chr.setProperties(CHR_PROPS_INDICATE | CHR_PROPS_WRITE); 
  server_CP_ControlPoint_Chr.setPermission(SECMODE_OPEN, SECMODE_OPEN); 
  server_CP_ControlPoint_Chr.setMaxLen(CP_CONTROL_POINT_DATALEN); // The charactersitic's data set varies in length
  server_CP_ControlPoint_Chr.setCccdWriteCallback(server_cccd_callback); 
  server_CP_ControlPoint_Chr.begin();
  server_CP_ControlPoint_Chr.setWriteCallback(server_CP_ControlPoint_Chr_callback); // Respond to events with "Write with Response" !!

  server_CP_Feature_Chr.setProperties(CHR_PROPS_READ);
  server_CP_Feature_Chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  server_CP_Feature_Chr.setMaxLen(CP_FEATURE_DATALEN);
  server_CP_Feature_Chr.begin();
  server_CP_Feature_Chr.write32(client_CP_Feature_Flags);
   
  server_CP_Location_Chr.setProperties(CHR_PROPS_READ);
  server_CP_Location_Chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  server_CP_Location_Chr.setFixedLen(1); // UINT8
  server_CP_Location_Chr.begin();
  server_CP_Location_Chr.write8(client_CP_Location_Value);  // Set the characteristic
}

void server_setupCSC()
{
  // Configure the Cadence and Speed service
  server_CyclingSpeedCadence_Service.begin();
  
  server_CSC_Measurement_Chr.setProperties(CHR_PROPS_NOTIFY);  
  server_CSC_Measurement_Chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS); 
  server_CSC_Measurement_Chr.setMaxLen(MAX_PAYLOAD);
  server_CSC_Measurement_Chr.setCccdWriteCallback(server_cccd_callback);  
  server_CSC_Measurement_Chr.begin();

  // Set the CSC Feature characteristic
  server_CSC_Feature_Chr.setProperties(CHR_PROPS_READ);
  server_CSC_Feature_Chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  server_CSC_Feature_Chr.setFixedLen(CSC_FEATURE_FIXED_DATALEN);
  server_CSC_Feature_Chr.begin();
  server_CSC_Feature_Chr.write16(client_CSC_Feature_Flags);
  
  // Configure the Sensor Location characteristic
  server_CSC_Location_Chr.setProperties(CHR_PROPS_READ);
  server_CSC_Location_Chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  server_CSC_Location_Chr.setFixedLen(1);
  server_CSC_Location_Chr.begin();
  server_CSC_Location_Chr.write8(client_CSC_Location_Value);  // Set the characteristic
}

void server_CP_ControlPoint_Chr_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  // Server CP Control Point data from Zwift is transferred to the client (trainer)
  // NO TREATMENT OF COMMAND !!!
  // Control Point write is documented to be write_resp !!!
  client_CP_ControlPoint_Chr.write_resp(data, len); // Just pass on and process later!

#ifdef DEBUG
  uint8_t cpcpDataLen = (uint8_t)len;    // Get the actual length of data bytes and type cast to (uint8_t)
  uint8_t cpcpData[cpcpDataLen];
  memset(cpcpData, 0, cpcpDataLen); // set to zero
  // Display the raw request packet actual length
  DEBUG_PRINTF(" -> Server CP Control Point Data [%d] [ ", cpcpDataLen);
  // Transfer the contents of data to cpcpData
  for (int i = 0; i < cpcpDataLen; i++) {
    if ( i <= sizeof(cpcpData)) {
      cpcpData[i] = *data++;
      // Display the raw request packet byte by byte in HEX
      DEBUG_PRINTF("%02X ", cpcpData[i], HEX);
    }
  }
  DEBUG_PRINTLN(" ]  ");
#endif 
}

void server_connect_callback(uint16_t conn_handle)
{
  char Peer_Name[MAX_PAYLOAD] = {0};
  uint8_t Peer_Addr[6] = {0};
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);
  connection->getPeerName(Peer_Name, sizeof(Peer_Name));
  ble_gap_addr_t peer_address = connection->getPeerAddr();
  memcpy(Peer_Addr, peer_address.addr, 6);
#ifdef DEBUG
  DEBUG_PRINTF("Feather nRF52 (Client) connected to Central device: [%s] MAC Address: ",  Peer_Name);
  PrintPeerAddress(Peer_Addr);
  DEBUG_PRINTLN();
#endif
  // Who has been exactly connected?
  // [1] Laptop is connecting
  if (memcmp(Peer_Addr, Laptop.PeerAddress, 6) == 0 ) { // Check Laptop MAC address
    // Laptop/PC is connecting !
    memcpy(Laptop.PeerName, Peer_Name, sizeof(Peer_Name));
    Laptop.conn_handle = conn_handle;
    Laptop.IsConnected = true;
    Peer_Name[9] = 0; // Cut off name at 9
    ShowOnOledLarge("Pairing", (const char*)Peer_Name, "Done!", 500);
    DEBUG_PRINTLN("Waiting for Central (Zwift) to set CCCD Notify/Indicate (enable) and start....");
    return; // We are done here!
  }
  DEBUG_PRINTLN("No match with Laptop Mac Address, so Client is connected to a Smartphone!");
  // [2] Smartphone is connecting
  memcpy(Smartphone.PeerName, Peer_Name, sizeof(Peer_Name));
  Smartphone.conn_handle = conn_handle;
  Smartphone.IsConnected = true;
  memcpy(Smartphone.PeerAddress, Peer_Addr, 6);
  Peer_Name[9] = 0; // Cut off name at 9
  ShowOnOledLarge("Pairing", (const char*)Peer_Name, "Done!", 500);
  DEBUG_PRINTLN("Waiting for Central (Phone) to set NUS Txd 'Notify' and start....");
} // end connect_callback

/*
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE
*/
void server_disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  char disc_reason[10] = {0};
  sprintf(disc_reason, "Reason:%2X", reason);
  if(Laptop.conn_handle == conn_handle ) { // Laptop/Desktop is disconnected
     Laptop.conn_handle = BLE_CONN_HANDLE_INVALID;
     Laptop.IsConnected = false;
     DEBUG_PRINTF("Server disconnected from Central (Laptop): [%s], reason: [%02X]\n", Laptop.PeerName, reason, HEX);
     Laptop.PeerName[9] = 0; // Cut off name at 9
     ShowOnOledLarge("Lost!", (const char*)Laptop.PeerName, (const char*)disc_reason, 500);
  }
  if(Smartphone.conn_handle == conn_handle ) { // Smartphone is disconnected
     Smartphone.conn_handle = BLE_CONN_HANDLE_INVALID;
     Smartphone.IsConnected = false;
     DEBUG_PRINTF("Server disconnected from Central (Phone): [%s], reason: [%02X]\n", Smartphone.PeerName, reason, HEX);
     Smartphone.PeerName[9] = 0; // Cut off name at 9
     ShowOnOledLarge("Lost!", (const char*)Smartphone.PeerName, (const char*)disc_reason, 500);
  }
  // Default RestartOnDisconnect == true 
  DEBUG_PRINTLN("Server is advertising again!");
}

void server_cccd_callback(uint16_t conn_handle, BLECharacteristic* chr, uint16_t cccd_value)
{
    // When changed, display the Notify/Indicate Status for all characteristics
    DEBUG_PRINTF("Central Updated CCCD to: [%d] --> ", cccd_value);
    // Check the characteristic UUID this CCCD callback is associated with,
    // in case this handler is used for multiple CCCD records.
    if (chr->uuid == server_HR_Measurement_Chr.uuid) { 
        if (chr->notifyEnabled(conn_handle)) { 
          DEBUG_PRINT("Server HR: Measurement 'Notify' enabled");
          client_HR_Measurement_Chr.enableNotify();
        } else {
          DEBUG_PRINT("Server HR: Measurement 'Notify' disabled");
          client_HR_Measurement_Chr.disableNotify();
        }
    }
    if (chr->uuid == server_CP_Measurement_Chr.uuid) { 
        if (chr->notifyEnabled(conn_handle)) { 
          DEBUG_PRINT("Server CP: Measurement 'Notify' enabled");
          client_CP_Measurement_Chr.enableNotify();
        } else {
          DEBUG_PRINT("Server CP: Measurement 'Notify' disabled");
          client_CP_Measurement_Chr.disableNotify();
        }
    }
    if (chr->uuid == server_CP_ControlPoint_Chr.uuid) { 
        if (chr->indicateEnabled(conn_handle)) { 
            DEBUG_PRINT("Server CP: ControlPoint 'Indicate' enabled");
            client_CP_ControlPoint_Chr.enableIndicate();
        } else {
            DEBUG_PRINT("Server CP: ControlPoint 'Indicate' disabled");
            client_CP_ControlPoint_Chr.disableIndicate();
        }
    }
    if (chr->uuid == server_CSC_Measurement_Chr.uuid) { 
        if (chr->notifyEnabled(conn_handle)) { 
          DEBUG_PRINT("Server CSC: Measurement 'Notify' enabled");
          client_CSC_Measurement_Chr.enableNotify();
        } else {
          DEBUG_PRINT("Server CSC: Measurement 'Notify' disabled");
          client_CSC_Measurement_Chr.disableNotify();
        }
    }
    if (chr->uuid == server_FTM_ControlPoint_Chr.uuid) { 
        if (chr->indicateEnabled(conn_handle)) { 
          DEBUG_PRINT("Server FTM: ControlPoint 'Indicate' enabled");
          client_FTM_ControlPoint_Chr.enableIndicate();
        } else {
          DEBUG_PRINT("Server FTM: ControlPoint 'Indicate' disabled");
          client_FTM_ControlPoint_Chr.disableIndicate();
        }
    }
    if (chr->uuid == server_FTM_IndoorBikeData_Chr.uuid) { 
        if (chr->notifyEnabled(conn_handle)) { 
          DEBUG_PRINT("Server FTM: IndoorBikeData 'Notify' enabled");
          client_FTM_IndoorBikeData_Chr.enableNotify();
        } else {
          DEBUG_PRINT("Server FTM: IndoorBikeData 'Notify' disabled");
          client_FTM_IndoorBikeData_Chr.disableNotify();
        }
    }

    if (chr->uuid == server_FTM_TrainingStatus_Chr.uuid) { 
        if (chr->notifyEnabled(conn_handle)) { 
          DEBUG_PRINT("Server FTM: TrainingStatus 'Notify' enabled");
          client_FTM_TrainingStatus_Chr.enableNotify();
        } else {
          DEBUG_PRINT("Server FTM: TrainingStatus 'Notify' disabled");
          client_FTM_TrainingStatus_Chr.disableNotify();
        }
    }
    if (chr->uuid == server_FTM_Status_Chr.uuid) { 
        if (chr->notifyEnabled(conn_handle)) { 
          DEBUG_PRINT("Server FTM: FitnessMachineStatus 'Notify' enabled");
          client_FTM_Status_Chr.enableNotify();
        } else {
          DEBUG_PRINT("Server FTM: FitnessMachineStatus 'Notify' disabled");
          client_FTM_Status_Chr.disableNotify();
        }
    }
    if (chr->uuid == server_NUS_TXD_Chr.uuid) { 
        if (chr->notifyEnabled(conn_handle)) { 
          DEBUG_PRINT("Server NUS: TXD 'Notify' enabled");
        } else {
          DEBUG_PRINT("Server NUS: TXD 'Notify' disabled");
        }
    }
    DEBUG_PRINTLN();
} // end Server CCCD callback

void Server_Sends_NUS_TXD_Persistent_Settings(void)
{
    // Send persistent stored values to Mobile Phone for correct Settings!
    // recalculate the values for use on the Phone
    char TXpacketBuffer[16] = { 0 };
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
    sprintf(TXpacketBuffer, "!S%d;%d;%d;%d;", iMax, iMin, iPerc, iDispl);
    // send these persistent data to the Settings page on the smartphone
    server_NUS_TXD_Chr.notify(TXpacketBuffer, sizeof(TXpacketBuffer));
    DEBUG_PRINTF("Server Sends NUS TXD Persistent settings to Phone: [%s]", TXpacketBuffer);
}

// ----------------------   END of SERVER SIDE FUNCTIONS  ------------------------- 
