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
 *  This Adafruit ESP32 Huzzah V2 tested code advertises and enables the relevant 
 *  Cycling Trainer Services: CPS, CSC and FTMS.
 *  It allows to connect to Cycling apps like Zwift (aka Client or Central)!
 *  It simulates a connected Cycling Trainer and in BLE terms it is a Server or 
 *  or in BlueFruit BLE library terms it is a Peripheral!
 *  Requirements: Zwift app or alike and an ESP32 board
 *  1) Upload and Run this code on the ESP32
 *  2) Start the Serial Monitor to catch debugging info
 *  3) Start Zwift and wait for the Devices Pairing Screen
 *  4) Unpair all previously paired devices
 *  5) Search on Zwift pairing screens for the ESP32 board: "Sim ESP32"
 *  6) Pair all four "simulated" devices: Power, Cadence, Heart Rate and Controllable
 *  7) Start a default Zwift ride or any ride you wish
 *     No need for you to do work on the trainer!
 *  8) Make Serial Monitor visible on top of the Zwift window 
 *  9) Inspect the info presented by Serial Monitor
 * 10) Notice how your avatar is riding the route all by itself...
 *  
 */
 
// -------------------------------------------------------------------------------------------
// COMPILER DIRECTIVE to allow/suppress DEBUG messages that help debugging...
// Uncomment general "#define DEBUG" to activate
#define DEBUG
// Include these debug utility macros in all cases!
#include "DebugUtils.h"
// --------------------------------------------------------------------------------------------
#ifdef DEBUG
#endif

#define 	BLE_APPEARANCE_GENERIC_CYCLING   1152
#define 	BLE_APPEARANCE_CYCLING_CYCLING_COMPUTER   1153
#define 	BLE_APPEARANCE_CYCLING_SPEED_SENSOR   1154
#define 	BLE_APPEARANCE_CYCLING_CADENCE_SENSOR   1155
#define 	BLE_APPEARANCE_CYCLING_POWER_SENSOR   1156
#define 	BLE_APPEARANCE_CYCLING_SPEED_CADENCE_SENSOR   1157

#include <NimBLEDevice.h>
#include <nimble/nimble/host/services/gap/include/services/gap/ble_svc_gap.h>

// Defined globally
const uint8_t MAX_PAYLOAD = 20; // Max 20 byte data size for single packet BLE transfer
BLEServer* pServer;
NimBLEAdvertising* pAdvertising;

uint8_t serverPeerAddress[6];
bool serverIsConnected = false;
uint16_t serverConnectionHandle = BLE_HS_CONN_HANDLE_NONE;

// ------------------------------------------------------------------------------------
// Client Generic Access Service-------------------------------------------------------
// -----------------------------------------------------------------------------------
// Server Generic Access variables
uint16_t client_GA_Appearance_Value = BLE_APPEARANCE_GENERIC_CYCLING;  // decimal: 1152 -> Generic Cycling
// Set the advertised device name
std::string client_GA_DeviceName_Str = "ESP32";
// ------------------------------------------------------------------------------------

// ------------------------------------------------------------------------------------
// Client Service Device Information 
// ------------------------------------------------------------------------------------
#define UUID16_SVC_DEVICE_INFORMATION                         BLEUUID((uint16_t)0x180A)
#define UUID16_CHR_MODEL_NUMBER_STRING                        BLEUUID((uint16_t)0x2A24)
#define UUID16_CHR_SERIAL_NUMBER_STRING                       BLEUUID((uint16_t)0x2A25)
#define UUID16_CHR_FIRMWARE_REVISION_STRING                   BLEUUID((uint16_t)0x2A26)
#define UUID16_CHR_HARDWARE_REVISION_STRING                   BLEUUID((uint16_t)0x2A27)
#define UUID16_CHR_SOFTWARE_REVISION_STRING                   BLEUUID((uint16_t)0x2A28)
#define UUID16_CHR_MANUFACTURER_NAME_STRING                   BLEUUID((uint16_t)0x2A29)
BLEService *server_DeviceInformation_Service;
BLECharacteristic *server_DIS_ModelNumber_Chr;       // Read
std::string client_DIS_ModelNumber_Str = "ESP32 Feather V2";
BLECharacteristic *server_DIS_SerialNumber_Chr;      // Read
std::string client_DIS_SerialNumber_Str = "12345";
BLECharacteristic *server_DIS_Firmware_Chr;          // Read
std::string client_DIS_Firmware_Str = "12345";
BLECharacteristic *server_DIS_Hardware_Chr;          // Read
std::string client_DIS_Hardware_Str = "12345";
BLECharacteristic *server_DIS_Software_Chr;          // Read
std::string client_DIS_Software_Str = "12345";
BLECharacteristic *server_DIS_ManufacturerName_Chr;  // Read
std::string client_DIS_Manufacturer_Str = "Adafruit Industries";
// -------------------------------------------------------------------------------------

/* NORDIC UART SERVICE a.k.a. NUS
 * NUS Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 * NUS RXD    : 6E400002-B5A3-F393-E0A9-E50E24DCCA9E
 * NUS TXD    : 6E400003-B5A3-F393-E0A9-E50E24DCCA9E
 */
static BLEUUID UUID_NUS_SERVICE("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID UUID_NUS_CHR_RXD("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID UUID_NUS_CHR_TXD("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");
BLEService *server_NordicUart_Service; 
BLECharacteristic *server_NUS_Rxd_Chr;        // Write No Response (Receiving Data)
BLECharacteristic *server_NUS_Txd_Chr;        // Read Notify (Sending Data)

/* Cycling Power Service --------------------------------------------------------------------------------------------------------
 * CP Service: 0x1818  
 * CP Characteristic: 0x2A63 (Measurement)
 * CP Characteristic: 0x2A65 (Feature)
 * CP Characteristic: 0x2A5D (Location)
 * CP Characteristic: 0x2A66 (Control Point) -> Not implemented
 */
#define UUID16_SVC_CYCLING_POWER                              BLEUUID((uint16_t)0x1818)
#define UUID16_CHR_CYCLING_POWER_MEASUREMENT                  BLEUUID((uint16_t)0x2A63)
//#define UUID16_CHR_CYCLING_POWER_VECTOR                       BLEUUID((uint16_t)0x2A64)
#define UUID16_CHR_CYCLING_POWER_FEATURE                      BLEUUID((uint16_t)0x2A65)
//#define UUID16_CHR_CYCLING_POWER_CONTROL_POINT                BLEUUID((uint16_t)0x2A66)
#define UUID16_CHR_SENSOR_LOCATION                            BLEUUID((uint16_t)0x2A5D) // shared with CSC

BLEService        *server_CyclingPower_Service;
BLECharacteristic *server_CP_Measurement_Chr; //                          Notify, Read
// Cycle power measurement config flags field variable
uint16_t server_CP_Measurement_Flags = 0; 
// CP Measurement data array for sending to Central device
const uint8_t CP_MEASUREMENT_DATALEN = 4;  // With present flags FixedLen = 4 
uint8_t server_CP_Measurement_Data[CP_MEASUREMENT_DATALEN];
BLECharacteristic *server_CP_Feature_Chr; //                              Read
// 32 Bit flag to set present CP Features  -> CP simulation variable
uint32_t client_CP_Feature_Flags = {0b00000000000000010000011010001011};  // Relevant Cycling Power features: set to TRUE!!!
BLECharacteristic *server_CP_Location_Chr; //                             Read
// 8 bit location of the sensor -> CP simulation variable
byte client_CP_Location_Value[1] = {0x0C};                    //          --> rear wheel !
//BLECharacteristic *server_CP_ControlPoint_Chr; //                       Indicate, Write

/* ------------------------------------------------------------------------------------------------------------------------------
 * Cycling Speed and Cadence Service
 * CSC Service:   0x1816 
 * CSC Measurement Characteristic:  0x2A5B
 * CSC Feature Characteristic:      0x2A5C  
 * CSC Control Point Characteristic:0x2A55 <not implemented> 
*/
#define UUID16_SVC_CYCLING_SPEED_AND_CADENCE                  BLEUUID((uint16_t)0x1816)
//#define UUID16_CHR_CSC_CONTROL_POINT                          BLEUUID((uint16_t)0x2A55)
#define UUID16_CHR_CSC_MEASUREMENT                            BLEUUID((uint16_t)0x2A5B)
#define UUID16_CHR_CSC_FEATURE                                BLEUUID((uint16_t)0x2A5C)
//#define UUID16_CHR_SENSOR_LOCATION                            BLEUUID((uint16_t)0x2A5D) // shared with CP

/*       CSC Measurement flags */
#define     CSC_MEASUREMENT_WHEEL_REV_PRESENT       0x01
#define     CSC_MEASUREMENT_CRANK_REV_PRESENT       0x02
/*       CSC feature flags */
#define     CSC_FEATURE_WHEEL_REV_DATA              0x01
#define     CSC_FEATURE_CRANK_REV_DATA              0x02
#define     CSC_FEATURE_MULTIPLE_SENSOR_LOC         0x04 

BLEService        *server_CyclingSpeedCadence_Service;  
BLECharacteristic *server_CSC_Measurement_Chr;               //     Notify, Read
// 8 Bit flag to set: which type of data present in CSC measurement packet -> CSC simulation variables
const uint8_t server_CSC_Measurement_Flags = (uint8_t)(CSC_MEASUREMENT_WHEEL_REV_PRESENT | CSC_MEASUREMENT_CRANK_REV_PRESENT);
const uint8_t CSC_MEASUREMENT_DATALEN = 11; //
uint8_t server_CSC_Measurement_Data[CSC_MEASUREMENT_DATALEN] = {}; 
BLECharacteristic *server_CSC_Feature_Chr;                   //     Read
// 16 Bit flag to set present CSC Features -> CSC simulation variable
uint16_t client_CSC_Feature_Flags = (CSC_FEATURE_WHEEL_REV_DATA | CSC_FEATURE_CRANK_REV_DATA | CSC_FEATURE_MULTIPLE_SENSOR_LOC);
BLECharacteristic *server_CSC_Location_Chr;                  //     Read
// 8 bit location of the sensor -> CSC simulation variable
byte client_CSC_Location_Value[1] = {0x0C};                    // --> rear wheel !

// ------------------------------------------------------------------
// Heart Rate Service defined globally!
// ------------------------------------------------------------------
/* HRM Service Definitions
 * Heart Rate Monitor Service:  0x180D
 * Heart Rate Measurement Char: 0x2A37 (Mandatory)
 * Body Sensor Location Char:   0x2A38 (Optional)
 */
#define UUID16_SVC_HEART_RATE             BLEUUID((uint16_t)0x180D)
#define UUID16_CHR_HEART_RATE_MEASUREMENT BLEUUID((uint16_t)0x2A37)
#define UUID16_CHR_BODY_SENSOR_LOCATION   BLEUUID((uint16_t)0x2A38)
BLEService *server_HeartRate_Service;
BLECharacteristic *server_HR_Measurement_Chr;  // Notify Write
#define HR_MEASUREMENT_DATALEN 4
uint8_t server_HR_Measurement_Data[HR_MEASUREMENT_DATALEN] = {0};
BLECharacteristic *server_HR_Location_Chr;     // Read
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
byte client_HR_Location_Chr_Value[1] = { 0x01 }; // Chest
// ---------------------------------------------------------------------

/* ---------------------------------------------------------------------------------------------------------------
 * Fitness Machine Service 
 * ---------------------------------------------------------------------------------------------------------------*/
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

BLEService        *server_FitnessMachine_Service; // FTM Service
BLECharacteristic *server_FTM_Feature_Chr; //  Fitness Machine Feature, mandatory, read
BLECharacteristic *server_FTM_Status_Chr; //  Fitness Machine Status, mandatory, notify
BLECharacteristic *server_FTM_IndoorBikeData_Chr;  //  Indoor Bike Data, optional, notify
BLECharacteristic *server_FTM_ControlPoint_Chr; //  Fitness Machine Control Point, optional, write & indicate
BLECharacteristic *server_FTM_TrainingStatus_Chr; //  Training Status, optional, read & notify
BLECharacteristic *server_FTM_SupportedResistanceLevelRange_Chr; // Supported Resistance Level, read, optional
BLECharacteristic *server_FTM_SupportedPowerRange_Chr; // Supported Power Levels, read, optional

const uint8_t AverageSpeedSupported = 1;      // byte 1
const uint8_t CadenceSupported = 2;           // byte 1
const uint8_t ResistanceLevelSupported = 128; // byte 1
const uint8_t HeartRateMeasurementSupported = 4; // byte 2
const uint8_t PowerMeasurementSupported = 64;    // byte 2
const uint8_t IndoorBikeSimulationParametersSupported = 32; // byte 6

const uint8_t FTM_FEATURE_FIXED_DATALEN = 8; // Fixed value!
//  Little Endian        LSO                                MSO
//  Feature = {0b10000011,0b01000000,0x00,0x00,0x00,0b00100000,0x00,0x00}; // Binary and Hex
//  Feature = {0x80,0x40,0x00,0x00,0x00,0x20,0x00,0x00}; // Hex
//  Feature = {0x86,0x50,0x00,0x00,0x0C,0xE0,0x00,0x00}; // Direto XR HEX
//  Feature = {0b10000110,0b01010000,0x00,0x00,0b00001100,0b11100000,0x00,0x00}; // Binary and Hex see indicated stars above
unsigned char client_FTM_Feature_Data[FTM_FEATURE_FIXED_DATALEN] = \
                                           {(unsigned char)(AverageSpeedSupported | CadenceSupported | ResistanceLevelSupported),\
                                            (unsigned char)(PowerMeasurementSupported |HeartRateMeasurementSupported),0x00,0x00,0x00,\
                                            (unsigned char)(IndoorBikeSimulationParametersSupported),0x00,0x00};
// Indoor Bike Data characteristic [server_FTM_IndoorBike_Data]
// ---> Length depends on data flagged to be present !!!!
// Fill first byte (LSO) with flags, others (MSO) are zero
const uint16_t flagMoreData = 1;
const uint16_t flagAverageSpeed = 2;
const uint16_t flagInstantaneousCadence = 4;
const uint16_t flagAverageCadence = 8;
const uint16_t flagTotalDistance = 16;
const uint16_t flagResistanceLevel = 32;
const uint16_t flagIntantaneousPower = 64;
const uint16_t flagAveragePower = 128;
// Second byte:
const uint16_t flagExpendedEnergy = 1;  // 256;
const uint16_t flagHeartRate = 2;       //512;
const uint16_t flagMetabolicEquivalent = 1024;
const uint16_t flagElapsedTime = 2048;
const uint16_t flagRemainingTime = 4096;

const uint8_t FTM_INDOORBIKE_DATALEN = 9; // NO Fixed Len --> setMaxLen when all flags are set (!) equals 28, so a MAXLEN = 20 will do !!
// Notice a "fixed len" of 9 is perfect with the following flags set ! Other flags is other "size"
unsigned char server_FTM_IndoorBike_Data[FTM_INDOORBIKE_DATALEN] = {(uint8_t)(flagMoreData | flagAverageSpeed | flagInstantaneousCadence | \
                                                                      flagIntantaneousPower),(uint8_t)flagHeartRate,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 

// Training status: flags: 0 (no string present); Status: 0x00 = Other
const uint8_t FTM_TRAINING_STATUS_FIXED_DATALEN = 2; // Fixed len
unsigned char server_FTM_TrainingStatus_Data[FTM_TRAINING_STATUS_FIXED_DATALEN] = {0x00, 0x01}; // 0x01 => idle
                                                              
// Supported Resistance Level Range
// Minimum Resistance Level: 0.0 Maximum Resistance Level: 100.0 Minimum Increment: 0.1
const uint8_t FTM_SRLR_FIXED_DATALEN = 6; // Fixed Len
unsigned char client_FTM_SupportedResistanceLevelRange_Data[FTM_SRLR_FIXED_DATALEN] = {0x00, 0x00, 0xE8, 0x03, 0x01, 0x00}; 

// Supported Power Range
const uint8_t FTM_SPR_FIXED_DATALEN = 6; // Fixed Len
// Minimum Power 0W Max Power 4000W Min Increment 1W
unsigned char client_FTM_SupportedPowerRange_Data[FTM_SPR_FIXED_DATALEN] = {0x00, 0x00, 0xA0, 0x0F, 0x01, 0x00};
/* Minimum Power: 0 W Maximum Power: 800 W Minimum Increment: 1 W
unsigned char client_FTM_SupportedPowerRange_Data[FTM_SPR_FIXED_DATALEN] = {0x00, 0x00, 0x20, 0x03, 0x01, 0x00};
*/

// Machine Status
// The Fitness Machine Status OpCode and the Parameter format are as follows:
// OpCode: 0x00 RFU (Reserved for Future Use)
// OpCode: 0x01 Reset
// OpCode: 0x02 Stopped [0x01] or Paused [0x02] by the user
// OpCode: 0x03 Stopped by safety key
// OpCode: 0x04 Fitness Machine Started or Resumed by the User
const uint8_t FTM_STATUS_DATALEN = 3; // Max Len
unsigned char server_FTM_Status_Data[FTM_STATUS_DATALEN] = {0x01, 0x00, 0x00};  // Sofar we only use first 2 bytes !

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

// Control Point: Common Response Buffers
const uint8_t ftmcpResponseCode = 0x80;
const uint8_t ftmcpRespConfirm = 0x01; // Ok!
const uint8_t ftmcpRespUnknown = 0x02; // Unknown OpCode
unsigned char ftmcpRespConfirmBuffer[3] = {ftmcpResponseCode, 0x00, ftmcpRespConfirm};
unsigned char ftmcpRespUnknownBuffer[3] = {ftmcpResponseCode, 0x00, ftmcpRespUnknown};

// -----------------------------------------------------------------------
// Global variables for decoding of INDOOR BIKE DATA RESISTANCE PARAMETERS
// -----------------------------------------------------------------------
float wind_speed = 0;       // meters per second, resolution 0.001
float grade = 0;            // percentage, resolution 0.01
float crr = 0;              // Coefficient of rolling resistance, resolution 0.0001
float cw = 0;               // Wind resistance Kg/m, resolution 0.01;
// -------------------------------------------------
// global cycling definitions for test purposes ONLY!
// -------------------------------------------------
const uint16_t  WHEEL_CIRCUMFERENCE_MM = 2105;
const uint16_t  MIN_SPEED_KPH =           25;
const uint16_t  MAX_SPEED_KPH =           35;
const uint16_t  MIN_CRANK_RPM =           85;
const uint16_t  MAX_CRANK_RPM =           95;
const uint16_t  MAX_POWER_WATT =         200;
const uint16_t  MIN_POWER_WATT =         100;
const uint8_t   MIN_HEART_RATE =         100;
const uint8_t   MAX_HEART_RATE =         150;
// ----------------------------------------------
// global CSC, CPS, HBM Variables for test purposes ONLY!
// ----------------------------------------------
uint32_t cum_wheel_rev = 0;
uint16_t last_wheel_event = 0;
uint16_t cum_cranks = 0;
uint16_t last_crank_event = 0;
uint16_t csc_sim_speed_kph = MIN_SPEED_KPH ;  // Speed in Km/hour 
uint16_t csc_sim_crank_rpm = MIN_CRANK_RPM ;  // RPM
uint16_t cps_sim_power_watt = MIN_POWER_WATT; // Power in Watts for simulation
uint8_t  sim_heart_beat_min = (MIN_HEART_RATE + 1); // Start value
// Specific update timing variables per characteristic
#define IBD_TIME_SPAN         200 // Time span (delay) for sending Indoor Bike Data
#define CP_CSC_TIME_SPAN      100 // Time span (delay) for sending CP and CSC data
#define HRM_TIME_SPAN         900 // Time span (delay) for sending HRM data
#define SIM_TIME_SPAN         750 // Time span (delay) for updating simulated CP and CSC values
unsigned long IBDtimeInterval = 0;
unsigned long CPCSCtimeInterval = 0;
unsigned long HRMtimeInterval = 0;
unsigned long SIMtimeInterval = 0;
// ---------------------------------------------------------------------------------
// Server Connect and Disconnect callbacks defined
class server_Connection_Callbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer, ble_gap_conn_desc *desc);
  void onDisconnect(BLEServer* pServer, ble_gap_conn_desc *desc);
  void onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc);
};

/** Handler class for characteristic actions */
class CharacteristicCallbacks: public NimBLECharacteristicCallbacks {
/*
    void onRead(NimBLECharacteristic* pCharacteristic){};
    void onWrite(NimBLECharacteristic* pCharacteristic){};
    void onNotify(NimBLECharacteristic* pCharacteristic){};
    void onStatus(NimBLECharacteristic* pCharacteristic, Status status, int code){};
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
    };
};

/** Define callback instances globally to use for multiple Charateristics */
static CharacteristicCallbacks server_Multi_Chr_Callbacks;

// ---------------------------------------------------------------------------------
void ConvertMacAddress(char *fullAddress, uint8_t addr[6], bool LittleEndian)
{ // Display byte by byte in HEX 
  if(LittleEndian) { // Unaltered: in Little Endian representation
    sprintf(fullAddress, "%02X:%02X:%02X:%02X:%02X:%02X", addr[0], addr[1], addr[2], \
      addr[3], addr[4], addr[5], HEX);   
  } else { // Altered: in reversed order
    sprintf(fullAddress, "%02X:%02X:%02X:%02X:%02X:%02X", addr[5], addr[4], addr[3], \
      addr[2], addr[1], addr[0], HEX);       
  }
}
void server_setupGA(void);
void server_setupDIS(void);
void server_setupHRM(void);
void server_setupNUS(void);
void server_setupFTMS(void);
void server_setupCSC(void);
void server_setupCPS(void);
void server_startADV(void);
// ---------------------------------------------------------------------------------
void setup()
{
#ifdef DEBUG  
    Serial.begin(115200);       
    while ( !Serial ) 
      delay(10); 
#endif
//----------------------------------------------------  
    DEBUG_PRINTLN("  ESP32 BLE Server/Peripheral: CPS, CSC, HBM and FTMS");
    DEBUG_PRINTLN("------------------- Version 01.0 --------------------");
    BLEDevice::init("ESP32"); // shortname
    pServer = BLEDevice::createServer();
    // -------------------------
#ifdef DEBUG
    BLEAddress MyAddress = BLEDevice::getAddress();
    //DEBUG_PRINTF("BLEDevice address in Little Endian order: [%s]\n", BLEDevice::getAddress().toString().c_str());
    uint8_t ESP32Address[6] = {};
    memcpy(&ESP32Address, MyAddress.getNative(), 6);
    char fullMacAddress[18] = {}; //
    ConvertMacAddress(fullMacAddress, ESP32Address, false); // true -> little endian representation!
    DEBUG_PRINTF("New ESP32 Server/Peripheral created with local Mac Address: [%s]\n", fullMacAddress);
#endif
    // -------------------------
    //Setup callbacks onConnect and onDisconnect
    pServer->setCallbacks(new server_Connection_Callbacks());
// ------------------------------------------------------- 

  DEBUG_PRINTLN("Configuring the default Generic Access Service");
  server_setupGA();
  DEBUG_PRINTLN("Configuring the Server Nordic Uart Service");  
  server_setupNUS();
  DEBUG_PRINTLN("Configuring the Server Device Information Service");
  server_setupDIS();
  // Setup the Cycle Power Service, Speed & Cadence Service and FTMS
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
  // Ready!
  IBDtimeInterval = millis() + IBD_TIME_SPAN;   // Set value first time
  CPCSCtimeInterval = millis() + CP_CSC_TIME_SPAN; // Set value first time
  HRMtimeInterval = millis() + HRM_TIME_SPAN; // idem
  SIMtimeInterval = millis() + SIM_TIME_SPAN; // idem  
  DEBUG_PRINTLN("Server is advertising: CPS, CSC and FTMS"); 
}

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
    pAdvertising->setMinInterval(32); // in 0.625ms units, 0 = use default.
    pAdvertising->setMaxInterval(244); // in 0.625ms units, 0 = use default.
    NimBLEDevice::startAdvertising();    
    // Start Advertising 
}

void server_Connection_Callbacks::onConnect(BLEServer* pServer, ble_gap_conn_desc *desc) {
    // Get some connection parameters of the peer device.
    serverConnectionHandle = desc->conn_handle;
    uint16_t serverConnectionInterval = desc->conn_itvl;   // Connection interval   
    uint16_t serverConnectionLatency = desc->conn_latency; // Connection latency
    uint16_t serverConnectionSupTimeout = desc->supervision_timeout;   // Connection supervision timeout     
    memcpy(&serverPeerAddress, NimBLEAddress(desc->peer_id_addr).getNative(), 6);
#ifdef DEBUG
    char fullMacAddress[18] = {}; //
    ConvertMacAddress(fullMacAddress, serverPeerAddress, false); // true -> little endian representation!
    DEBUG_PRINTF("Server Connection Parameters -> Interval: [%d] Latency: [%d] Supervision Timeout: [%d]\n",serverConnectionInterval, \
                                                                  serverConnectionLatency, serverConnectionSupTimeout); 
    DEBUG_PRINTF("ESP32 Server connected to Client device with MAC Address: [%s] Handle: [%d]\n", fullMacAddress, serverConnectionHandle); 
    DEBUG_PRINTLN("Waiting for Client Device to set 'Notify/Indicate' enabled for relevant Characteristics...");
#endif
    serverIsConnected = true;
    BLEDevice::stopAdvertising();
    /** We can use the connection handle here to ask for different connection parameters.
     *  Args: connection handle, min connection interval, max connection interval, latency and supervision timeout.
     *  Units; Min/Max Intervals: 1.25 millisecond increments --> between: 24 (30 ms) resp. 48 (60 ms)
     *  Latency: number of intervals allowed to skip. --> keep zero
     *  Timeout: 10 millisecond increments. --> Try 400 (4000 ms)
     */ 
    //pServer->updateConnParams(serverConnectionHandle, 24, 48, 0, 400);
    //DEBUG_PRINTLN("Server Updates Connection Parameters -> Min Interval: [24] Max Interval: [48] Latency: [0] Supervision Timeout: [400]");  
 
};

void server_Connection_Callbacks::onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc) {
        DEBUG_PRINTF("MTU updated: %u for connection ID: %u\n", MTU, desc->conn_handle);           
};

void server_Connection_Callbacks::onDisconnect(BLEServer* pServer, ble_gap_conn_desc *desc) {
    serverIsConnected = false; // to disable loop() a.s.a.p.
    // Get some Disconnection parameters of the peer device.
    uint16_t ConnectionHandle = desc->conn_handle;
    uint8_t RemoteAddress[6] = {};
    memcpy(&RemoteAddress, NimBLEAddress(desc->peer_id_addr).getNative(), 6);
    char fullMacAddress[18] = {}; //
    ConvertMacAddress(fullMacAddress, RemoteAddress, false); // true -> little endian representation!
    DEBUG_PRINTF("ESP32 Server disconnected from Client device with MAC Address: [%s] Handle: [%d]\n", fullMacAddress, ConnectionHandle);
    DEBUG_PRINTLN(" --> Server is advertising again!");
    serverConnectionHandle = BLE_HS_CONN_HANDLE_NONE;  
    // NimBLE does auto start after disconnect 
    //BLEDevice::startAdvertising();
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
    server_CP_Location_Chr->setValue(client_CP_Location_Value, 1);
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
    server_CSC_Location_Chr->setValue(client_CSC_Location_Value, 1);
    server_CyclingSpeedCadence_Service->start();    
}

class server_FTM_ControlPoint_Chr_callback: public BLECharacteristicCallbacks {
void onWrite(BLECharacteristic *pCharacteristic) {
  std::string ftmcpData = server_FTM_ControlPoint_Chr->getValue();
  uint8_t ftmcpDataLen = ftmcpData.length();
 
  memset(server_FTM_Control_Point_Data.bytes, 0, sizeof(server_FTM_Control_Point_Data.bytes));
  // Display the raw request packet
  DEBUG_PRINTF(" --> Raw FTM Control Point Data [len: %d] ", ftmcpDataLen);
  // Transfer the contents of data to server_FTM_Control_Point_Data.bytes
  for (int i = 0; i < ftmcpDataLen; i++) {
      server_FTM_Control_Point_Data.bytes[i] = ftmcpData[i];
  }
  /// Decodes an incoming Fitness Machine Control Point request
  DEBUG_PRINTF("[OpCode: %02X] [Values: ", server_FTM_Control_Point_Data.values.OPCODE, HEX);
  for (int i=0; i<ftmcpDataLen; i++) { 
    DEBUG_PRINTF("%02X ", server_FTM_Control_Point_Data.values.OCTETS[i], HEX); 
  }
  DEBUG_PRINTLN("]");
  switch(server_FTM_Control_Point_Data.values.OPCODE) {
    case ftmcpRequestControl: {
      // Always allow control point to confirm: OK!
      ftmcpRespConfirmBuffer[1] = server_FTM_Control_Point_Data.values.OPCODE;
      server_FTM_ControlPoint_Chr->setValue(ftmcpRespConfirmBuffer, 3);
      server_FTM_ControlPoint_Chr->indicate();
      DEBUG_PRINTLN("Request Control of Machine!");
      break;
    }
    case ftmcpStartOrResume: {
       // confirm OK!
      ftmcpRespConfirmBuffer[1] = server_FTM_Control_Point_Data.values.OPCODE;
      server_FTM_ControlPoint_Chr->setValue(ftmcpRespConfirmBuffer, 3);
      server_FTM_ControlPoint_Chr->indicate();
      DEBUG_PRINTLN("Start or Resume Machine!");
      server_FTM_TrainingStatus_Data[0] = 0x00;
      server_FTM_TrainingStatus_Data[1] = 0x04; // High Intensity Interval
      server_FTM_TrainingStatus_Chr->setValue(server_FTM_TrainingStatus_Data, FTM_TRAINING_STATUS_FIXED_DATALEN);      
      server_FTM_TrainingStatus_Chr->notify();
      server_FTM_Status_Data[0] = 0x04;  // Machine status: Started or Resumed
      server_FTM_Status_Data[1] = 0x00;
      server_FTM_Status_Chr->setValue(server_FTM_Status_Data, 1); // Only First Byte
      server_FTM_Status_Chr->notify();
      break;
    }
    case ftmcpStopOrPause: {
       // confirm OK!
      ftmcpRespConfirmBuffer[1] = server_FTM_Control_Point_Data.values.OPCODE;
      server_FTM_ControlPoint_Chr->setValue(ftmcpRespConfirmBuffer, 3);
      server_FTM_ControlPoint_Chr->indicate();
      DEBUG_PRINTLN("Stop or Pause Machine, Parameter: Stop!");
      server_FTM_TrainingStatus_Data[0] = 0x00;  // 0x00 No String present; 0x02 Extended string present; 0x01 Training Status String present
      server_FTM_TrainingStatus_Data[1] = 0x01;  // idle
      server_FTM_TrainingStatus_Chr->setValue(server_FTM_TrainingStatus_Data, FTM_TRAINING_STATUS_FIXED_DATALEN);      
      server_FTM_TrainingStatus_Chr->notify();
      server_FTM_Status_Data[0] = 0x02;  // Machine status: Stopped or Paused
      server_FTM_Status_Data[1] = 0x01;  // Stopped parameter
      server_FTM_Status_Chr->setValue(server_FTM_Status_Data, 2); // Only first 2 bytes
      server_FTM_Status_Chr->notify(); 
      break;
    }
    case ftmcpSetIndoorBikeSimulationParameters: {
      // Confirm OK!
      ftmcpRespConfirmBuffer[1] = server_FTM_Control_Point_Data.values.OPCODE;
      server_FTM_ControlPoint_Chr->setValue(ftmcpRespConfirmBuffer, 3);      
      server_FTM_ControlPoint_Chr->indicate();
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
      // Confirm OK!
      ftmcpRespConfirmBuffer[1] = server_FTM_Control_Point_Data.values.OPCODE;
      server_FTM_ControlPoint_Chr->setValue(ftmcpRespConfirmBuffer, 3);
      server_FTM_ControlPoint_Chr->indicate();
      DEBUG_PRINTLN("Reset Machine!");
      server_FTM_TrainingStatus_Data[0] = 0x00;  // 0x00 No String present; 0x02 Extended string present; 0x01 Training Status String present
      server_FTM_TrainingStatus_Data[1] = 0x01;  // idle
      server_FTM_TrainingStatus_Chr->setValue(server_FTM_TrainingStatus_Data, FTM_TRAINING_STATUS_FIXED_DATALEN);
      server_FTM_TrainingStatus_Chr->notify();
      server_FTM_Status_Data[0] = 0x01;  // Machine Status: Reset
      server_FTM_Status_Data[1] = 0x00;
      server_FTM_Status_Chr->setValue(server_FTM_Status_Data, 1); // Only First Byte
      server_FTM_Status_Chr->notify();
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
      ftmcpRespUnknownBuffer[1] = server_FTM_Control_Point_Data.values.OPCODE;
      server_FTM_ControlPoint_Chr->setValue(ftmcpRespUnknownBuffer, 3);
      server_FTM_ControlPoint_Chr->indicate();
      DEBUG_PRINTLN("Unresolved OpCode!");
      break;
    }
    } // switch
  }; // onWrite
  
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
    server_FTM_Feature_Chr->setValue(client_FTM_Feature_Data, FTM_FEATURE_FIXED_DATALEN);

    server_FTM_Status_Chr = server_FitnessMachine_Service->createCharacteristic(UUID16_CHR_FITNESS_MACHINE_STATUS, 
                                                                            NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    server_FTM_Status_Chr->setCallbacks(&server_Multi_Chr_Callbacks); //NIMBLE                                                                         
    server_FTM_IndoorBikeData_Chr = server_FitnessMachine_Service->createCharacteristic(UUID16_CHR_INDOOR_BIKE_DATA, 
                                                                            NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    server_FTM_IndoorBikeData_Chr->setCallbacks(&server_Multi_Chr_Callbacks); //NIMBLE 
    server_FTM_SupportedResistanceLevelRange_Chr = server_FitnessMachine_Service->createCharacteristic(UUID16_CHR_SUPPORTED_RESISTANCE_LEVEL_RANGE, 
                                                                            NIMBLE_PROPERTY::READ); 
    server_FTM_SupportedResistanceLevelRange_Chr->setValue(client_FTM_SupportedResistanceLevelRange_Data, FTM_SRLR_FIXED_DATALEN);                                                                                
    server_FTM_SupportedPowerRange_Chr = server_FitnessMachine_Service->createCharacteristic(UUID16_CHR_SUPPORTED_POWER_RANGE, 
                                                                            NIMBLE_PROPERTY::READ);
    server_FTM_SupportedPowerRange_Chr->setValue(client_FTM_SupportedPowerRange_Data, FTM_SPR_FIXED_DATALEN); 
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
  }; // onWrite
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
  // Set the Generic Access Appearance value from default: [0] a.k.a. "Unknown" to [1152] --> Generic Cycling
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
//  
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
    server_HR_Location_Chr->setValue(client_HR_Location_Chr_Value, 1);
    server_HeartRate_Service->start();  
}

void Simulate_heart_rate(void)
{
    static bool UpDown = true;
    // Simulate increasing/decreasing HRM in time
    if ((sim_heart_beat_min >= MAX_HEART_RATE) | (sim_heart_beat_min <= MIN_HEART_RATE)) {
      UpDown = !UpDown;
    }
    if (UpDown) {
      sim_heart_beat_min += 1; // Up In steps of 1 beat!
      } else {
      sim_heart_beat_min -= 1; // Down In steps of 1 beat!
    }
}

void Simulate_speed_cadence_power(void)
{
    uint16_t wheel_rev_period;
    uint16_t crank_rev_period;

    // Simulate increasing Power generation
    cps_sim_power_watt += 5; // In steps of 10 watt!
    if (cps_sim_power_watt >= MAX_POWER_WATT) {
         cps_sim_power_watt = MIN_POWER_WATT;
    }

    /* Update simulated crank and wheel rotation speed */
    csc_sim_speed_kph++; // in steps of 1
    if ( csc_sim_speed_kph >= MAX_SPEED_KPH) {
         csc_sim_speed_kph = MIN_SPEED_KPH;
    }
    
    csc_sim_crank_rpm++; // in steps of 1
    if (csc_sim_crank_rpm >= MAX_CRANK_RPM) {
         csc_sim_crank_rpm = MIN_CRANK_RPM;
    }
    
    /* Calculate simulated measurement values */
    if (csc_sim_speed_kph > 0){
        wheel_rev_period = (36*64*WHEEL_CIRCUMFERENCE_MM) / (625*csc_sim_speed_kph);
        cum_wheel_rev++;
        last_wheel_event += wheel_rev_period;
    }
    
    if (csc_sim_crank_rpm > 0){
        crank_rev_period = ((60*1024) / csc_sim_crank_rpm);
        cum_cranks++;
        last_crank_event += crank_rev_period; 
    }
}

void loop()
{
  delay(100);
  if (!serverIsConnected) { 
    delay(100);
    return;
  }
  
  if (millis() > SIMtimeInterval) { // Update simulated values
      Simulate_speed_cadence_power();
      SIMtimeInterval = millis() + SIM_TIME_SPAN;
  }
  if(millis() > HRMtimeInterval) { // Update HRM value and sent to client
      //digitalToggle(LED_RED);
      Simulate_heart_rate();
      // Handle HRM measurement
      server_HR_Measurement_Data[0] = (0 | 2 | 4); // 8 bit value, Sensor Contact detected, Contact Supported
      server_HR_Measurement_Data[1] = sim_heart_beat_min; // Single byte: sim value for Heart Beats per minute;
      server_HR_Measurement_Data[2] = 0;
      server_HR_Measurement_Data[3] = 0;
      // The characteristic's value is updated
      server_HR_Measurement_Chr->setValue(server_HR_Measurement_Data, HR_MEASUREMENT_DATALEN);   
      server_HR_Measurement_Chr->notify(); 
      HRMtimeInterval = millis() + HRM_TIME_SPAN;   
      //}        
  }  
  if(millis() > CPCSCtimeInterval) {
      // Handle Power measurement --------------------
      // Respect Little Endian representation
      server_CP_Measurement_Data[0] = (uint8_t)(server_CP_Measurement_Flags & 0xff); // lsb flags
      server_CP_Measurement_Data[1] = (uint8_t)(server_CP_Measurement_Flags >> 8);   // msb flags 
      server_CP_Measurement_Data[2] = (uint8_t)(cps_sim_power_watt & 0xff);// lsb inst. power
      server_CP_Measurement_Data[3] = (uint8_t)(cps_sim_power_watt >> 8);  // msb inst. power                                
      // The characteristic's value is updated
      server_CP_Measurement_Chr->setValue(server_CP_Measurement_Data, CP_MEASUREMENT_DATALEN);       
      server_CP_Measurement_Chr->notify();
      //DEBUG_PRINT("Cycle Power Measurement updated to: "); DEBUG_PRINTLN(cps_sim_power_watt); 
      //}
      // END Handle Power measurement --------------------
      
      // Handle CSC data measurement ----------------------
      //First byte flags are set: 
      //Wheel Revolution Data Present
      //Crank Revolution Data Present
      server_CSC_Measurement_Data[0]= server_CSC_Measurement_Flags;
      // Setting values for cycling measures:
      // Cumulative Wheel Revolutions (unitless)
      // Last Wheel Event Time (Unit has a resolution of 1/1024s)
      // Cumulative Crank Revolutions (unitless)
      // Last Crank Event Time (Unit has a resolution of 1/1024s)           
      memcpy(&server_CSC_Measurement_Data[1], &cum_wheel_rev, 4);    // uint32_t
      memcpy(&server_CSC_Measurement_Data[5], &last_wheel_event, 2); // uint16_t
      memcpy(&server_CSC_Measurement_Data[7], &cum_cranks, 2);       // uint16_t
      memcpy(&server_CSC_Measurement_Data[9], &last_crank_event, 2); // uint16_t
      // The characteristic's value is updated
      server_CSC_Measurement_Chr->setValue(server_CSC_Measurement_Data, sizeof(server_CSC_Measurement_Data));
      server_CSC_Measurement_Chr->notify();
      //DEBUG_PRINTF("CSC Measurement/IBD updated to: %d KPH %d RPM PWR: %d\n", csc_sim_speed_kph , csc_sim_crank_rpm, cps_sim_power_watt);
      //  }
      CPCSCtimeInterval = millis() + CP_CSC_TIME_SPAN; // Set new interval before sending
      // END Handle CSC measurement ------------------------
  } // CPCSCtimeInterval
  
  // Handle Server FTMS IBD ------------------------------
  if(millis() > IBDtimeInterval) {
      // send some indoor bike data
      server_FTM_IndoorBike_Data[0] = (uint8_t)(flagMoreData | flagInstantaneousCadence | flagIntantaneousPower | flagAverageSpeed); // Flags LSO byte1
      server_FTM_IndoorBike_Data[1] = (uint8_t)flagHeartRate; // 0x00; Flags MSO byte2 all zero
      // instantaneous speed IndoorBikeData needs km/h in resolution of 0.01 --> csc_sim_speed_kph needs to be multiplied with 100
      server_FTM_IndoorBike_Data[2] = ((100*csc_sim_speed_kph) & 0xFF); // Instantaneous Speed LSO
      server_FTM_IndoorBike_Data[3] = ((100*csc_sim_speed_kph) >> 8) & 0xFF; // Instantaneous Speed MSO
      // instantaneous cadence IndoorBikeData is Crank RPM in resolution of 0.5 --> csc_sim_crank_rpm needs to be multiplied with 2
      server_FTM_IndoorBike_Data[4] = (2*csc_sim_crank_rpm) & 0xFF; // Instantaneous Cadence LSO
      server_FTM_IndoorBike_Data[5] = ((2*csc_sim_crank_rpm) >> 8) & 0xFF; // Instantaneous Cadence MSO
      // instantaneous Power IndoorBikeData in resolution of 1 --> No multiplication
      server_FTM_IndoorBike_Data[6] = cps_sim_power_watt & 0xFF; // Instantaneous Power LSO
      server_FTM_IndoorBike_Data[7] = (cps_sim_power_watt >> 8) & 0xFF; // Instantaneous Power MSO
      // Single byte: sim value for Heart Beats per minute
      server_FTM_IndoorBike_Data[8] = sim_heart_beat_min; 
      // The characteristic's value is updated
      server_FTM_IndoorBikeData_Chr->setValue(server_FTM_IndoorBike_Data, FTM_INDOORBIKE_DATALEN);       
      server_FTM_IndoorBikeData_Chr->notify(); 
      //  }
      // Handle Server FTMS IBD ------------------------------
      IBDtimeInterval = millis() + IBD_TIME_SPAN; // Set new interval
  } // End Server FTMS IBD ------------------------------

} // END of Loop and program