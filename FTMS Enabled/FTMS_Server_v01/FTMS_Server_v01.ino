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
 *  This Feather nRF52840 tested code advertises and enables the relevant 
 *  Cycling Trainer Services: CPS, CSC and FTMS.
 *  It allows to connect to Cycling apps like Zwift (aka Client or Central)!
 *  It simulates a connected Cycling Trainer and in BLE terms it is a Server or 
 *  or in BlueFruit BLE library terms it is a Peripheral
 *  Requirements: Zwift app or alike and Feather nRF52 board
 *  1) Start Zwift and wait for the Devices Pairing Screen
 *  2) Unpair all previously paired devices
 *  3) Upload and Run this code on the Feather nRF52
 *  4) Start the Serial Monitor to catch debugging info
 *  5) Search on Zwift pairing screen for the Feather nRF52: "nRF52 SIM"
 *  6) Pair all three "Simulator" devices: Power, Cadence and Controllable
 *  7) Start a default Zwift ride or any ride you wish
 *  8) Make Serial Monitor visible on top of the Zwift window 
 *  9) Inspect the info presented by Serial Monitor
 * 10) Notice how your avatar is riding the route...
 *  
 */

#include <bluefruit.h>

const uint8_t MAX_PAYLOAD = 20; // Max 20 byte data size for single packet BLE transfer
uint16_t server_Connection_Handle = BLE_CONN_HANDLE_INVALID;

// -------------------------------------------------------------------------------------------
// COMPILER DIRECTIVE to allow/suppress SERIAL.PRINT messages that help debugging...
// Uncomment to activate
#define DEBUG
// --------------------------------------------------------------------------------------------

/*
 * Cycling Speed and Cadence Service
 * CSC Service:   0x1816 
 * CSC Measurement Characteristic:  0x2A5B
 * CSC Feature Characteristic:      0x2A5C  
 * CSC Control Point Characteristic:0x2A55 <not implemented> 
*/
BLEService        server_CyclingSpeedCadence_Service = BLEService(UUID16_SVC_CYCLING_SPEED_AND_CADENCE); 
BLECharacteristic server_CSC_Measurement_Chr = BLECharacteristic(UUID16_CHR_CSC_MEASUREMENT);              // Notify, Read
BLECharacteristic server_CSC_Feature_Chr = BLECharacteristic(UUID16_CHR_CSC_FEATURE);                      // Read
BLECharacteristic server_CSC_Location_Chr = BLECharacteristic(UUID16_CHR_SENSOR_LOCATION);                 // Read

/* Cycling Power Service
 * CP Service: 0x1818  
 * CP Characteristic: 0x2A63 (Measurement)
 * CP Characteristic: 0x2A65 (Feature)
 * CP Characteristic: 0x2A5D (Location)
 * CP Characteristic: 0x2A66 (Control Point)
 */
BLEService        server_CylingPower_Service = BLEService(UUID16_SVC_CYCLING_POWER);
BLECharacteristic server_CP_Measurement_Chr = BLECharacteristic(UUID16_CHR_CYCLING_POWER_MEASUREMENT);    // Notify, Read
BLECharacteristic server_CP_Feature_Chr = BLECharacteristic(UUID16_CHR_CYCLING_POWER_FEATURE);            // Read
BLECharacteristic server_CP_Location_Chr = BLECharacteristic(UUID16_CHR_SENSOR_LOCATION);                 // Read
BLECharacteristic server_CP_ControlPoint_Chr = BLECharacteristic(UUID16_CHR_CYCLING_POWER_CONTROL_POINT); // Indicate, Write

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
BLEService server_FitnessMachine_Service = BLEService(0x1826); // FTM Service
BLECharacteristic server_FTM_Feature_Chr = BLECharacteristic(UUID16_CHR_FITNESS_MACHINE_FEATURE); //  Fitness Machine Feature, mandatory, read
BLECharacteristic server_FTM_IndoorBikeData_Chr = BLECharacteristic(UUID16_CHR_INDOOR_BIKE_DATA); //  Indoor Bike Data, optional, notify
BLECharacteristic server_FTM_TrainingStatus_Chr = BLECharacteristic(UUID16_CHR_TRAINING_STATUS); //  Training Status, optional, read & notify
BLECharacteristic server_FTM_SupportedResistanceLevelRange_Chr = BLECharacteristic(UUID16_CHR_SUPPORTED_RESISTANCE_LEVEL_RANGE); // Supported Resistance Level, read, optional
BLECharacteristic server_FTM_SupportedPowerRange_Chr = BLECharacteristic(UUID16_CHR_SUPPORTED_POWER_RANGE); // Supported Power Levels, read, optional
BLECharacteristic server_FTM_ControlPoint_Chr = BLECharacteristic(UUID16_CHR_FITNESS_MACHINE_CONTROL_POINT); //  Fitness Machine Control Point, optional, write & indicate
BLECharacteristic server_FTM_Status_Chr = BLECharacteristic(UUID16_CHR_FITNESS_MACHINE_STATUS); //  Fitness Machine Status, mandatory, notify

// Server DIS (Device Information Service) helper class instance
BLEDis server_bledis;  // Read

// -------- Definitions of all FTM Characteristic Data buffers -----------------------
// Define and fill all buffers used to write to the characteristics and initial values
// -----------------------------------------------------------------------------------
// Device Information Service
// -----------------------------------------------------------------------------------
unsigned char client_DIS_Manufacturer_Str[MAX_PAYLOAD] = "Adafruit Industries";
unsigned char client_DIS_ModelNumber_Str[MAX_PAYLOAD] = "nRf52840";
unsigned char FirmwareRevStr[] = "0.0.0";
unsigned char client_DIS_SerialNumber_Str[MAX_PAYLOAD] = "1234";
unsigned char HardwareRevStr[] = "0.0.0";
unsigned char SoftwareRevStr[] = "0.0.0";

// -----------------------------------------------------------------------------------
// Fitness Machine Service
// -----------------------------------------------------------------------------------
// Features Characteristic [client_FTM_Feature_Data]
/* 
Fitness Machine Features (byte 1-4):
Average Speed Supported
Cadence Supported
Resistance Level Supported
Power Measurement Supported
Byte 1
Bit0  Average Speed Supported         *
Bit1  Cadence Supported               *
Bit2  Total Distance Supported
Bit3  Inclination Supported
Bit4  Elevation Gain Supported
Bit5  Pace Supported
Bit6  Step Count Supported
Bit7  Resistance Level Supported      *
Byte 2
Bit8  Stride Count Supported
Bit9  Expended Energy Supported
Bit10 Heart Rate Measurement Supported
Bit11 Metabolic Equivalent Supported
Bit12 Elapsed Time Supported
Bit13 Remaining Time Supported
Bit14 Power Measurement Supported     *
Bit15 Force on Belt and Power Output Supported
Byte 3 and 4
Bit16 User Data Retention Supported
Bit17-31Reserved for Future Use

Target Setting Features (byte 5-8) :
Byte 5
Bit0  Speed Target Setting Supported
Bit1  Inclination Target Setting Supported
Bit2  Resistance Target Setting Supported           ?
Bit3  Power Target Setting Supported                ?
Bit4  Heart Rate Target Setting Supported
Bit5  Targeted Expended Energy Configuration Supported
Bit6  Targeted Step Number Configuration Supported
Bit7  Targeted Stride Number Configuration Supported
Byte 6
Bit8  Targeted Distance Configuration Supported
Bit9  Targeted Training Time Configuration Supported
Bit10 Targeted Time in Two Heart Rate Zones Configuration Supported
Bit11 Targeted Time in Three Heart Rate Zones Configuration Supported
Bit12 Targeted Time in Five Heart Rate Zones Configuration Supported
Bit13 Indoor Bike Simulation Parameters Supported   *
Bit14 Wheel Circumference Configuration Supported
Bit15 Spin Down Control Supported                   
Byte 7 and 8
Bit16 Targeted Cadence Configuration Supported
Bit17-31 Reserved for Future Use
*/
const uint8_t AverageSpeedSupported = 1;      // byte 1
const uint8_t CadenceSupported = 2;           // byte 1
const uint8_t ResistanceLevelSupported = 128; // byte 1
const uint8_t PowerMeasurementSupported = 64; // byte 2
const uint8_t IndoorBikeSimulationParametersSupported = 32; // byte 6

const uint8_t FTM_FEATURE_FIXED_DATALEN = 8; // Fixed value!
//  Little Endian        LSO                                MSO
//  Feature = {0b10000011,0b01000000,0x00,0x00,0x00,0b00100000,0x00,0x00}; // Binary and Hex
//  Feature = {0x80,0x40,0x00,0x00,0x00,0x20,0x00,0x00}; // Hex
unsigned char client_FTM_Feature_Data[FTM_FEATURE_FIXED_DATALEN] = \
                                           {(unsigned char)(AverageSpeedSupported | CadenceSupported | ResistanceLevelSupported),\
                                            (unsigned char)(PowerMeasurementSupported),0x00,0x00,0x00,\
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
const uint16_t flagExpendedEnergy = 256;
const uint16_t flagHeartRate = 512;
const uint16_t flagMetabolicEquivalent = 1024;
const uint16_t flagElapsedTime = 2048;
const uint16_t flagRemainingTime = 4096;

const uint8_t FTM_INDOORBIKE_DATALEN = 8; // NO Fixed Len --> setMaxLen when all flags are set (!) equals 28, so a MAXLEN = 20 will do !!
// Notice a "fixed len" of 8 is perfect with the following flags set ! Other flags is other "size"
unsigned char server_FTM_IndoorBike_Data[FTM_INDOORBIKE_DATALEN] = {(uint8_t)(flagMoreData | flagAverageSpeed | flagInstantaneousCadence | \
                                                                      flagIntantaneousPower),0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 

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

// -----------------------------------------------------------------------------------
// Server Cycling Power variables
// -----------------------------------------------------------------------------------
uint16_t server_CP_Measurement_Flags = 0; // Cycle power measurement config flags field variable
/* The Cycling Power Measurement characteristic is a variable length structure containing a Flags field, 
   an Instantaneous Power field and, based on the contents of the Flags field, may contain one or more additional fields...
                             { (uint8_t)(server_CP_Measurement_Flags & 0xff), (uint8_t)(server_CP_Measurement_Flags >> 8),  // flags 
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
// Notice that a MaxLen of 20 is enough to cover many flags set.... HOWEVER with ALL flags set MaxLen equals 35
const uint8_t CP_MEASUREMENT_DATALEN = 4;  // With present flags FixedLen = 4 
uint8_t server_CP_Measurement_Data[CP_MEASUREMENT_DATALEN];

// Configure the Cycle Power Feature characteristic
  // Min Len    = 1
  // Max Len    = 32 bits
  //    B0:3    = UINT8 - Cycling Power Feature (MANDATORY)
  //      b0    = Pedal power balance supported; 0 = false, 1 = true                                          *
  //      b1    = Accumulated torque supported; 0 = false, 1 = true                                           *
  //      b2    = Wheel revolution data supported; 0 = false, 1 = true
  //      b3    = Crank revolution data supported; 0 = false, 1 = true                                        *
  //      b4    = Extreme magnatudes supported; 0 = false, 1 = true
  //      b5    = Extreme angles supported; 0 = false, 1 = true
  //      b6    = Top/bottom dead angle supported; 0 = false, 1 = true
  //      b7    = Accumulated energy supported; 0 = false, 1 = true
  //      b8    = Offset compensation indicator supported; 0 = false, 1 = true
  //      b9    = Offset compensation supported; 0 = false, 1 = true                                          *
  //      b10   = Cycling power measurement characteristic content masking supported; 0 = false, 1 = true     *
  //      b11   = Multiple sensor locations supported; 0 = false, 1 = true
  //      b12   = Crank length adj. supported; 0 = false, 1 = true
  //      b13   = Chain length adj. supported; 0 = false, 1 = true
  //      b14   = Chain weight adj. supported; 0 = false, 1 = true
  //      b15   = Span length adj. supported; 0 = false, 1 = true
  //      b16   = Sensor measurement context; 0 = force, 1 = torque                                           *
  //      b17   = Instantaineous measurement direction supported; 0 = false, 1 = true
  //      b18   = Factory calibrated date supported; 0 = false, 1 = true
  //      b19   = Enhanced offset compensation supported; 0 = false, 1 = true
  //   b20:21   = Distribtue system support; 0 = legacy, 1 = not supported, 2 = supported, 3 = RFU
  //   b22:32   = Reserved
const uint8_t CP_FEATURE_DATALEN = 4; // Set MaxLen to 4
uint32_t client_CP_Feature_Flags = {0b00000000000000010000011010001011}; // Relevant Cycling Power features: set to TRUE!!!
//uint8_t server_CP_Feature_Data[CP_FEATURE_DATALEN] = {0x8B, 0x06, 0x01, 0x00}; // In our case --> flags taken from Quarq power meter in little Endian format
//uint8_t server_CP_Feature_Data[CP_FEATURE_DATALEN] = {(uint8_t)(client_CP_Feature_Flags & 0xff), (uint8_t)(client_CP_Feature_Flags >> 8), \
                                                        (uint8_t)(client_CP_Feature_Flags >> 16), (uint8_t)(client_CP_Feature_Flags >> 24)};
// Server CPCP Cyling Power Control Point characteristic
const uint16_t CP_CONTROL_POINT_DATALEN = 5;
// 8 bit location of the sensor
uint8_t client_CP_Location_Value = 12; // --> rear wheel !

// -----------------------------------------------------------------------------------
// Server Cycling Speed and Cadence variables
// -----------------------------------------------------------------------------------
/*       CSC Measurement flags */
#define     CSC_MEASUREMENT_WHEEL_REV_PRESENT       0x01
#define     CSC_MEASUREMENT_CRANK_REV_PRESENT       0x02
/*       CSC feature flags */
#define     CSC_FEATURE_WHEEL_REV_DATA              0x01
#define     CSC_FEATURE_CRANK_REV_DATA              0x02
#define     CSC_FEATURE_MULTIPLE_SENSOR_LOC         0x04 
/*       CSC simulation configuration */

// 8 Bit flag to set: which type of data present in CSC measurement packet
uint8_t server_CSC_Measurement_Flags = (uint8_t)(CSC_MEASUREMENT_WHEEL_REV_PRESENT | CSC_MEASUREMENT_CRANK_REV_PRESENT);
const uint8_t CSC_MEASUREMENT_DATALEN = 11; //
uint8_t server_CSC_Measurement_Data[CSC_MEASUREMENT_DATALEN] = {}; 
 // 16 Bit flag to set present CSC Features
const uint8_t CSC_FEATURE_FIXED_DATALEN = 2; // Fixed Len
const uint16_t CSC_FEATURES = (CSC_FEATURE_WHEEL_REV_DATA | CSC_FEATURE_CRANK_REV_DATA | CSC_FEATURE_MULTIPLE_SENSOR_LOC);
uint16_t client_CSC_Feature_Flags = CSC_FEATURES; 
// 8 bit location of the sensor
uint8_t client_CSC_Location_Value = 12; // --> rear wheel !

// -----------------------------------------------------------------------------------
// Server Generic Access variables
// -----------------------------------------------------------------------------------
uint16_t client_GA_Appearance_Value = 0x0480;  // decimal: 1152 -> Cycling
// Set the advertised device name (keep it short!)
unsigned char client_GA_DeviceName_Data[MAX_PAYLOAD] = "nRF52 SIM";

// ------------------- End of Definitions of all Characteristic Data buffers  ---------------------------

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
// ----------------------------------------------
// global CSC and CPS Variables for test purposes ONLY!
// ----------------------------------------------
uint32_t cum_wheel_rev = 0;
uint16_t last_wheel_event = 0;
uint16_t cum_cranks = 0;
uint16_t last_crank_event = 0;
uint16_t csc_sim_speed_kph = MIN_SPEED_KPH ;  // Speed in Km/hour 
uint16_t csc_sim_crank_rpm = MIN_CRANK_RPM ;  // RPM
uint16_t cps_sim_power_watt = MIN_POWER_WATT; // Power in Watts for simulation
// --------------------------------------------

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb, milliseconds

  Serial.println(" Feather nRF52 Server/Peripheral: FTMS, CPS and CSC");
  Serial.println("-------------------  Version 01 ---------------------");
  // Initialise the Bluefruit module
  Serial.println("Initialise the Bluefruit nRF52 module: Peripheral");
#endif
  // begin (Peripheral = 1, Central = 0)
  Bluefruit.begin(1, 0);

  // Configure and Start the Device Information Service
#ifdef DEBUG
  Serial.println("Configuring the Server Device Information Service");
#endif
  server_setupDIS();
  // Setup the Cycle Power Service, Speed & Cadence Service and FTMS
#ifdef DEBUG
  Serial.println("Configuring the Server Cycle Power Service");
#endif
  server_setupCPS();
#ifdef DEBUG
  Serial.println("Configuring the Server Cadence and Speed Service");  
#endif
  server_setupCSC();
#ifdef DEBUG
  Serial.println("Configuring the Server Fitness Machine Service");  
#endif
  server_setupFTMS();
  // Setup and start advertising
#ifdef DEBUG
  Serial.println("Setting up the Server advertising payload(s)");
#endif
  server_startADV();
#ifdef DEBUG
  Serial.println("Server is advertising: FTMS, CPS and CSC!");
#endif
}

/*
 * Functions
 */
void server_startADV(void)
{
// Setup and start advertising

  // Supported tx_power values depending on mcu:
  // - nRF52832: -40dBm, -20dBm, -16dBm, -12dBm, -8dBm, -4dBm, 0dBm, +3dBm and +4dBm.
  // - nRF52840: -40dBm, -20dBm, -16dBm, -12dBm, -8dBm, -4dBm, 0dBm, +2dBm, +3dBm, +4dBm, +5dBm, +6dBm, +7dBm and +8dBm.
  Bluefruit.setTxPower(4); // See above for supported values: +4dBm
  // Set blink rate in advertising mode
  Bluefruit.setConnLedInterval(250);
#ifdef DEBUG
  Serial.printf("Setting Server Device Name to: [%s]\n", client_GA_DeviceName_Data);
#endif
  Bluefruit.setName((const char*)client_GA_DeviceName_Data);
  if (Bluefruit.setAppearance(client_GA_Appearance_Value))
  {
#ifdef DEBUG
    Serial.printf("Setting Server Appearance to [%d] Generic: Cycling\n", client_GA_Appearance_Value);
#endif
  }
  // Set the connect/disconnect callback handlers
  Bluefruit.Periph.setConnectCallback(server_connect_callback);
  Bluefruit.Periph.setDisconnectCallback(server_disconnect_callback);

  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include CPS CSC and FTMS BLEServices defined above
  Bluefruit.Advertising.addService(server_CylingPower_Service);  
  Bluefruit.Advertising.addService(server_CyclingSpeedCadence_Service);
  Bluefruit.Advertising.addService(server_FitnessMachine_Service);

  // Include Name
  Bluefruit.Advertising.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
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
  server_FTM_IndoorBikeData_Chr.setFixedLen(FTM_INDOORBIKE_DATALEN);
//  server_FTM_IndoorBikeData_Chr.setMaxLen(MAX_PAYLOAD); // To be on the safe side, when many features are set! 
  server_FTM_IndoorBikeData_Chr.setCccdWriteCallback(server_cccd_callback);  // Optionally capture CCCD updates
  server_FTM_IndoorBikeData_Chr.begin();
//  server_FTM_IndoorBikeData_Chr.notify(server_FTM_IndoorBike_Data, FTM_INDOORBIKE_DATALEN);

  // Training Status, optional, read & notify
  server_FTM_TrainingStatus_Chr.setProperties(CHR_PROPS_NOTIFY); 
  server_FTM_TrainingStatus_Chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS); 
  server_FTM_TrainingStatus_Chr.setFixedLen(FTM_TRAINING_STATUS_FIXED_DATALEN);
  server_FTM_TrainingStatus_Chr.setCccdWriteCallback(server_cccd_callback);  // Optionally capture CCCD updates
  server_FTM_TrainingStatus_Chr.begin();
//  server_FTM_TrainingStatus_Chr.notify(server_FTM_TrainingStatus_Data, FTM_TRAINING_STATUS_FIXED_DATALEN);  

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
  server_FTM_ControlPoint_Chr.begin();

  // Fitness Machine Status
  server_FTM_Status_Chr.setProperties(CHR_PROPS_NOTIFY); 
  server_FTM_Status_Chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS); 
  server_FTM_Status_Chr.setMaxLen(FTM_STATUS_DATALEN); // Notice that with the present Target Features, Machine Status we only use 2 bytes max!
  server_FTM_Status_Chr.setCccdWriteCallback(server_cccd_callback);  // Optionally capture CCCD updates
  server_FTM_Status_Chr.begin();
//  server_FTM_Status_Chr.notify(server_FTM_Status_Data, 1);  // Only one byte
}

void server_FTM_ControlPoint_Chr_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  uint8_t ftmcpDataLen = (uint8_t)len;
  memset(server_FTM_Control_Point_Data.bytes, 0, sizeof(server_FTM_Control_Point_Data.bytes));
#ifdef DEBUG
  // Display the raw request packet
  Serial.printf(" --> Raw FTM Control Point Data [len: %d] ", ftmcpDataLen);
#endif
  // Transfer the contents of data to server_FTM_Control_Point_Data.bytes
  for (int i = 0; i < ftmcpDataLen; i++) {
      server_FTM_Control_Point_Data.bytes[i] = *data++;
    }
/* Decodes an incoming Fitness Machine Control Point request */
#ifdef DEBUG
    Serial.printf("[OpCode: %02X] [Values: ", server_FTM_Control_Point_Data.values.OPCODE, HEX);
    for (int i=0; i<ftmcpDataLen; i++) { 
      Serial.printf("%02X ", server_FTM_Control_Point_Data.values.OCTETS[i], HEX); 
    }
    Serial.println("]");
#endif
  switch(server_FTM_Control_Point_Data.values.OPCODE) {
    case ftmcpRequestControl: {
      // Always allow control point to confirm: OK!
      ftmcpRespConfirmBuffer[1] = server_FTM_Control_Point_Data.values.OPCODE;
      server_FTM_ControlPoint_Chr.indicate(ftmcpRespConfirmBuffer, 3);
#ifdef DEBUG
      Serial.println("Request Control of Machine!");
#endif
      break;
    }
    case ftmcpStartOrResume: {
      server_FTM_TrainingStatus_Data[0] = 0x00;
      server_FTM_TrainingStatus_Data[1] = 0x04; // High Intensity Interval
      server_FTM_TrainingStatus_Chr.notify(server_FTM_TrainingStatus_Data, FTM_TRAINING_STATUS_FIXED_DATALEN);

      server_FTM_Status_Data[0] = 0x04;  // Machine status: Started or Resumed
      server_FTM_Status_Data[1] = 0x00;
      server_FTM_Status_Chr.notify(server_FTM_Status_Data, 1); // Only First Byte
       // confirm OK!
      ftmcpRespConfirmBuffer[1] = server_FTM_Control_Point_Data.values.OPCODE;
      server_FTM_ControlPoint_Chr.indicate(ftmcpRespConfirmBuffer, 3);
#ifdef DEBUG
      Serial.println("Start or Resume Machine!");
#endif
      break;
    }
    case ftmcpStopOrPause: {
      server_FTM_TrainingStatus_Data[0] = 0x00;  // 0x00 No String present; 0x02 Extended string present; 0x01 Training Status String present
      server_FTM_TrainingStatus_Data[1] = 0x01;  // idle
      server_FTM_TrainingStatus_Chr.notify(server_FTM_TrainingStatus_Data, FTM_TRAINING_STATUS_FIXED_DATALEN);
      server_FTM_Status_Data[0] = 0x02;  // Machine status: Stopped or Paused
      server_FTM_Status_Data[1] = 0x01;  // Stopped parameter
      server_FTM_Status_Chr.notify(server_FTM_Status_Data, 2); // Only first 2 bytes
       // confirm OK!
      ftmcpRespConfirmBuffer[1] = server_FTM_Control_Point_Data.values.OPCODE;
      server_FTM_ControlPoint_Chr.indicate(ftmcpRespConfirmBuffer, 3);
#ifdef DEBUG
      Serial.println("Stop or Pause Machine, Parameter: Stop!");
#endif
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
#ifdef DEBUG // Remember, if debugging with Zwift, that these values are divided by 2 if in normal 50% settings!
        Serial.println("Set Indoor Bike Simulation Parameters!");
        Serial.print("Wind speed (1000): "); Serial.print(wind_speed);
        Serial.print(" | Grade (100): "); Serial.print(grade);
        Serial.print(" | Crr (10000): "); Serial.print(crr);
        Serial.print(" | Cw (100): "); Serial.println(cw);
#endif
      ftmcpRespConfirmBuffer[1] = server_FTM_Control_Point_Data.values.OPCODE;
      server_FTM_ControlPoint_Chr.indicate(ftmcpRespConfirmBuffer, 3);
      break;
    }
    case ftmcpReset: {
      server_FTM_TrainingStatus_Data[0] = 0x00;  // 0x00 No String present; 0x02 Extended string present; 0x01 Training Status String present
      server_FTM_TrainingStatus_Data[1] = 0x01;  // idle
      server_FTM_TrainingStatus_Chr.notify(server_FTM_TrainingStatus_Data, FTM_TRAINING_STATUS_FIXED_DATALEN);

      server_FTM_Status_Data[0] = 0x01;  // Machine Status: Reset
      server_FTM_Status_Data[1] = 0x00;
      server_FTM_Status_Chr.notify(server_FTM_Status_Data, 1); // Only First Byte
      ftmcpRespConfirmBuffer[1] = server_FTM_Control_Point_Data.values.OPCODE;
      server_FTM_ControlPoint_Chr.indicate(ftmcpRespConfirmBuffer, 3);
#ifdef DEBUG
      Serial.println("Reset Machine!");
#endif
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
      server_FTM_ControlPoint_Chr.indicate(ftmcpRespUnknownBuffer, 3);
#ifdef DEBUG
      Serial.println("Unresolved OpCode!");
#endif
      break;
    }
  } // switch
}

void server_setupCPS(void)
{
  // Configure the Cycling Power service
  // See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.service.cycling_power.xml
  // Supported Characteristics:
  // Name                         UUID    Requirement Properties
  // ---------------------------- ------  ----------- ----------
  // Cycle Power Measurement      0x2A63  Mandatory   Notify
  // Cycle Power Feature          0x2A65  Mandatory   Read
  // Sensor Location              0x2A5D  Mandatory   Read
  
  server_CylingPower_Service.begin();

  // Note: You must call .begin() on the BLEService before calling .begin() on
  // any characteristic(s) within that service definition.. Calling .begin() on
  // a BLECharacteristic will cause it to be added to the last BLEService that
  // was 'begin()'ed!

  // Configure the Cycle Power Measurement characteristic
  // See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.cycling_power_measurement.xml
  // Properties = Notify
  // Min Len    = 2
  // Max Len    = 34
  //    B0:1    = UINT16  - Flag (MANDATORY)
  //      b0    = Pedal power balance present; 0 = false, 1 = true
  //      b1    = Pedal power balance reference; 0 = unknown, 1 = left
  //      b2    = Accumulated torque present; 0 = false, 1 = true
  //      b3    = Accumulated torque source; 0 = wheel, 1 = crank
  //      b4    = Wheel revolution data present; 0 = false, 1 = true
  //      b5    = Crank revolution data present; 0 = false, 1 = true
  //      b6    = Extreme force magnatudes present; 0 = false, 1 = true
  //      b7    = Extreme torque magnatues present; 0 = false, 1 = true
  //      b8    = Extreme angles present; 0 = false, 1 = true
  //      b9    = Top dead angle present; 0 = false, 1 = true
  //      b10   = Bottom dead angle present; 0 = false, 1 = true
  //      b11   = Accumulated energy present; 0 = false, 1 = true
  //      b12   = Offset compensation indicator; 0 = false, 1 = true
  //      b13   = Reseved
  //      b14   = n/a 
  //      b15   = n/a
  //    B2:3    = SINT16 - Instantaneous power, Watts (decimal)
  //    B4      = UINT8 -  Pedal power balance, Percent (binary) 1/2
  //    B5:6    = UINT16 - Accumulated torque, Nm; res (binary) 1/32
  //    B7:10   = UINT32 - Cumulative wheel revolutions, (decimal)
  //    B11:12  = UINT16 - Last wheel event time, second (binary) 1/2048
  //    B13:14  = UINT16 - Cumulative crank revolutions, (decimal)
  //    B15:16  = UINT16 - Last crank event time, second (binary) 1/1024 
  //    B17:18  = SINT16 - Max force magnitude, Newton (decimal)
  //    B19:20  = SINT16 - Min force magnitude, Newton (decimal)
  //    B21:22  = SINT16 - Max torque magnitude, Nm (binary) 1/1024
  //    B23:24  = SINT16 - Min torque magnitude, Nm (binary) 1/1024
  //    B25:26  = UINT12 - Max angle, degree (decimal)
  //    B27:28  = UINT12 - Min angle, degree (decimal)
  //    B29:30  = UINT16 - Top dead spot angle, degree (decimal)
  //    B31:32  = UINT16 - Bottom dead spot angle, degree (decimal)
  //    B33:34  = UINT16 - Accumulated energy, kJ (decimal)
  
  server_CP_Measurement_Chr.setProperties(CHR_PROPS_NOTIFY); 
  server_CP_Measurement_Chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS); 
  server_CP_Measurement_Chr.setFixedLen(CP_MEASUREMENT_DATALEN); // Notice that this value only works with present flags set!
//  server_CP_Measurement_Chr.setMaxLen(MAX_PAYLOAD); // Will work in most cases!
  server_CP_Measurement_Chr.setCccdWriteCallback(server_cccd_callback);  // Optionally capture CCCD updates
  server_CP_Measurement_Chr.begin();

  server_CP_ControlPoint_Chr.setProperties(CHR_PROPS_INDICATE | CHR_PROPS_WRITE); 
  server_CP_ControlPoint_Chr.setPermission(SECMODE_OPEN, SECMODE_OPEN); 
  server_CP_ControlPoint_Chr.setMaxLen(CP_CONTROL_POINT_DATALEN); // The charactersitic's data set varies in length
  server_CP_ControlPoint_Chr.setCccdWriteCallback(server_cccd_callback);  // Optionally capture CCCD updates
  server_CP_ControlPoint_Chr.begin();
  server_CP_ControlPoint_Chr.setWriteCallback(server_CP_ControlPoint_Chr_callback); // Respond to events with "Write with Response" !!

  server_CP_Feature_Chr.setProperties(CHR_PROPS_READ);
  server_CP_Feature_Chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  server_CP_Feature_Chr.setMaxLen(CP_FEATURE_DATALEN);
  server_CP_Feature_Chr.begin();
  server_CP_Feature_Chr.write32(client_CP_Feature_Flags);
//  server_CP_Feature_Chr.write(server_CP_Feature_Data, CP_FEATURE_DATALEN);

  // Configure the Sensor Location characteristic
  // See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.sensor_location.xml
  // Properties = Read
  // Min Len    = 1
  // Max Len    = 1
  //  B0:1      = UINT8 - Sensor Location
  //  Dec.
  //      0     = Other
  //      1     = Top of shoe
  //      2     = In shoe
  //      3     = Hip
  //      4     = Front wheel
  //      5     = Left crank
  //      6     = Right crank
  //      7     = Left pedal
  //      8     = Right pedal
  //      9     = Front hub
  //      10    = Rear dropout
  //      11    = Chainstay
  //      12    = Rear wheel
  //      13    = Rear hub
  //      14    = Chest
  //      15    = Spider
  //      16    = Chain ring
  //  17:255    = Reserved
   
  server_CP_Location_Chr.setProperties(CHR_PROPS_READ);
  server_CP_Location_Chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  server_CP_Location_Chr.setFixedLen(1); // UINT8
  server_CP_Location_Chr.begin();
  server_CP_Location_Chr.write8(client_CP_Location_Value);  // Set the characteristic
}

void server_setupCSC()
{
  // Configure the Cadence and Speed service
  // See: https://https://www.bluetooth.com/specifications/specs/cycling-speed-and-cadence-service-1-0/
  // See: https://www.bluetooth.com/specifications/specs/cycling-speed-and-cadence-profile-1-0/
  // Supported Characteristics:
  // Name                          UUID    Requirement Properties
  // ----------------------------  ------  ----------- ----------
  // Cadence and Speed Measurement 0x2A5B  Mandatory   Notify
  // Cadence and Speed Feature     0x2A5C  Mandatory   Read
  // Sensor Location               0x2A5D  Mandatory   Read 
  
  server_CyclingSpeedCadence_Service.begin();
  
  server_CSC_Measurement_Chr.setProperties(CHR_PROPS_NOTIFY);  
  server_CSC_Measurement_Chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS); 
  server_CSC_Measurement_Chr.setFixedLen(CSC_MEASUREMENT_DATALEN); // With the present flags set!
//  server_CSC_Measurement_Chr.setMaxLen(MAX_PAYLOAD);
  server_CSC_Measurement_Chr.setCccdWriteCallback(server_cccd_callback);  // Optionally capture CCCD updates
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
/* Control commands
       {
        OPCODE_SET_CUMULATIVE_VALUE               =  1,
        OPCODE_UPDATE_SENSOR_LOCATION             =  2,
        OPCODE_REQUEST_SUPPORTED_SENSOR_LOCATIONS =  3,
        OPCODE_SET_CRANK_LENGTH                   =  4,
        OPCODE_REQUEST_CRANK_LENGTH               =  5,
        OPCODE_SET_CHAIN_LENGTH                   =  6,
        OPCODE_REQUEST_CHAIN_LENGTH               =  7,
        OPCODE_SET_CHAIN_WEIGHT                   =  8,
        OPCODE_REQUEST_CHAIN_WEIGHT               =  9,
        OPCODE_SET_SPAN_LENGTH                    = 10,
        OPCODE_REQUEST_SPAN_LENGTH                = 11,
        OPCODE_START_OFFSET_COMPENSATION          = 12,
        OPCODE_MASK_CYCLING_POWER_MEASUREMENT_CHARACTERISTIC_CONTENT = 13,
        OPCODE_REQUEST_SAMPLING_RATE              = 14,
        OPCODE_REQUEST_FACTORY_CALIBRATION_DATE   = 15,
        OPCODE_RESPONSE_CODE                      = 32,
    };
*/
// Interpret the Control Point command, handle and send a response message with the requested data !!
/* Notice this is NOT implemented at all !!!!
cpcpRespData[CP_CONTROL_POINT_DATALEN] = {};
const uint8_t RESPONSE_SUCCESS = 1;
const uint8_t RESPONSE_OP_CODE_NOT_SUPPORTED = 2;
const uint8_t RESPONSE_INVALID_PARAMETER = 3;
const uint8_t RESPONSE_OPERATION_FAILED = 4;
const uint8_t OPCODE_RESPONSE_CODE = 32;
Len = 3
cpcpRespData[0] = OpCode Response Code = 0x20 // 32 Decimal
cpcpRespData[1] = OpCode
cpcpRespData[2] = responseValue // success, not supported, invalid parameter, etcetera
Len = 5
cpcpRespData[3] = (uint8_t)(responseParameter & 0xFF);
cpcpRespData[4] = (uint8_t)(responseParameter >> 8);
*/

#ifdef DEBUG
  uint8_t cpcpDataLen = (uint8_t)len;    // Get the actual length of data bytes and type cast to (uint8_t)
  uint8_t cpcpData[cpcpDataLen];
  memset(cpcpData, 0, cpcpDataLen); // set to zero
  // Display the raw request packet actual length
  Serial.printf(" -> Server CP Control Point Data [%d] [ ", cpcpDataLen);
  // Transfer the contents of data to cpcpData
  for (int i = 0; i < cpcpDataLen; i++) {
    if ( i <= sizeof(cpcpData)) {
      cpcpData[i] = *data++;
      // Display the raw request packet byte by byte in HEX
      Serial.printf("%02X ", cpcpData[i], HEX);
    }
  }
  Serial.println(" ]  "); 
#endif
}

#ifdef DEBUG
void PrintPeerAddress(uint8_t addr[6])
{
  for (int i = 1; i < 6; i++) {
      // Display byte by byte in HEX reverse order: little Endian
      Serial.printf("%02X:",addr[(6-i)], HEX);
  }
   Serial.printf("%02X ",addr[0], HEX);
}
#endif

void server_connect_callback(uint16_t conn_handle)
{
  char central_name[MAX_PAYLOAD] = { 0 };
  uint8_t central_addr[6] = {0};
  server_Connection_Handle = conn_handle;
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);
  connection->getPeerName(central_name, sizeof(central_name));
  ble_gap_addr_t peer_address = connection->getPeerAddr();
  memcpy(central_addr, peer_address.addr, 6);
#ifdef DEBUG
  Serial.printf("Feather nRF52 (Peripheral) connected to Laptop/PC/Tablet (Central) device: [%s] MAC Address: ", central_name);
  PrintPeerAddress(central_addr);
  Serial.println();
  Serial.println("Waiting for Central Device to set CCCD Notify (enable) and start reading....");
#endif
}

static void Simulate_speed_cadence_power()
{
    uint16_t wheel_rev_period;
    uint16_t crank_rev_period;

    // Simulate increasing Power generation
    cps_sim_power_watt += 10; // In steps of 10 watt!
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

/*
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 * https://github.com/adafruit/Adafruit_nRF52_Arduino/blob/master/cores/nRF5/nordic/softdevice/s140_nrf52_6.1.1_API/include/ble_hci.h
 */
void server_disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
#ifdef DEBUG
  Serial.println("Server disconnected from Central Device");
  Serial.println("Server is advertising again!");
#endif
}

void server_cccd_callback(uint16_t conn_handle, BLECharacteristic* chr, uint16_t cccd_value)
{
    // When changed, display the Notify/Indicate Status for all characteristics
#ifdef DEBUG
    Serial.printf("Central Updated CCCD to: [%d] --> ", cccd_value);
#endif
    // Check the characteristic UUID this CCCD callback is associated with,
    // in case this handler is used for multiple CCCD records.
    if (chr->uuid == server_CP_Measurement_Chr.uuid) { 
        if (chr->notifyEnabled(conn_handle)) { 
#ifdef DEBUG
          Serial.print("Server CP: Measurement 'Notify' enabled");
#endif
        //  server_CP_Measurement_Chr.notify(server_CP_Measurement_Data, CP_MEASUREMENT_DATALEN); 
        } else {
#ifdef DEBUG
          Serial.print("Server CP: Measurement 'Notify' disabled");
#endif
        }
    }
    
    if (chr->uuid == server_CP_ControlPoint_Chr.uuid) { 
        if (chr->indicateEnabled(conn_handle)) { 
#ifdef DEBUG
            Serial.print("Server CP: ControlPoint 'Indicate' enabled");
#endif
        } else {
#ifdef DEBUG
            Serial.print("Server CP: ControlPoint 'Indicate' disabled");
#endif
        }
    }
    
    if (chr->uuid == server_CSC_Measurement_Chr.uuid) { 
        if (chr->notifyEnabled(conn_handle)) { 
#ifdef DEBUG
          Serial.print("Server CSC: Measurement 'Notify' enabled");
#endif
        //  server_CSC_Measurement_Chr.notify(server_CSC_Measurement_Data, CSC_MEASUREMENT_DATALEN);
        } else {
#ifdef DEBUG
          Serial.print("Server CSC: Measurement 'Notify' disabled");
#endif
        }
    }
    
    if (chr->uuid == server_FTM_IndoorBikeData_Chr.uuid) { 
        if (chr->notifyEnabled(conn_handle)) { 
#ifdef DEBUG
          Serial.print("Server FTM: IndoorBikeData 'Notify' enabled");
#endif
        //  server_FTM_IndoorBikeData_Chr.notify(server_FTM_IndoorBike_Data, FTM_INDOORBIKE_DATALEN);
        } else {
#ifdef DEBUG
          Serial.print("Server FTM: IndoorBikeData 'Notify' disabled");
#endif
        }
    }
    
    if (chr->uuid == server_FTM_TrainingStatus_Chr.uuid) { // Zwift is NOT interested at all !!!
        if (chr->notifyEnabled(conn_handle)) { 
#ifdef DEBUG
          Serial.print("Server FTM: TrainingStatus 'Notify' enabled");
#endif
        //  server_FTM_TrainingStatus_Chr.notify(server_FTM_TrainingStatus_Data, FTM_TRAINING_STATUS_FIXED_DATALEN); 
        } else {
#ifdef DEBUG
          Serial.print("Server FTM: TrainingStatus 'Notify' disabled");
#endif
        }
    }
    
    if (chr->uuid == server_FTM_Status_Chr.uuid) { 
        if (chr->notifyEnabled(conn_handle)) { 
#ifdef DEBUG
          Serial.print("Server FTM: FitnessMachineStatus 'Notify' enabled");
#endif
        //  server_FTM_Status_Chr.notify(server_FTM_Status_Data, 1); 
        } else {
#ifdef DEBUG
          Serial.print("Server FTM: FitnessMachineStatus 'Notify' disabled");
#endif
        }
    }
    
#ifdef DEBUG
    Serial.println();
#endif
} // end Server CCCD callback

void loop()
{
  digitalToggle(LED_RED);
  if ( Bluefruit.connected() ) {
    // Handle Power measurement --------------------
    // Respect Little Endian representation
    server_CP_Measurement_Data[0] = (uint8_t)(server_CP_Measurement_Flags & 0xff); // lsb flags
    server_CP_Measurement_Data[1] = (uint8_t)(server_CP_Measurement_Flags >> 8);   // msb flags 
    server_CP_Measurement_Data[2] = (uint8_t)(cps_sim_power_watt & 0xff);// lsb inst. power
    server_CP_Measurement_Data[3] = (uint8_t)(cps_sim_power_watt >> 8);  // msb inst. power                                
    // If it is connected but CCCD is not enabled
    // The characteristic's value is still updated although notification is not sent
    if (server_CP_Measurement_Chr.notifyEnabled(server_Connection_Handle)) { 
      server_CP_Measurement_Chr.notify(server_CP_Measurement_Data, CP_MEASUREMENT_DATALEN);
#ifdef DEBUG
      //Serial.print("Cycle Power Measurement updated to: "); Serial.println(cps_sim_power_watt); 
#endif
    }
    // Handle Power measurement --------------------
    //
    // Handle CSC data ----------------------
    Simulate_speed_cadence_power();
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
    // If it is connected but CCCD is not enabled
    // The characteristic's value is still updated although notification is not sent
    if (server_CSC_Measurement_Chr.notifyEnabled(server_Connection_Handle)) { 
      server_CSC_Measurement_Chr.notify(server_CSC_Measurement_Data, sizeof(server_CSC_Measurement_Data));
#ifdef DEBUG
      //Serial.printf("CSC Measurement/IBD updated to: %d KPH %d RPM PWR: %d\n", csc_sim_speed_kph , csc_sim_crank_rpm, cps_sim_power_watt);
#endif
    }
    // Handle CSC ------------------------
    //
    // Handle Server FTMS ------------------------------
    // send some indoor bike data
    server_FTM_IndoorBike_Data[0] = (uint8_t)(flagMoreData | flagInstantaneousCadence | flagIntantaneousPower | flagAverageSpeed); // Flags LSO 
    server_FTM_IndoorBike_Data[1] = 0x00; // Flags MSO all zero
     // instantaneous speed IndoorBikeData needs km/h in resolution of 0.01 --> csc_sim_speed_kph needs to be multiplied with 100
    server_FTM_IndoorBike_Data[2] = ((100*csc_sim_speed_kph) & 0xFF); // Instantaneous Speed LSO
    server_FTM_IndoorBike_Data[3] = ((100*csc_sim_speed_kph) >> 8) & 0xFF; // Instantaneous Speed MSO
    // instantaneous cadence IndoorBikeData is Crank RPM in resolution of 0.5 --> csc_sim_crank_rpm needs to be multiplied with 2
    server_FTM_IndoorBike_Data[4] = (2*csc_sim_crank_rpm) & 0xFF; // Instantaneous Cadence LSO
    server_FTM_IndoorBike_Data[5] = ((2*csc_sim_crank_rpm) >> 8) & 0xFF; // Instantaneous Cadence MSO
    // instantaneous Power IndoorBikeData in resolution of 1 --> No multiplication
    server_FTM_IndoorBike_Data[6] = cps_sim_power_watt & 0xFF; // Instantaneous Power LSO
    server_FTM_IndoorBike_Data[7] = (cps_sim_power_watt >> 8) & 0xFF; // Instantaneous Power MSO
    // If it is connected but CCCD is not enabled
    // The characteristic's value is still updated although notification is not sent
    if (server_FTM_IndoorBikeData_Chr.notifyEnabled(server_Connection_Handle)) { 
      server_FTM_IndoorBikeData_Chr.notify(server_FTM_IndoorBike_Data, FTM_INDOORBIKE_DATALEN); 
    }
    // Handle Server FTMS ------------------------------
  }   // endif Bluefruit.connected
} // END of Loop and program