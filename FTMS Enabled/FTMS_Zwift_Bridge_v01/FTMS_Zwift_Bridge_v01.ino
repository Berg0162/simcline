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
 *  in between, the Feather nRF52 being man-in-the-middle (MITM). 
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
 *  Requirements: Zwift app or alike, Feather nRF52 board and a FTMS/CPS/CSC supporting indoor trainer
 *  1) Upload and Run this code on the Feather nRF52
 *  2) Start the Serial Monitor to catch debugging info
 *  3) Start/Power On the indoor trainer  
 *  4) Feather nRF52 and trainer (with <name>) will pair as reported in the output
 *  5) Start Zwift on your computer or tablet and wait....
 *  6) Search on the Zwift pairing screens for the Feather nRF52 a.k.a. "<name> SIM"
 *  7) Pair: Power, Cadence and Controllable one after another with "<name> SIM"
 *  8) Optionally one can pair as well devices for heartrate and/or steering (Sterzo)
 *  9) Start the default Zwift ride or any ride you wish
 * 10) Make Serial Monitor output window visible on top of the Zwift window 
 * 11) Hop on the bike: make your avatar move and feel resistance change with the road
 * 12) Inspect the info presented by Serial Monitor.....
 *  
 *  Your trainer's device <name> is modified by the bridge to "<name> SIM", to allow for a clear distinction 
 *  between the bridge (simulating your trainer) and your original trainer, when advertising the trainer's services!
 *  You will notice this only when connecting to Zwift on the pairing screens!
 *  
 */

// -------------------------------------------------------------------------------------------
// COMPILER DIRECTIVE to allow/suppress SERIAL.PRINT messages that help debugging...
// Uncomment to activate
#define DEBUG
//  Restrict activating one or more of the following DEBUG directives --> process intensive 
//  Have caused spurious side effects like a loss of quality of service handling!!
//#define DEBUG_CP_MEASUREMENT
//#define DEBUG_CSC_MEASUREMENT
//#define DEBUG_FTM_INDOORBIKEDATA
// --------------------------------------------------------------------------------------------

#include <bluefruit.h>


const uint8_t MAX_PAYLOAD = 20; // Max 20 byte data size for single packet BLE transfer

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
// Laptop/Desktop Device Address that runs Zwift: [00:01:02:03:04:05]
// Little Endian: in reversed order !!!!
#define LAPTOPADDRESS {0x05,0x04,0x03,0x02,0x01,0x00}
// Trainer FTMS enabled Device Address [00:01:02:03:04:05]
// Little Endian: in reversed order !!!!
#define TRAINERADDRESS {0x05,0x04,0x03,0x02,0x01,0x00}
// -----------------------------------------------------------------
// Initialize connectable device registration
  Device_info_t Trainer    = {TRAINERADDRESS, {0x00}, BLE_CONN_HANDLE_INVALID, false};
  Device_info_t Laptop     = { LAPTOPADDRESS, {0x00}, BLE_CONN_HANDLE_INVALID, false};
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
BLEService server_FitnessMachine_Service = BLEService(0x1826);
BLECharacteristic server_FTM_Feature_Chr = BLECharacteristic(UUID16_CHR_FITNESS_MACHINE_FEATURE); // Read
BLECharacteristic server_FTM_IndoorBikeData_Chr = BLECharacteristic(UUID16_CHR_INDOOR_BIKE_DATA); // Notify
BLECharacteristic server_FTM_TrainingStatus_Chr = BLECharacteristic(UUID16_CHR_TRAINING_STATUS);  // Notify, Read
const uint8_t FTM_TRAINING_STATUS_FIXED_DATALEN = 2; // Fixed len
BLECharacteristic server_FTM_SupportedResistanceLevelRange_Chr = BLECharacteristic(UUID16_CHR_SUPPORTED_RESISTANCE_LEVEL_RANGE); // Read
BLECharacteristic server_FTM_SupportedPowerRange_Chr = BLECharacteristic(UUID16_CHR_SUPPORTED_POWER_RANGE);  // Read
BLECharacteristic server_FTM_ControlPoint_Chr = BLECharacteristic(UUID16_CHR_FITNESS_MACHINE_CONTROL_POINT); // Write & Indicate
BLECharacteristic server_FTM_Status_Chr = BLECharacteristic(UUID16_CHR_FITNESS_MACHINE_STATUS);              // Notify
const uint8_t FTM_STATUS_DATALEN = 3; // Max Len

/* Device Information Service helper class instance */
BLEDis server_bledis;  // Read
unsigned char FirmwareRevStr[] = "0.0.0";
unsigned char HardwareRevStr[] = "0.0.0";
unsigned char SoftwareRevStr[] = "0.0.0";

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
// Global Server variables for decoding of INDOOR BIKE DATA RESISTANCE PARAMETERS
// --------------------------------------------------------------------------------
float wind_speed = 0;       // meters per second, resolution 0.001
float grade = 0;            // percentage, resolution 0.01
float crr = 0;              // Coefficient of rolling resistance, resolution 0.0001
float cw = 0;               // Wind resistance Kg/m, resolution 0.01;
// --------------------------------------------------------------------------------

#define TIME_SPAN 3000 // Time span in millis 1000 = 1 second
unsigned long TimeInterval = 0;

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb
  Serial.println("Feather nRF52840 MITM supporting: FTMS, CPS and CSC");
  Serial.println("------------------ Version 01 ---------------------");
#endif
  // Initialize Bluefruit with maximum connections as Peripheral = 1, Central = 1
  Bluefruit.begin(1, 1);
  Setup_Client_FTMS();
  Setup_Client_CPS();
  Setup_Client_CSC();
  Setup_Client_DIS();
  Client_Start_Scanning();
  
  while (Bluefruit.Scanner.isRunning()) { // do nothing else but scanning....
  }
  // wait enough time or go on when Client/Central is connected and all set!
  TimeInterval = millis() + TIME_SPAN; // ADD just enough delay
  while( TimeInterval > millis() || (!Trainer.IsConnected) ) {
    }

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
  Serial.println("Setting up the Server-side advertising payload(s)");
#endif
  server_startADV();
#ifdef DEBUG
  Serial.println("Server-side is FTMS, CPS and CSC advertising!");
#endif
  while (Bluefruit.Advertising.isRunning()) { // ONLY advertise!
  }
#ifdef DEBUG 
  Serial.println("Server-side stopped advertising! Paired to Zwift?");
#endif  
  TimeInterval = millis() + TIME_SPAN; // ADD just enough DELAY
  // wait enough time or go on when Server/Peripheral is connected and set!
  while ( (TimeInterval > millis()) || (!Laptop.IsConnected) ) { 
    }
  // Only now enable trainer to stream CPS, CSC and FTMS data...
  Client_Enable_Notify_Indicate(); 
#ifdef DEBUG   
  Serial.println("Client- and Server-side are Up and Running!");
#endif
}

// ----------------------   CLIENT SIDE FUNCTIONS  -------------------------
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
#ifdef DEBUG  
  Serial.println("FTM Service and Chars are 'initialized'");
#endif
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
#ifdef DEBUG  
  Serial.println("CP Service and Chars are 'initialized'");
#endif
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
#ifdef DEBUG
  Serial.println("CSC Service and Chars are 'initialized'");
#endif
}  

void Setup_Client_DIS(void)
{
  // Initialize client Generic Access Service
  client_GenericAccess_Service.begin();
  // Initialize some characteristics of the Generic Access Service.
  client_GA_DeviceName_Chr.begin();
  client_GA_Appearance_Chr.begin();  
#ifdef DEBUG
  Serial.println("Generic Access Service and Chars are 'initialized'");
#endif 
  // Initialize client Device Information Service
  client_DIS_Service.begin();
  // Initialize some characteristics of the Device Information Service.
  client_DIS_ManufacturerName_Chr.begin();
  client_DIS_ModelNumber_Chr.begin();
  client_DIS_SerialNumber_Chr.begin();
#ifdef DEBUG
  Serial.println("Device Information Service and Chars are 'initialized'");
#endif
}  

void loop()
{
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
  Bluefruit.Scanner.restartOnDisconnect(false); // default is true !!! this stage we do not want to RESTART
  Bluefruit.Scanner.filterRssi(-70);   // original value of -80 , we want to scan only nearby peripherals, so get close to your device !!
  Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms
  Bluefruit.Scanner.filterUuid(UUID16_SVC_CYCLING_POWER, 0x1826); // only invoke callback if one or more of these services are advertised 
  Bluefruit.Scanner.useActiveScan(true);
  Bluefruit.Scanner.start(0);  // 500 // 0 = Don't stop scanning or n = after n/100 seconds
#ifdef DEBUG
  Serial.println("Start Scanning for CPS and FTMS!");
  delay(100); // To show print message !
#endif
}

void Client_Enable_Notify_Indicate(void)
{
  // ---------------------  Enable CP and CSC Notify and Indicate ------------------------------------
  // Reaching here means we are ready to go, let's enable notification on measurement chr
  if ( client_CP_Measurement_Chr.enableNotify() ) {
#ifdef DEBUG
    Serial.println("Enabled 'Notify' for Client CP Measurement values");
#endif
  } else {
#ifdef DEBUG
    Serial.println("Couldn't enable 'Notify' for Client CP Measurement Characteristic. Increase DEBUG LEVEL for troubleshooting");
#endif
  }
  if ( client_CP_ControlPoint_Chr.enableIndicate() ) {
#ifdef DEBUG  
    Serial.println("Enabled 'Indicate' for Client CP ControlPoint Responses");
#endif
  } else {
#ifdef DEBUG  
    Serial.println("Couldn't enable 'Indicate' for Client CP ControlPoint Characteristic. Increase DEBUG LEVEL for troubleshooting");
#endif
  }
  if ( client_CSC_Measurement_Chr.enableNotify() ) {
#ifdef DEBUG
    Serial.println("Enabled 'Notify' for Client CSC Measurement values");
#endif
  } else  {
#ifdef DEBUG
    Serial.println("Couldn't enable notify for Client CSC Measurement Characteristic. Increase DEBUG LEVEL for troubleshooting");
#endif
  }
// -------------------------  Enable CP and CSC  Notify and Indicate --------------------------------
// -------------------------  Enable FTMS Notify and Indicate ---------------------------------------  
  if ( client_FTM_TrainingStatus_Chr.enableNotify() ) {
#ifdef DEBUG
    Serial.println("Enabled 'Notify' for Client FTM TrainingStatus values");
#endif
  } else {
#ifdef DEBUG
    Serial.println("Couldn't enable 'Notify' for Client FTM Training Status Characteristic. Increase DEBUG LEVEL for troubleshooting");
#endif
  }
  if ( client_FTM_IndoorBikeData_Chr.enableNotify() ) {
#ifdef DEBUG
    Serial.println("Enabled 'Notify' for Client FTM IndoorBikeData values");
#endif
  } else {
#ifdef DEBUG
    Serial.println("Couldn't enable 'Notify' for Client FTM Indoor Bike Data Characteristic. Increase DEBUG LEVEL for troubleshooting");
#endif
  }
  if ( client_FTM_ControlPoint_Chr.enableIndicate() ) {
#ifdef DEBUG 
    Serial.println("Enabled 'Indicate' for Client FTM ControlPoint Response Messages");
#endif
  } else {
#ifdef DEBUG
    Serial.println("Couldn't enable 'Indicate' for Client FTM Control Point Characteristic. Increase DEBUG LEVEL for troubleshooting");
#endif
  }
  if ( client_FTM_Status_Chr.enableNotify() ) {
#ifdef DEBUG
    Serial.println("Enabled 'Notify' for Client FTM Status values");
#endif
  } else {
#ifdef DEBUG
    Serial.println("Couldn't enable 'Notify' for Client FTM Status Characteristic. Increase DEBUG LEVEL for troubleshooting");
#endif
  }
// -------------------------  Enable FTMS Notify and Indicate ---------------------------------------  
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
  server_FTM_TrainingStatus_Chr.notify(data, len); // Just pass on and process later!
  
#ifdef DEBUG
  uint8_t SDataLen = (uint8_t)len;
  uint8_t SDataBuf[SDataLen] = {};
  // Transfer first the contents of data to buffer (array of chars)
  Serial.printf(" -> Client Raw FTM Training Status Data: [%d] [ ", len);
  for (int i = 0; i <  sizeof(SDataBuf); i++) {
      SDataBuf[i] = *data++;
      Serial.printf("%02X ", SDataBuf[i], HEX);
  }
  Serial.println("] ");
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
  // Only when DEBUG is defined IDBData will be parsed and printed!
  ParseIndoorBikeData(data, len);
#endif
}

#ifdef DEBUG_FTM_INDOORBIKEDATA
void ParseIndoorBikeData(uint8_t* data, uint16_t len)
{
// ---> IBD Buffer Data Length depends on data flagged to be present !!!!
  uint8_t IBDDataLen = (uint8_t)len;
  uint8_t IBDDataBuf[IBDDataLen] = {};
  // Transfer first the contents of data to buffer (array of chars)
  Serial.printf(" -> Client Raw FTM IBD Data: [%d] [ ", len);
  for (int i = 0; i < sizeof(IBDDataBuf); i++) {
      IBDDataBuf[i] = *data++;
      Serial.printf("%02X ", IBDDataBuf[i], HEX);
  }
  Serial.print("] ");

  uint8_t offset = 0;
  uint16_t flags = 0;
  memcpy(&flags, &IBDDataBuf[offset], 2); // Transfer buffer fields to variable
  offset += 2;  // UINT16
  if ((flags & 1) != 0) {
    //  More Data
    Serial.print(" More Data!");
  }
  if ((flags & 2) != 0) {
    //  Average Speed 0.01
    uint16_t sim_Speed = 0;
    memcpy(&sim_Speed, &IBDDataBuf[offset], 2); // Transfer buffer fields to variable
    Serial.printf(" Speed: %d KPH", (sim_Speed/100));
    offset += 2;  // UINT16 
    }
  if ((flags & 4) != 0) {
    //  Instantaneous Cadence 0.5
    uint16_t inst_Cadence = 0;
    memcpy(&inst_Cadence, &IBDDataBuf[offset], 2); // Transfer buffer fields to variable
    Serial.printf(" Instantaneous Cadence: %d RPM", (inst_Cadence/2));
    offset += 2;  // UINT16 
    }
  if ((flags & 8) != 0) {
    //  Average Cadence 0.5
    uint16_t av_Cadence = 0;
    memcpy(&av_Cadence, &IBDDataBuf[offset], 2); // Transfer buffer fields to variable
    Serial.printf(" Average Cadence: %d RPM", (av_Cadence/2));
    offset += 2;  // UINT16 
    }
  if ((flags & 16) != 0) {
    //  Total Distance  1
    uint8_t byte_LSO = (uint8_t)IBDDataBuf[offset];   // byte 0 LSO
    uint8_t byte_MID = (uint8_t)IBDDataBuf[offset+1]; // byte 1
    uint8_t byte_MSO = (uint8_t)IBDDataBuf[offset+2]; // byte 2 MSO
    // Convert little endian format and transfer to 32 bit variable
    uint32_t tot_Distance = ((byte_MSO << 24) | (byte_MID << 16) | (byte_LSO << 8)) >> 8;
    Serial.printf(" Total Distance: %d m", tot_Distance);
    offset += 3;  // UINT24 16 + 8
    }
  if ((flags & 32) != 0) {
    //  Resistance Level  1
    uint16_t res_Level = 0;
    memcpy(&res_Level, &IBDDataBuf[offset], 2); // Transfer buffer fields to variable
    Serial.printf(" Resistance Level: %d ", res_Level);
    offset += 2;  // UINT16 
    }
  if ((flags & 64) != 0) {
    //  Instantaneous Power 1
    uint16_t inst_Power = 0;
    memcpy(&inst_Power, &IBDDataBuf[offset], 2); // Transfer buffer fields to variable
    Serial.printf(" Instantaneous Power: %d Watt", inst_Power);
    offset += 2;  // UINT16 
    }
  if ((flags & 128) != 0) {
    //  Average Power 1
    uint16_t av_Power = 0;
    memcpy(&av_Power, &IBDDataBuf[offset], 2); // Transfer buffer fields to variable
    Serial.printf(" Average Power: %d Watt", av_Power);
    offset += 2;  // UINT16 
    }
  if ((flags & 256) != 0) {
    //  Expended Energy -> UINT16 UINT16 UINT8
    // Total Energy UINT16  1
    uint16_t tot_Energy = 0;
    memcpy(&tot_Energy, &IBDDataBuf[offset], 2); // Transfer buffer fields to variable
    Serial.printf(" Tot. Energy: %d kCal", tot_Energy);
    offset += 2;  // UINT16 
    // Energy per hour UINT16 1
    uint16_t Energy_hr = 0;
    memcpy(&Energy_hr, &IBDDataBuf[offset], 2); // Transfer buffer fields to variable
    Serial.printf(" Energy/hr: %d kCal/hr", Energy_hr);
    offset += 2;  // UINT16 
    // Energy per minute UINT8  1
    uint8_t Energy_pm = 0;
    memcpy(&Energy_pm, &IBDDataBuf[offset], 1); // Transfer buffer fields to variable
    Serial.printf(" Energy/m: %d kCal/m", Energy_pm);
    offset += 1;  // UINT8 
    }
  if ((flags & 512) != 0) {
    //  Heart Rate  1
    uint8_t Heart_Rate = 0;
    memcpy(&Heart_Rate, &IBDDataBuf[offset], 1); // Transfer buffer fields to variable
    Serial.printf(" Heart Rate: %d HBM", Heart_Rate);
    offset += 1;  // UINT8 
    }
  if ((flags & 1024) != 0) {
    //  Metabolic Equivalent 0.1
    uint8_t Mets = 0;
    memcpy(&Mets, &IBDDataBuf[offset], 1); // Transfer buffer fields to variable
    Serial.printf(" Metabolic Equivalent: %d ", Mets/10);
    offset += 1;  // UINT8 
    }
  if ((flags & 2048) != 0) {
    //  Elapsed Time  1
    uint16_t elap_time = 0;
    memcpy(&elap_time, &IBDDataBuf[offset], 2); // Transfer buffer fields to variable
    Serial.printf(" Elapsed time: %d s", elap_time);
    offset += 2;  // UINT16 
    }
  if ((flags & 4096) != 0) {
    //  Remaining Time  1
    uint16_t rem_time = 0;
    memcpy(&rem_time, &IBDDataBuf[offset], 2); // Transfer buffer fields to variable
    Serial.printf(" Remaining time: %d s", rem_time);
    offset += 2;  // UINT16 
    }
  Serial.println();
}
#endif

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
  server_FTM_ControlPoint_Chr.indicate(data, len); // Just pass on and process later!
  
#ifdef DEBUG
  uint8_t RespBufferLen = (uint8_t)len;
  uint8_t RespBuffer[RespBufferLen] = {}; // It is max 6 bytes long
  // Transfer first the contents of data to buffer (array of chars)
  Serial.print("Client Control Point Response: [ "); 
  for (int i = 0; i < sizeof(RespBuffer); i++) {
      RespBuffer[i] = *data++;
      Serial.printf("%02X ", RespBuffer[i], HEX);
  }
  Serial.println("] ");
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
  server_FTM_Status_Chr.notify(data, len); // Just pass on and process later!
  
#ifdef DEBUG
  uint8_t SDataLen = (uint8_t)len;
  uint8_t SDataBuf[SDataLen] = {};
  // Transfer first the contents of data to buffer (array of chars)
  Serial.printf(" -> Client Raw FTM Machine Status Data: [%d] [ ", len);
  for (int i = 0; i < sizeof(SDataBuf); i++) {
      SDataBuf[i] = *data++;
      Serial.printf("%02X ", SDataBuf[i], HEX);
  }
  Serial.println("] ");
#endif
}

/**
 * Callback invoked when scanner pick up an advertising data
 * @param report Structural advertising data
 */
void client_scan_callback(ble_gap_evt_adv_report_t* report)
{
  // Since we configure the scanner with filterUuid()
  // Scan callback only is invoked for device with CPS service advertised
  // Connect to device with client services in advertising
#ifdef DEBUG  
  Serial.println("Advertising Client with FTM, CPS or CSC is found! ... Raw data packet:");
  Serial.println(F("Timestamp Addr              Rssi Data"));
  Serial.printf("%09d ", millis());
  // MAC is in little endian --> print reverse
  Serial.printBufferReverse(report->peer_addr.addr, 6, ':');
  Serial.print(F(" "));
  Serial.print(report->rssi);
  Serial.print(F("  "));
  Serial.printBuffer(report->data.p_data, report->data.len, '-');
  Serial.println();
#endif
  Bluefruit.Central.connect(report);
}


#ifdef DEBUG
void PrintPeerAddress(uint8_t addr[6])
{
  for (int i = 1; i < 6; i++) {
      // Display byte by byte in HEX reverse: little Endian
      Serial.printf("%02X:",addr[(6-i)], HEX);
  }
   Serial.printf("%02X ",addr[0], HEX);
}
#endif

/**
 * Callback invoked when an connection is established
 * @param conn_handle
 */
void client_connect_callback(uint16_t conn_handle)
{
  Trainer.conn_handle = conn_handle;
  Trainer.IsConnected = true;
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);
  connection->getPeerName(Trainer.PeerName, sizeof(Trainer.PeerName));
  ble_gap_addr_t PeerAddr = connection->getPeerAddr(); // Fill BLE Gap struct
  memcpy(Trainer.PeerAddress, PeerAddr.addr, 6); // Copy Peer Address from struct
#ifdef DEBUG
  Serial.printf("Feather nRF52 (Central) connected to Trainer (Peripheral) device: [%s] MAC Address: ", Trainer.PeerName);
  PrintPeerAddress(Trainer.PeerAddress);
  Serial.println();
#endif 

#ifdef DEBUG
  Serial.println("Now checking all mandatory Client Services and Characteristics!");
  Serial.print("Discovering Client Cycling Power (CP) Service ... ");
#endif
  // If CPS is not found, disconnect, resume scanning, and return
  if ( client_CyclingPower_Service.discover(conn_handle) )
  {
#ifdef DEBUG
    BLEConnection* conn = Bluefruit.Connection(conn_handle);
    uint16_t max_payload = conn->getMtu()-3;
    uint16_t data_length = conn->getDataLength();
    Serial.print("Found it! ");
    Serial.printf("CPS Max Payload: %d Data Length: %d\n", max_payload, data_length);
#endif
  } else {
#ifdef DEBUG
    Serial.println("Not Found!"); 
    Serial.println("Disconnecting since Client Cyling Power Service is mandatory!");
#endif
    // MANDATORY so disconnect   
    Bluefruit.disconnect(conn_handle);
    return;
  }
#ifdef DEBUG 
  Serial.print("Discovering Client CP Measurement characteristic ... ");
#endif
  if ( !client_CP_Measurement_Chr.discover() )
  {
    // Measurement chr is mandatory, if it is not found (valid), then disconnect
#ifdef DEBUG
    Serial.println("Not Found! Disconnecting since Client CP Measurement Characteristic is mandatory!");
#endif
    // MANDATORY so disconnect  
    Bluefruit.disconnect(conn_handle);
    return;
  } else {
#ifdef DEBUG
  Serial.println("Found it!");
#endif 
  } 
#ifdef DEBUG
  Serial.print("Discovering Client CP Control Point characteristic ... ");
#endif
  if ( client_CP_ControlPoint_Chr.discover() )
  {
    // CP Control Point chr is not mandatory
#ifdef DEBUG
    Serial.println("Found it!");  
#endif
  } else {
#ifdef DEBUG
  Serial.println("Not Found! Client CP Control Point characteristic is NOT mandatory!");  
#endif
  }
#ifdef DEBUG  
  Serial.print("Discovering Client CP Feature characteristic ... ");
#endif
  if ( client_CP_Feature_Chr.discover() )
  {
#ifdef DEBUG
    Serial.print("Found it!");
#endif
  // Configure the Cycle Power Feature characteristic
  // Read 32-bit client_CP_Feature_Chr value
  client_CP_Feature_Flags = client_CP_Feature_Chr.read32();
#ifdef DEBUG
  const uint8_t CPFC_FIXED_DATALEN = 4;
  uint8_t cpfcData[CPFC_FIXED_DATALEN] = {(uint8_t)(client_CP_Feature_Flags & 0xff), (uint8_t)(client_CP_Feature_Flags >> 8), 
                                          (uint8_t)(client_CP_Feature_Flags >> 16), (uint8_t)(client_CP_Feature_Flags >> 24)};
  Serial.print(" -> Client Raw CP Feature bytes: [4] [ ");
  for (int i = 0; i < CPFC_FIXED_DATALEN; i++) {
    if ( i <= sizeof(cpfcData)) {
      Serial.printf("%02X ", cpfcData[i], HEX);
    }
  }
  Serial.println("] ");
  for (int i = 0; i < sizeof(client_CP_Feature_Str); i++) {
    if ( client_CP_Feature_Flags & (1 << i) )
      {
       Serial.println(client_CP_Feature_Str[i]);
      }
    }
#endif
  } else {
#ifdef DEBUG
    Serial.println("NOT Found! Client CP Feature characteristic is NOT Mandatory!");
#endif
  }
#ifdef DEBUG
  Serial.print("Discovering Client CP Sensor Location characteristic ... ");
#endif
  if ( client_CP_Location_Chr.discover() )
  {
#ifdef DEBUG
    Serial.print("Found it!");
#endif
  // The Sensor Location characteristic
  // Read 8-bit client CP sensor location value
    client_CP_Location_Value = client_CP_Location_Chr.read8();   
#ifdef DEBUG
    Serial.print(" -> Client CP Location Sensor: ");
    Serial.printf("Loc#: %d %s\n", client_CP_Location_Value, client_Sensor_Location_Str[client_CP_Location_Value]);
#endif
  } else {
#ifdef DEBUG
    Serial.println("NOT Found! Client CP Sensor Location characteristic is NOT mandatory!");
#endif
  }
#ifdef DEBUG  
  Serial.print("Discovering Cycling Speed and Cadence (CSC) Service ... ");
#endif
  if ( client_CyclingSpeedCadence_Service.discover(conn_handle) ) //   UUID16_SVC_CYCLING_SPEED_AND_CADENCE
  {
#ifdef DEBUG
    BLEConnection* conn = Bluefruit.Connection(conn_handle);
    uint16_t max_payload = conn->getMtu()-3;
    uint16_t data_length = conn->getDataLength();
    Serial.print("Found it! ");
    Serial.printf("CSCS Max Payload: %d Data Length: %d\n", max_payload, data_length);
#endif
  } else {
#ifdef DEBUG
    Serial.println("Not Found! Disconnecting since CSC Service is mandatory!");
#endif
    // MANDATORY so disconnect  
    Bluefruit.disconnect(conn_handle);
    return;
  }
  // Test for client CSC Characteristics when the client CSC Service is existing
#ifdef DEBUG
  Serial.print("Discovering Client CSC Measurement CHR ... ");
#endif
  if ( !client_CSC_Measurement_Chr.discover() ) //   UUID16_CHR_CSC_MEASUREMENT
  {
#ifdef DEBUG
    Serial.println("Not Found! Disconnecting since Client CSC Measurement CHR is mandatory!");
#endif
    // MANDATORY so disconnect  
    Bluefruit.disconnect(conn_handle);
    return;
  }
#ifdef DEBUG
  Serial.println("Found it!");
#endif  

#ifdef DEBUG
  Serial.print("Discovering Client CSC Location CHR ... ");
#endif
  if ( client_CSC_Location_Chr.discover() ) //   UUID16_CHR_SENSOR_LOCATION
  {
#ifdef DEBUG 
    Serial.print("Found it!");
#endif
    // Read 16-bit client CSC sensor location value
    client_CSC_Location_Value = client_CSC_Location_Chr.read8();
#ifdef DEBUG
    Serial.print(" -> Client CSC Location Sensor: ");
    Serial.printf("Loc#: %d %s\n", client_CSC_Location_Value, client_Sensor_Location_Str[client_CSC_Location_Value]);
#endif
  } else {
#ifdef DEBUG
      Serial.println("Not Found! Client CSC Location Characteristic is NOT mandatory!");
#endif
    }
#ifdef DEBUG
  Serial.print("Discovering Client CSC Feature CHR ... ");   
#endif
  if ( client_CSC_Feature_Chr.discover() ) //   UUID16_CHR_CSC_FEATURE
  {
#ifdef DEBUG
    Serial.print("Found it!");
#endif
    // Read sensor CSC Feature value in 16 bit
    client_CSC_Feature_Flags = client_CSC_Feature_Chr.read16();
#ifdef DEBUG
    uint8_t cscfcData[CSC_FEATURE_FIXED_DATALEN] = { (uint8_t)(client_CSC_Feature_Flags & 0xff), (uint8_t)(client_CSC_Feature_Flags >> 8) }; //  Little Endian Representation
    Serial.printf(" -> Client Raw CSC Feature bytes: [2] [ ");
    for (int i = 0; i < sizeof(cscfcData); i++) {
      Serial.printf("%02X ", cscfcData[i], HEX);
    }
    Serial.println("] ");
    for (int i = 0; i < sizeof(client_CSC_Feature_Str); i++) {
    if ( (client_CSC_Feature_Flags & (1 << i)) != 0 )
      {
       Serial.println(client_CSC_Feature_Str[i]);
      }
    }
#endif
  } else {
#ifdef DEBUG
      Serial.println("Not Found! Client CSC Feature Characteristic is NOT mandatory!");
#endif
  }
  // If Generic Access is not found then go on.... NOT FATAL !
  if ( client_GenericAccess_Service.discover(conn_handle) ) {
#ifdef DEBUG
      Serial.print(F("Found Client Generic Access\n"));
#endif
      if ( client_GA_DeviceName_Chr.discover() ) {
         client_GA_DeviceName_Chr.read(client_GA_DeviceName_Data, sizeof(client_GA_DeviceName_Data));
#ifdef DEBUG
        Serial.printf("Found Device Name: [%s]\n", client_GA_DeviceName_Data);
#endif
      }     
      if ( client_GA_Appearance_Chr.discover() ) {
         client_GA_Appearance_Value = client_GA_Appearance_Chr.read16();
#ifdef DEBUG
         Serial.printf("Found Appearance: [%04X]\n", client_GA_Appearance_Value);
#endif
      }     
  }
  // If DIS is not found then go on.... NOT FATAL !
  if ( client_DIS_Service.discover(conn_handle) ) {
#ifdef DEBUG
    Serial.print(F("Found Client Device Information: \n"));
#endif
    //  1
    if ( client_DIS_ManufacturerName_Chr.discover() ) {
      // read and print out Manufacturer
        if ( client_DIS_ManufacturerName_Chr.read(client_DIS_Manufacturer_Str, sizeof(client_DIS_Manufacturer_Str)) ) {
#ifdef DEBUG
        Serial.print("Client Manufacturer:  "); Serial.println(client_DIS_Manufacturer_Str);
#endif
      }
    }
    //  2
    if ( client_DIS_ModelNumber_Chr.discover() ) {
      // read and print out Model Number
      if ( client_DIS_ModelNumber_Chr.read(client_DIS_ModelNumber_Str, sizeof(client_DIS_ModelNumber_Str)) ) { 
#ifdef DEBUG
        Serial.print("Client Model Number:  "); Serial.println(client_DIS_ModelNumber_Str);
#endif
      }
    }
    //  3
    if ( client_DIS_SerialNumber_Chr.discover() ) {
      // read and print out Serial Number
      if ( client_DIS_SerialNumber_Chr.read(client_DIS_SerialNumber_Str, sizeof(client_DIS_SerialNumber_Str)) ) {
#ifdef DEBUG
          Serial.print("Client Serial Number: "); Serial.println(client_DIS_SerialNumber_Str);
#endif
      }
    }
  }
#ifdef DEBUG
  Serial.print("Discovering Client Fitness Machine (FTM) Service ... ");
#endif
  // If FTM is not found, disconnect, resume scanning, and return
  if ( client_FitnessMachine_Service.discover(conn_handle) )
  {
#ifdef DEBUG
    BLEConnection* conn = Bluefruit.Connection(conn_handle);
    uint16_t max_payload = conn->getMtu()-3;
    uint16_t data_length = conn->getDataLength();
    Serial.print("Found it! ");
    Serial.printf("FTMS Max Payload: %d Data Length: %d\n", max_payload, data_length);
#endif
  } else {
#ifdef DEBUG
    Serial.println("Not Found! Disconnecting since Client FTM Service is mandatory!");
#endif
    // MANDATORY so disconnect   
    Bluefruit.disconnect(conn_handle);
    return;
  }
#ifdef DEBUG
  Serial.print("Discovering Client FTM Feature Characteristic ... ");
#endif
  if ( client_FTM_Feature_Chr.discover() )
  {
#ifdef DEBUG
    Serial.print("Found it!");
#endif
    // Read FTM Feature Data
    client_FTM_Feature_Chr.read(client_FTM_Feature_Data, 8);
#ifdef DEBUG
    Serial.print(" -> Client Raw FTM Feature bytes: [8] [ ");
    for (int i = 0; i < sizeof(client_FTM_Feature_Data); i++) {
        Serial.printf("%02X ", client_FTM_Feature_Data[i], HEX);
    } // for
    Serial.println("] ");
#endif
  } else {
#ifdef DEBUG
    Serial.println("Not Found! Disconnecting since Client FTM Feature Characteristic is mandatory!");
#endif
    // MANDATORY so disconnect   
    Bluefruit.disconnect(conn_handle);
    return;
  }
#ifdef DEBUG
  Serial.print("Discovering Client FTM Training Status Characteristic ... ");
#endif
  if ( client_FTM_TrainingStatus_Chr.discover() )
  {
#ifdef DEBUG
    Serial.println("Found it!");
#endif
  } else {
#ifdef DEBUG
    Serial.println("Not Found! Client FTM Training Status Characteristic is NOT mandatory!");
#endif
  }
#ifdef DEBUG
  Serial.print("Discovering Client FTM Supported Resistance Level Range Characteristic ... ");
#endif
  if ( client_FTM_SupportedResistanceLevelRange_Chr.discover() )
  {
#ifdef DEBUG
    Serial.print("Found it!");
#endif
    // Read Supported Resistance Level Range Data
    client_FTM_SupportedResistanceLevelRange_Chr.read(client_FTM_SupportedResistanceLevelRange_Data, 6);
#ifdef DEBUG
    Serial.print(" -> Client Raw FTM Supported Resistance Level Range bytes: [6] [ ");
    for (int i = 0; i < sizeof(client_FTM_SupportedResistanceLevelRange_Data); i++) {
        Serial.printf("%02X ", client_FTM_SupportedResistanceLevelRange_Data[i], HEX);
    } // for
    Serial.println("] ");
#endif
  } else {
#ifdef DEBUG 
    Serial.println("NOT Found! Disconnecting since Client FTM Supported Resistance Level Range Characteristic is mandatory!");
#endif
    // MANDATORY so disconnect   
    Bluefruit.disconnect(conn_handle);
  }
#ifdef DEBUG
  Serial.print("Discovering Client FTM Supported Power Range Characteristic ... ");
#endif
  if ( client_FTM_SupportedPowerRange_Chr.discover() )
  {
#ifdef DEBUG
    Serial.print("Found it!");
#endif
    // Read Supported Resistance Level Range values
    client_FTM_SupportedPowerRange_Chr.read(client_FTM_SupportedPowerRange_Data, 6);
#ifdef DEBUG
    Serial.print(" -> Client Raw FTM Supported Power Range bytes: [6] [ ");
    for (int i = 0; i < sizeof(client_FTM_SupportedPowerRange_Data); i++) {
        Serial.printf("%02X ", client_FTM_SupportedPowerRange_Data[i], HEX);
    } // for
    Serial.println("] ");
#endif
  } else {
#ifdef DEBUG
//    Serial.println("Not Found! Client FTM Supported Power Range Characteristic is NOT mandatory!"); 
    Serial.println("Not Found! Disconnecting since Client FTM Supported Power Range Characteristic is mandatory!");
#endif
    // MANDATORY so disconnect   
    Bluefruit.disconnect(conn_handle);
    return;
  }
#ifdef DEBUG
  Serial.print("Discovering Client FTM Indoor Bike Data Characteristic ... ");
#endif
  // If FTM Indoor Bike Data is not found, disconnect, resume scanning, and return
  if ( client_FTM_IndoorBikeData_Chr.discover() )
  {
#ifdef DEBUG
    Serial.println("Found it!");
#endif
  } else {
#ifdef DEBUG
    Serial.println("Not Found! Client FTM Indoor Bike Data Characteristic is NOT mandatory!");
//    Serial.println("Not Found! Disconnecting since Client FTM Indoor Bike Data Characteristic is mandatory!");
#endif
/*
    // MANDATORY so disconnect   
    Bluefruit.disconnect(conn_handle);
    return;
*/
  }
#ifdef DEBUG
  Serial.print("Discovering Client FTM Control Point Characteristic ... ");
#endif
  if ( client_FTM_ControlPoint_Chr.discover() )
  {
#ifdef DEBUG
    Serial.println("Found it!");
#endif
  } else {
#ifdef DEBUG
    Serial.println("Not Found! Disconnecting since Client FTM Control Point Characteristic is mandatory!");
#endif
    // MANDATORY so disconnect   
    Bluefruit.disconnect(conn_handle);
    return;
  }
#ifdef DEBUG
  Serial.print("Discovering Client FTM Status Characteristic ... ");
#endif
  // If FTM Status is not found, disconnect, resume scanning, and return
  if ( client_FTM_Status_Chr.discover() )
  {
#ifdef DEBUG
    Serial.println("Found it!");
#endif
  } else {
#ifdef DEBUG
    Serial.println("Not Found! Disconnecting since Client FTM Status Characteristic is mandatory!");
#endif
    // MANDATORY so disconnect   
    Bluefruit.disconnect(conn_handle);
    return;
  }
} // End client_connect_callback

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void client_disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
#ifdef DEBUG
  Serial.printf("Client disconnected from Peripheral Device, reason: [%02X]\n", reason, HEX);
#endif
  Trainer.conn_handle = BLE_CONN_HANDLE_INVALID;
  Trainer.IsConnected = false;
  // Force server to disconnect!
  if (Laptop.conn_handle != BLE_CONN_HANDLE_INVALID) {
    Bluefruit.disconnect(Laptop.conn_handle);
  }
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
  Serial.printf(" -> Client Raw CP Data: [%d] [ ", len); 
  for (int i = 0; i < sizeof(buffer); i++) {
    if ( i <= sizeof(buffer)) {
      buffer[i] = *data++;
      Serial.printf("%02X ", buffer[i], HEX);
    }
  }
  Serial.print("] ");
  uint8_t offset = 0;
  // Get flags field
  uint16_t flags = 0;
  memcpy(&flags, &buffer[offset], 2); // Transfer buffer fields to variable
  offset += 2;  // UINT16
  // Get Instantaneous Power values UINT16
  uint16_t PowerValue = 0;
  memcpy(&PowerValue, &buffer[offset], 2); // Transfer buffer fields to variable
  offset += 2;  // UINT16
  Serial.printf("Instantaneous Power: %4d\n", PowerValue);
  // Get the other CP measurement values
  if ((flags & 1) != 0) {
    //  Power Balance Present
    Serial.print(" --> Pedal Power Balance!");
  }
  if ((flags & 2) != 0) {
    // Accumulated Torque
    Serial.println(" --> Accumulated Torque!");
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
  server_CP_ControlPoint_Chr.indicate(data, len); // Just pass on and process later!
  
#ifdef DEBUG
  uint8_t cpcpDataLen = (uint8_t)len;
  uint8_t cpcpData[cpcpDataLen]= {}; 
  // Transfer first the contents of data to buffer (array of chars)
  Serial.printf(" -> Client Raw CP Control Point Data: [%d] [ ", len); 
  for (int i = 0; i < sizeof(cpcpData); i++) {
      cpcpData[i] = *data++;
      Serial.printf("%02X ", cpcpData[i], HEX);
  }
  Serial.print("] ");
#endif
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
  Serial.printf(" -> Client Raw CSC Data: [%d] [ ", len); 
  for (int i = 0; i < sizeof(buffer); i++) {
    if ( i <= sizeof(buffer)) {
      buffer[i] = *data++;
      Serial.printf("%02X ", buffer[i], HEX);
    }
  }
  Serial.print("] ");
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
    Serial.printf(" Cum. wheel rev.: %d Last wheel event: %d ", cum_wheel_rev, last_wheel_event);
  }
  // we have to check the flags' nth bit to see if C2 field exists 
  if ((flags & 2) != 0) {
    uint16_t cum_cranks = 0;
    memcpy(&cum_cranks, &buffer[offset], 2);
    offset += 2; // UINT16
    uint16_t last_crank_event = 0;
    memcpy(&last_crank_event, &buffer[offset], 2);
    offset += 2; // UINT16
    Serial.printf(" Cum cranks: %d Last crank event: %d", cum_cranks, last_crank_event);
  }
  Serial.println();
#endif
}
// ----------------------   END of CLIENT SIDE FUNCTIONS   -------------------------  
// ----------------------  START of SERVER SIDE FUNCTIONS  ------------------------- 

void append_Dev_Name(void)
{
  size_t len = strlen((const char*)client_GA_DeviceName_Data); // Len of null terminated char array
  const char extension[] = {' ','S','I','M','\0'};
  if ( len > (MAX_PAYLOAD-sizeof(extension)) ) { // check for char array size
    len = (MAX_PAYLOAD-sizeof(extension));
  }
  memcpy(&client_GA_DeviceName_Data[len], &extension, sizeof(extension)); // Append to end of DevName
}
 
void server_startADV(void)
{
// Setup and start advertising
  // Supported tx_power values depending on mcu:
  // - nRF52832: -40dBm, -20dBm, -16dBm, -12dBm, -8dBm, -4dBm, 0dBm, +3dBm and +4dBm.
  // - nRF52840: -40dBm, -20dBm, -16dBm, -12dBm, -8dBm, -4dBm, 0dBm, +2dBm, +3dBm, +4dBm, +5dBm, +6dBm, +7dBm and +8dBm.
  Bluefruit.setTxPower(4); // See above for supported values: +4dBm
  // Set blink rate in advertising mode
  Bluefruit.setConnLedInterval(250);
  append_Dev_Name();
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
   * - Enable auto advertising Yes/NO if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
  */
  Bluefruit.Advertising.restartOnDisconnect(false); // false --> at this stage we do NOT want to auto RESTART
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
  server_FTM_IndoorBikeData_Chr.setProperties(CHR_PROPS_NOTIFY);  // because type "notify"
  server_FTM_IndoorBikeData_Chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS); // readAccess, writeAccess
  server_FTM_IndoorBikeData_Chr.setMaxLen(MAX_PAYLOAD); // To be on the safe side, when many features are set! 
  server_FTM_IndoorBikeData_Chr.setCccdWriteCallback(server_cccd_callback);  // Optionally capture CCCD updates
  server_FTM_IndoorBikeData_Chr.begin();

  // Training Status, optional, read & notify
  server_FTM_TrainingStatus_Chr.setProperties(CHR_PROPS_NOTIFY);  // because type is "notify"
  server_FTM_TrainingStatus_Chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS); // readAccess, writeAccess
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
  server_FTM_ControlPoint_Chr.setProperties(CHR_PROPS_INDICATE | CHR_PROPS_WRITE); // CHR_PROPS_READ // | CHR_PROPS_WRITE_WO_RESP); // Write with No response !!
  server_FTM_ControlPoint_Chr.setPermission(SECMODE_OPEN, SECMODE_OPEN); // readAccess, writeAccess DO NOT SET: SECMODE_NO_ACCESS !
  server_FTM_ControlPoint_Chr.setMaxLen(FTM_CONTROL_POINT_DATALEN); // Maxlen of Client written data: (1) OpCode and (FTM_CONTROL_POINT_DATALEN-1) OCTETS
  server_FTM_ControlPoint_Chr.setWriteCallback(server_FTM_ControlPoint_Chr_callback);
  server_FTM_ControlPoint_Chr.begin();

  // Fitness Machine Status, mandatory, notify  BLENotify, 
  server_FTM_Status_Chr.setProperties(CHR_PROPS_NOTIFY);  // because type is "notify"
  server_FTM_Status_Chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS); // readAccess, writeAccess
  server_FTM_Status_Chr.setMaxLen(FTM_STATUS_DATALEN); // Notice that with the present Target Features, Machine Status we only use 2 bytes max!
  server_FTM_Status_Chr.setCccdWriteCallback(server_cccd_callback);  // Optionally capture CCCD updates
  server_FTM_Status_Chr.begin();
}

void server_FTM_ControlPoint_Chr_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  // Server FTM Control Point data is tranferred to the Client
  // NO TREATMENT OF COMMAND !!!
  client_FTM_ControlPoint_Chr.write_resp(data, len); // Just pass on and process later!

  uint8_t ftmcpDataLen = (uint8_t)len;
  memset(server_FTM_Control_Point_Data.bytes, 0, sizeof(server_FTM_Control_Point_Data.bytes));
  // Transfer the contents of data to server_FTM_Control_Point_Data.bytes
  for (int i = 0; i < ftmcpDataLen; i++) {
      server_FTM_Control_Point_Data.bytes[i] = *data++;
    }
/* Decodes an incoming Fitness Machine Control Point request */
#ifdef DEBUG
    Serial.printf(" --> Raw FTM Control Point Data [len: %d] ", ftmcpDataLen);
    Serial.printf("[OpCode: %02X] [Values: ", server_FTM_Control_Point_Data.values.OPCODE, HEX);
    for (int i=0; i<ftmcpDataLen; i++) { 
      Serial.printf("%02X ", server_FTM_Control_Point_Data.values.OCTETS[i], HEX); 
    }
    Serial.println("]");
#endif
  switch(server_FTM_Control_Point_Data.values.OPCODE) {
    case ftmcpRequestControl: {
#ifdef DEBUG
      Serial.println("Request Control of Machine!");
#endif
      break;
    }
    case ftmcpStartOrResume: {
#ifdef DEBUG
      Serial.println("Start or Resume Machine!");
#endif
      break;
    }
    case ftmcpStopOrPause: {
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
#ifdef DEBUG // Remember, if debugging with Zwift, that these values are divided by 2 if in Zwift 50% settings!
      Serial.print("Set Indoor Bike Simulation Parameters --> ");
      Serial.print("Wind speed (1000): ");  Serial.print(wind_speed);
      Serial.print(" | Grade (100): ");     Serial.print(grade);
      Serial.print(" | Crr (10000): ");     Serial.print(crr);
      Serial.print(" | Cw (100): ");        Serial.println(cw);
#endif
      break;
    }
    case ftmcpReset: {
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
  server_CylingPower_Service.begin();
  
  server_CP_Measurement_Chr.setProperties(CHR_PROPS_NOTIFY);  // type is "notify"
  server_CP_Measurement_Chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS); // readAccess, writeAccess
  server_CP_Measurement_Chr.setMaxLen(MAX_PAYLOAD); // Will work in most cases!
  server_CP_Measurement_Chr.setCccdWriteCallback(server_cccd_callback);  // Optionally capture CCCD updates
  server_CP_Measurement_Chr.begin();
  
  server_CP_ControlPoint_Chr.setProperties(CHR_PROPS_INDICATE | CHR_PROPS_WRITE); // Indicate and Write !!
  server_CP_ControlPoint_Chr.setPermission(SECMODE_OPEN, SECMODE_OPEN); // readAccess, writeAccess DO NOT SET: SECMODE_NO_ACCESS !
  server_CP_ControlPoint_Chr.setMaxLen(CP_CONTROL_POINT_DATALEN); // The charactersitic's data set varies in length
  server_CP_ControlPoint_Chr.setCccdWriteCallback(server_cccd_callback);  // Optionally capture CCCD updates
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
  
  server_CSC_Measurement_Chr.setProperties(CHR_PROPS_NOTIFY);  // because type "notify"
  server_CSC_Measurement_Chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS); // readAccess, writeAccess
  server_CSC_Measurement_Chr.setMaxLen(MAX_PAYLOAD);
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
  // Server CP Control Point data is transferred to the client (trainer)
  // NO TREATMENT OF COMMAND !!!
  // Transfer cpcp data from the Server (Zwift) to the Client (Trainer)
  client_CP_ControlPoint_Chr.write_resp(data, len); // Just pass on and process later!

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

void server_connect_callback(uint16_t conn_handle)
{
  Laptop.conn_handle = conn_handle;
  Laptop.IsConnected = true;
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);
  connection->getPeerName(Laptop.PeerName, sizeof(Laptop.PeerName));
  ble_gap_addr_t PeerAddr = connection->getPeerAddr(); // Fill BLE Gap struct
  memcpy(Laptop.PeerAddress, PeerAddr.addr, 6); // Copy Peer Address from struct
#ifdef DEBUG
  Serial.printf("Feather nRF52 (Peripheral) connected to Laptop/PC/Tablet (Central) device: [%s] MAC Address: ", Laptop.PeerName);
  PrintPeerAddress(Laptop.PeerAddress);
  Serial.println();
  Serial.println("Waiting for Central (Zwift) to set CCCD Notify/Indicate (enable) and start reading....");
#endif 
}

/*
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE
*/
void server_disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
#ifdef DEBUG
  Serial.printf("Server disconnected from Central Device, reason: [%02X]\n", reason, HEX);
#endif
  Laptop.conn_handle = BLE_CONN_HANDLE_INVALID;
  Laptop.IsConnected = false;
  // Force client to disconnect!
  if(Trainer.conn_handle != BLE_CONN_HANDLE_INVALID) {
    Bluefruit.disconnect(Trainer.conn_handle);
  }
}

void server_cccd_callback(uint16_t conn_handle, BLECharacteristic* chr, uint16_t cccd_value)
{
    // When changed, display the Notify/Indicate Status for all characteristics
#ifdef DEBUG
    Serial.printf("Central Device Updated CCCD to: [%d] --> ", cccd_value);
#endif
    // Check the characteristic UUID this CCCD callback is associated with,
    // in case this handler is used for multiple CCCD records.
    if (chr->uuid == server_CP_Measurement_Chr.uuid) { 
        if (chr->notifyEnabled(conn_handle)) { 
#ifdef DEBUG
          Serial.print("Server CP: Measurement 'Notify' enabled");
#endif
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
        } else {
#ifdef DEBUG
          Serial.print("Server FTM: TrainingStatus 'Notify' disabled");
#endif
        }
    }
    
    if (chr->uuid == server_FTM_Status_Chr.uuid) { 
        if (chr->notifyEnabled(conn_handle)) { 
#ifdef DEBUG
          Serial.print("Server FTM: Status 'Notify' enabled");
#endif
        } else {
#ifdef DEBUG
          Serial.print("Server FTM: Status 'Notify' disabled");
#endif
        }
    }
    
    if (chr->uuid == server_FTM_ControlPoint_Chr.uuid) { 
        if (chr->indicateEnabled(conn_handle)) { 
#ifdef DEBUG
            Serial.print("Server FTM: ControlPoint 'Indicate' enabled");
#endif
        } else {
#ifdef DEBUG
            Serial.print("Server FTM: ControlPoint 'Indicate' disabled");
#endif
        }
    }
   
#ifdef DEBUG
    Serial.println();
#endif
} // end Server CCCD callback
// ----------------------   END of SERVER SIDE FUNCTIONS  ------------------------- 
