/*********************************************************************
 This is programming code for ESP32 Espressif Wroom boards
 
      Tested with Adafruit Feather ESP32 V2 a.k.a. Huzzah
 
 The code uses heavily the supplied ESP32 NimBLE libraries !!          
    see: https://github.com/h2zero/NimBLE-Arduino NimBLE Version 2.0
 Many have invested time and resources providing open source code!
 
        MIT license, check LICENSE for more information
        All text must be included in any redistribution
*********************************************************************/

/* -----------------------------------------------------------------------------------------------------
               This code should work with all indoor cycling trainers that fully support,
                   Wahoo Cycling Power Service and Cycling Speed & Cadence Service
   ------------------------------------------------------------------------------------------------------

    The code links a BLE Server (a peripheral to Zwift) and a BLE Client (a central to the Wahoo) with a bridge
    in between, the ESP32 being man-in-the-middle (MITM).
    The ESP32-bridge can control, filter and alter the bi-directional interchanged data!
    The client-side (central) scans and connects with the trainer relevant services: CPS + Wahoo and CSC (optional).
    It collects all cyling data of the services and passes these on to the server-side....
    The client-side supplies the indoor trainer with target and resistance control data.
    The server-side (peripheral) advertises and enables connection with cycling apps like Zwift and collects the app's
    control commands, target and resistance data. It passes these on to the client-side....
    The server-side supplies the app with the generated cycling data in return.

    The client plus server (MITM) are transparent to the Wahoo trainer as well as to the training app Zwift or alike!

    Requirements: Zwift app or alike, ESP32 board and a CPS/CSC supporting Wahoo trainer
    1) Upload and Run this code on on the ESP32 board
    2) Start the Serial Monitor to catch debugging info
    3) Start/Power On the indoor trainer
    4) ESP32 board and trainer will pair as reported in the output
    5) Start Zwift on your computer or tablet and wait....
    6) Search on the Zwift pairing screens for the ESP32 a.k.a. <SIM32>
    7) Pair: Power Source, Cadence (optional) and Resistance one after another with <SIM32>
    8) Optionally one can pair as well devices for heartrate and/or steering (Sterzo)
    9) Start the first Zwift ride or any ride you wish
   10) Make Serial Monitor output window visible on top of the Zwift window
   11) Hop on the bike: do the work and feel resistance change with the road
   12) Inspect the info presented by Serial Monitor.....

    This device is identified with the name <SIM32>. You will see this only when connecting to Zwift on the 
    pairing screens! Notice: Zwift extends device names with additional numbers for identification!

*/
/*
Version 1.0
Changed Stack Depth values from 2048 to 4096 for Server Control Point Indicate (modified) and Write w Response
Version 1.1
Inserted check (boolean) on Write-Response out of synch...
Version 1.2
Changed device identification naming to a simpler scheme: SIM32 or SIM52 instead of <SIM DevName>
Server Characteristic values (read only) are now updated when a new client connection is established
NimBLE registerForNotify() has been deprecated and is replaced with subscribe() / unsubscribe()
Version 2.0
Revisited the code to improve its logic, robustness and working. Further adapted the code at 
numerous places to conform with NimBLE Version 2.0
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
//#define DEBUG_CP_MEASUREMENT    // If defined allows for parsing and decoding the Cycling Power Data
//#define DEBUG_CSC_MEASUREMENT   // If defined allows for parsing and decoding the Cycling Speed and Cadence Data
//#define DEBUG_WAHOO_CONTROLPOINT_RESPONSE     // If defined allows for parsing the Data
#define DEBUG_WAHOO_CONTROLPOINT_OPCODE_DATA  // If defined allows for parsing and decoding Data
#endif
// --------------------------------------------------------------------------------------------

#define 	BLE_APPEARANCE_GENERIC_CYCLING   1152

/******************************* O P T I O N A L ********************************** 
   To enable more meaningful text messages, please un-comment inside "nimconfig.h"
   (../arduino/libraries/NimBLE-Arduino/src/nimconfig.h) at the following entry:
***********************************************************************************
 ** @brief Un-comment to see NimBLE host return codes as text debug log messages.
 *  Uses approx. 7kB of flash memory.
 *
//#define CONFIG_NIMBLE_CPP_ENABLE_RETURN_CODE_TEXT
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

// Trainer Wahoo Device Address, in printed format: [00:01:02:03:04:05]
#define TRAINERADDRESS {0x05,0x04,0x03,0x02,0x01,0x00} // Little Endian format!!
// -----------------------------------------------------------------
// Initialize connectable device registration
Device_info_t Trainer    = {TRAINERADDRESS, "MyTrainer", BLE_HS_CONN_HANDLE_NONE, false};
Device_info_t Laptop     = { LAPTOPADDRESS, "MyLaptop" , BLE_HS_CONN_HANDLE_NONE, false};
Device_info_t Smartphone = {        {0x00}, "MyPhone"  , BLE_HS_CONN_HANDLE_NONE, false};
// ----------------------------------------------------------------------------------

// Client Generic Access --------------------------------------------------------------
#define UUID16_SVC_GENERIC_ACCESS                             NimBLEUUID((uint16_t)0x1800)
#define UUID16_CHR_DEVICE_NAME                                NimBLEUUID((uint16_t)0x2A00)
#define UUID16_CHR_APPEARANCE                                 NimBLEUUID((uint16_t)0x2A01)
#define UUID16_CHR_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS NimBLEUUID((uint16_t)0x2A04)
#define UUID16_CHR_CENTRAL_ADDRESS_RESOLUTION                 NimBLEUUID((uint16_t)0x2AA6)
NimBLERemoteService* pRemote_GenericAccess_Service;
NimBLERemoteCharacteristic* pRemote_GA_Appearance_Chr; // Read
uint16_t client_GA_Appearance_Value = BLE_APPEARANCE_GENERIC_CYCLING;  // Default decimal: 1152 -> Generic Cycling
NimBLERemoteCharacteristic* pRemote_GA_DeviceName_Chr;// Read, Write
std::string client_GA_DeviceName_Str = THISDEVICENAME;

// Client Service Device Information --------------------------------------------------
#define UUID16_SVC_DEVICE_INFORMATION                         NimBLEUUID((uint16_t)0x180A)
#define UUID16_CHR_MODEL_NUMBER_STRING                        NimBLEUUID((uint16_t)0x2A24)
#define UUID16_CHR_MANUFACTURER_NAME_STRING                   NimBLEUUID((uint16_t)0x2A29)
#define UUID16_CHR_SERIAL_NUMBER_STRING                       NimBLEUUID((uint16_t)0x2A25)
/*
#define UUID16_CHR_FIRMWARE_REVISION_STRING                   NimBLEUUID((uint16_t)0x2A26)
#define UUID16_CHR_HARDWARE_REVISION_STRING                   NimBLEUUID((uint16_t)0x2A27)
#define UUID16_CHR_SOFTWARE_REVISION_STRING                   NimBLEUUID((uint16_t)0x2A28)
*/
NimBLERemoteService*  pRemote_DeviceInformation_Service; 
NimBLERemoteCharacteristic* pRemote_DIS_ManufacturerName_Chr;   // Read
std::string client_DIS_Manufacturer_Str;
NimBLERemoteCharacteristic* pRemote_DIS_ModelNumber_Chr;        // Read
std::string client_DIS_ModelNumber_Str;
NimBLERemoteCharacteristic* pRemote_DIS_SerialNumber_Chr;       // Read
std::string client_DIS_SerialNumber_Str;
// -------------------------------------------------------------------------------------
NimBLEService *server_DeviceInformation_Service;
NimBLECharacteristic *server_DIS_ModelNumber_Chr;       // Read
NimBLECharacteristic *server_DIS_SerialNumber_Chr;      // Read
NimBLECharacteristic *server_DIS_ManufacturerName_Chr;  // Read
/*
NimBLECharacteristic *server_DIS_Firmware_Chr;          // Read
std::string client_DIS_Firmware_Str = "12345";
NimBLECharacteristic *server_DIS_Hardware_Chr;          // Read
std::string client_DIS_Hardware_Str = "12345";
NimBLECharacteristic *server_DIS_Software_Chr;          // Read
std::string client_DIS_Software_Str = "12345";
*/

//--------------------------------------------------------------------------------------

/* NORDIC UART SERVICE a.k.a. NUS
 * NUS Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 * NUS RXD    : 6E400002-B5A3-F393-E0A9-E50E24DCCA9E
 * NUS TXD    : 6E400003-B5A3-F393-E0A9-E50E24DCCA9E
 */
NimBLEUUID UUID_NUS_SERVICE("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
NimBLEUUID UUID_NUS_CHR_RXD("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
NimBLEUUID UUID_NUS_CHR_TXD("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");
NimBLEService *server_NordicUart_Service; 
NimBLECharacteristic *server_NUS_Rxd_Chr;        // Write No Response (Receiving Data)
NimBLECharacteristic *server_NUS_Txd_Chr;        // Read Notify (Sending Data)

/* Cycling Power Service --------------------- CLIENT ------------------------------------------
 * CP Service: 0x1818  
 * CP Characteristic: 0x2A63 (Measurement)    Mandatory
 * CP Characteristic: 0x2A65 (Feature)        Mandatory
 * CP Characteristic: 0x2A5D (Location)       Optional
 * CP Characteristic: 0x2A66 (Control Point)  Optional
 */
#define UUID16_SVC_CYCLING_POWER                              NimBLEUUID((uint16_t)0x1818)
#define UUID16_CHR_CYCLING_POWER_MEASUREMENT                  NimBLEUUID((uint16_t)0x2A63)
//#define UUID16_CHR_CYCLING_POWER_VECTOR                       NimBLEUUID((uint16_t)0x2A64)
#define UUID16_CHR_CYCLING_POWER_FEATURE                      NimBLEUUID((uint16_t)0x2A65)
//#define UUID16_CHR_CYCLING_POWER_CONTROL_POINT                NimBLEUUID((uint16_t)0x2A66)
#define UUID16_CHR_SENSOR_LOCATION                            NimBLEUUID((uint16_t)0x2A5D) // shared with CSC
NimBLERemoteService*        pRemote_CyclingPower_Service;
NimBLERemoteCharacteristic* pRemote_CP_Measurement_Chr;    // Notify, Read
NimBLERemoteCharacteristic* pRemote_CP_Feature_Chr;        // Read
uint32_t client_CP_Feature_Flags = 0;
NimBLERemoteCharacteristic* pRemote_CP_Location_Chr;       // Read
uint8_t client_CP_Location_Value = {0x0C};                       // --> rear wheel !
/* ---------------------------------------------------------------------------------------------------------------
 * Wahoo Proprietary Control Point Characteristic
 * ---------------------------------------------------------------------------------------------------------------*/
// Proprietary Wahoo Trainer Control Point Characteristic is part of the Cycling Power Service !
static NimBLEUUID UUID16_CHR_WAHOO_CONTROL_POINT("A026E005-0A7D-4AB3-97FA-F1500F9FEB8B");
static NimBLERemoteCharacteristic* pRemote_Wahoo_ControlPoint_Chr; //  Wahoo Control Point, optional, write & indicate

// -------------------------------------- SERVER -------------------------------------------------
NimBLEService        *server_CyclingPower_Service;
NimBLECharacteristic *server_CP_Measurement_Chr; //                          Notify, Read
NimBLECharacteristic *server_CP_Feature_Chr; //                              Read
NimBLECharacteristic *server_CP_Location_Chr; //                             Read
/*NimBLECharacteristic *server_CP_ControlPoint_Chr; // Indicate, Write*/
// Proprietary Wahoo Trainer Control Point Characteristic is part of the Cycling Power Service !
NimBLECharacteristic *server_Wahoo_ControlPoint_Chr; // Wahoo Control Point, optional, write & indicate
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
#define UUID16_SVC_CYCLING_SPEED_AND_CADENCE                  NimBLEUUID((uint16_t)0x1816)
//#define UUID16_CHR_CSC_CONTROL_POINT                          NimBLEUUID((uint16_t)0x2A55)
#define UUID16_CHR_CSC_MEASUREMENT                            NimBLEUUID((uint16_t)0x2A5B)
#define UUID16_CHR_CSC_FEATURE                                NimBLEUUID((uint16_t)0x2A5C)
//#define UUID16_CHR_SENSOR_LOCATION                            NimBLEUUID((uint16_t)0x2A5D) // shared with CP
NimBLERemoteService* pRemote_CyclingSpeedCadence_Service;
NimBLERemoteCharacteristic* pRemote_CSC_Measurement_Chr;         // Notify, Read
NimBLERemoteCharacteristic* pRemote_CSC_Feature_Chr;             // Read
const uint8_t CSC_FEATURE_FIXED_DATALEN = 2; // UINT16
uint16_t client_CSC_Feature_Flags = 0;
const uint8_t client_CSC_Feature_Len = 3;
const char* client_CSC_Feature_Str[client_CSC_Feature_Len] = {"Wheel rev supported", "Crank rev supported", "Multiple locations supported"};
NimBLERemoteCharacteristic* pRemote_CSC_Location_Chr;          // Read
uint8_t client_CSC_Location_Value = {0x0C};                    // Default --> rear wheel !
// ---------------------------------------------------------------------------------------
NimBLEService        *server_CyclingSpeedCadence_Service;  
NimBLECharacteristic *server_CSC_Measurement_Chr;               //     Notify, Read
NimBLECharacteristic *server_CSC_Feature_Chr;                   //     Read
NimBLECharacteristic *server_CSC_Location_Chr;                  //     Read
// ---------------------------------------------------------------------------------------

/**
 * Wahoo Trainer Proprietary Control Point opcodes in decimal 
 * 
 * LSO: uint8 Op Code
 * MSO: 0..18 octets Parameters
 */
const uint8_t unlock                     = 32;
const uint8_t setResistanceMode          = 64;
const uint8_t setStandardMode            = 65;
const uint8_t setErgMode                 = 66;
const uint8_t setSimMode                 = 67;
const uint8_t setSimCRR                  = 68;
const uint8_t setSimWindResistance       = 69;
const uint8_t setSimGrade                = 70;
const uint8_t setSimWindSpeed            = 71;
const uint8_t setWheelCircumference      = 72;
const uint8_t unlockCommand[3]           = {unlock, 0xEE, 0xFC}; // Unlock codes

/** 
 * The Wahoo Proprietary Control Point data type structure 
 * 
 */
const uint8_t WAHOO_CONTROL_POINT_DATALEN = 19; // Control point consists of 1 opcode (byte) and maximum 18 bytes as parameters
// This wahoocp_data_t structure represents the control point data. The first octet represents the opcode of the request
// followed by a parameter array of maximum 18 octects
typedef struct __attribute__( ( packed ) )
{
  uint8_t OPCODE;
  uint8_t OCTETS[(WAHOO_CONTROL_POINT_DATALEN-1)];
} wahoocp_data_t;

typedef union // The union type automatically maps the bytes member array to the wahoocp_data_t structure member values
{
  wahoocp_data_t values;
  uint8_t bytes[WAHOO_CONTROL_POINT_DATALEN];
} wahoocp_data_ut;

// Fitness Machine Control Point Data variable
wahoocp_data_ut server_Wahoo_Control_Point_Data;

// Wahoo Control Point: Response Buffers
const uint8_t WahooRespConfirm = 0x01; // Ok!
const uint8_t WahooRespUnknown = 0x02; // Unknown OpCode
const uint8_t WahooResponseCode= 0x00; // To be set later to the correct value
uint8_t WahooRespConfirmBuffer[2] = {WahooRespConfirm, WahooResponseCode};
uint8_t WahooRespUnknownBuffer[2] = {WahooRespUnknown, WahooResponseCode};

// --------------------------------------------------------------------------------------
NimBLEClient* pClient_Wahoo = nullptr;
const NimBLEAdvertisedDevice* trainerDevice = nullptr;
NimBLEScan* pNimBLEScan = nullptr;
NimBLEServer* pServer = nullptr;
NimBLEAdvertising *pAdvertising = nullptr;

// ControlPoint Write With Response xTask for transferring training App data to the trainer
std::string xTaskClientWriteWithResponseData; 
void static xTaskClientWriteWithResponse(void *parameter);
TaskHandle_t xTaskClientWriteWithResponseHandle = NULL;
boolean xTaskClientWriteWithResponseDataReady = false;
const BaseType_t xTaskCoreID0 = 0;

// These variables are handled in loop() to start sort of Callback functions
bool doClientConnectCall = false;
bool RestartScanningOnDisconnect = false;
bool DoCallClientEnable = false;
bool DoCallClientDisable = false;
bool hasConnectPassed = false;

// Values used to enable or disable notifications/indications
const bool indications = false;  //false as first argument to subscribe to indications instead of notifications
const bool notifications = true; //true as first argument to subscribe to notifications
// ---------------------------------------------------------------------------------

// Client Connect and Disconnect callbacks defined
class client_Connection_Callbacks:public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* pClient);
  void onConnectFail(NimBLEClient* pClient, int reason);
  void onDisconnect(NimBLEClient* pClient, int reason);
  bool onConnParamsUpdateRequest(NimBLEClient* pClient, ble_gap_upd_params *params);  
};

// Server Connect and Disconnect callbacks defined
class server_Connection_Callbacks:public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo);
  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason);
  void onMTUChange(uint16_t MTU, NimBLEConnInfo& connInfo);
};

// Handler class for Server Multi Characteristic action(s) defined
class CharacteristicCallbacks:public NimBLECharacteristicCallbacks {
/*  We only define onSubscribe !!!
    void onRead(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo);
    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo);  
    void onStatus(NimBLECharacteristic* pCharacteristic, Status status, int code);
*/    
    void onSubscribe(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo, uint16_t subValue);
};  

// Define CharacteristicCallbacks instance(s) globally to use for multiple Server Characteristics 
static CharacteristicCallbacks server_Multi_Chr_Callbacks;

void ConvertMacAddress(char *fullAddress, uint8_t addr[6], bool NativeFormat);
void server_setupGA(void);
void server_setupDIS(void);
void server_setupNUS(void);
void server_setupCSC(void);
void server_setupCPS(void);
void server_startADV(void);
void client_Start_Scanning(void);
bool ClientConnectServer(void);
// ---------------------------------------------------------------------------------

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
  while (!Serial) delay(10); 
  Serial.flush();
  delay(1000); // Give Serial I/O time to settle
#endif
  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("ESP32 NimBLE MITM supporting: CPS + Wahoo and CSC (optional)");
  DEBUG_PRINTLN("----------------------- Version 02.0 -----------------------");
  DEBUG_PRINTF("Device Name: %s with NimBLE Version 2.0\n", THISDEVICENAME);
  delay(200);
#ifdef CONFIG_NIMBLE_CPP_ENABLE_RETURN_CODE_TEXT
  DEBUG_PRINTLN("CONFIG_NIMBLE_CPP_ENABLE_RETURN_CODE_TEXT enabled!");
#endif
  NimBLEDevice::init(THISDEVICENAME); // Shortname    
  // Start the Server-side now!
  pServer = NimBLEDevice::createServer();
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
  // Start the GATT server. Required to be called after setup of all services and 
  // characteristics / descriptors for the NimBLE host to register them.
  DEBUG_PRINTLN("ESP32 Server is setup and started!"); 
  pServer->start();
  DEBUG_PRINTLN("Setting up the Server advertising payload(s)");
  server_startADV();  
    
  // Start the Client-side!
  client_Start_Scanning();
  while(pNimBLEScan->isScanning()) ; // Wait until scanning is finished
  if(doClientConnectCall) {
    doClientConnectCall = false;
    if(!ClientConnectServer()) {
      NimBLEDevice::deleteClient(pClient_Wahoo); // Delete client object and clear from list
      pClient_Wahoo = nullptr;   // Clear to null
    }
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
            server_DIS_ManufacturerName_Chr->setValue(client_DIS_Manufacturer_Str); // Transfer/Update the value to the server side
            DEBUG_PRINTF("-> Client Reads Manufacturer Name: [%s]\n", client_DIS_Manufacturer_Str.c_str());
          }            
      }     
      pRemote_DIS_ModelNumber_Chr = pRemote_DeviceInformation_Service->getCharacteristic(UUID16_CHR_MODEL_NUMBER_STRING);       
      if ( pRemote_DIS_ModelNumber_Chr != nullptr ) {
          if(pRemote_DIS_ModelNumber_Chr->canRead()) {
            client_DIS_ModelNumber_Str = pRemote_DIS_ModelNumber_Chr->readValue();
            server_DIS_ModelNumber_Chr->setValue(client_DIS_ModelNumber_Str); // Transfer/Update the value to the server side
            DEBUG_PRINTF("-> Client Reads Model Number:      [%s]\n", client_DIS_ModelNumber_Str.c_str());
          }
      }  
      pRemote_DIS_SerialNumber_Chr = pRemote_DeviceInformation_Service->getCharacteristic(UUID16_CHR_SERIAL_NUMBER_STRING);       
      if ( pRemote_DIS_SerialNumber_Chr != nullptr ) {
          if(pRemote_DIS_SerialNumber_Chr->canRead()) {
            client_DIS_SerialNumber_Str = pRemote_DIS_SerialNumber_Chr->readValue();
            server_DIS_SerialNumber_Chr->setValue(client_DIS_SerialNumber_Str); // Transfer/Update the value to the server side
            DEBUG_PRINTF("-> Client Reads Serial Number:     [%s]\n", client_DIS_SerialNumber_Str.c_str());
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
            int RespErr = ble_svc_gap_device_name_set((const char*)client_GA_DeviceName_Str.c_str()); // Transfer/Update the value to the server side    
            DEBUG_PRINTF("-> Client Reads Device Name:   [%s]\n", client_GA_DeviceName_Str.c_str());
          }            
      }     
      pRemote_GA_Appearance_Chr = pRemote_GenericAccess_Service->getCharacteristic(UUID16_CHR_APPEARANCE);       
      if ( pRemote_GA_Appearance_Chr != nullptr ) {
          if(pRemote_GA_Appearance_Chr->canRead()) {
            client_GA_Appearance_Value = pRemote_GA_Appearance_Chr->readValue<uint16_t>();
            int RespErr = ble_svc_gap_device_appearance_set(client_GA_Appearance_Value); // Transfer/Update the value to the server side
            DEBUG_PRINTF("-> Client Reads Appearance:    [%d]\n", client_GA_Appearance_Value);
          }
      }     
    return true;     
}

void client_CP_Measurement_Notify_Callback(NimBLERemoteCharacteristic* pNimBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) 
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
  DEBUG_PRINTF("-> Client Rec'd Raw Wahoo CP Data: [%d] [%d] [ ", isNotify, length); 
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

void client_Wahoo_ControlPoint_Indicate_Callback(NimBLERemoteCharacteristic* pNimBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify)
{
  // The receipt of Control Point settings is acknowledged by the trainer: handle it
  // Send Client's Response message to the Server
  // NO TREATMENT OF COMMAND !!!
  if((Laptop.IsConnected)) {  
    server_Wahoo_ControlPoint_Chr->setValue(pData, length);
    server_Wahoo_ControlPoint_Chr->indicate(); // Just pass on and process later!
  } 
#ifdef DEBUG_WAHOO_CONTROLPOINT_RESPONSE
  uint8_t RespBufferLen = (uint8_t)length;
  uint8_t RespBuffer[RespBufferLen] = {}; // It is max 6 bytes long
  // Transfer first the contents of data to buffer (array of chars)
  DEBUG_PRINT("-> Client Rec'd Raw Wahoo Control Point Response Data: [ "); 
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
    if(!pRemote_Wahoo_ControlPoint_Chr->canIndicate()) {
      DEBUG_PRINTLN("Mandatory Client_Wahoo_ControlPoint_Chr: Cannot Indicate!");
      return false; // Mandatory when service is present
    }

    // --------------------------------------------------------------------------------------------------------------
    // Start a CRITICAL task pRemote_FTM_ControlPoint_Chr write-with-response
    // Pin task to core 0 and high priority for best results!
    // Stack size after extensive testing -> 8192 (very stable!!)
    // Check first if xTaskClientWriteWithResponse does not exist
    if(xTaskClientWriteWithResponseHandle == NULL) { 
      xTaskCreatePinnedToCore(xTaskClientWriteWithResponse, "Write w Response", 8192, NULL, 24, &xTaskClientWriteWithResponseHandle, xTaskCoreID0);
      DEBUG_PRINTLN("Client_Wahoo_ControlPoint_Chr: xTask Write-With-Response created!");
    }    
    // ---------------------------------------------------------------------------------------------------------------

    pRemote_CP_Measurement_Chr = pRemote_CyclingPower_Service->getCharacteristic(UUID16_CHR_CYCLING_POWER_MEASUREMENT);
    if (pRemote_CP_Measurement_Chr == nullptr) {
      DEBUG_PRINTLN("Mandatory client_CP_Measurement_Chr: Not Found!");
      return false;
    }
    DEBUG_PRINTLN("Client_CP_Measurement_Chr: Found!");  
    if(!pRemote_CP_Measurement_Chr->canNotify()) {
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
      client_CP_Feature_Flags = pRemote_CP_Feature_Chr->readValue<uint32_t>();
      server_CP_Feature_Chr->setValue(client_CP_Feature_Flags); // Transfer/Update the value to the server side
#ifdef DEBUG
      const uint8_t CPFC_FIXED_DATALEN = 4;
      uint8_t cpfcData[CPFC_FIXED_DATALEN] = {(uint8_t)(client_CP_Feature_Flags & 0xff), (uint8_t)(client_CP_Feature_Flags >> 8), 
                                          (uint8_t)(client_CP_Feature_Flags >> 16), (uint8_t)(client_CP_Feature_Flags >> 24)};
      DEBUG_PRINT("-> Client Reads Raw CP Feature bytes: [4] [ ");
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
        client_CP_Location_Value = pRemote_CP_Location_Chr->readValue<uint8_t>();
        server_CP_Location_Chr->setValue(&client_CP_Location_Value, 1); // Transfer/Update the value to the server side
        // CP sensor location value is 8 bit
#ifdef DEBUG
        DEBUG_PRINT(" -> ESP32 Client Reads CP Location Sensor:");
        if(client_CP_Location_Value <= client_Sensor_Location_Str_Len) 
          DEBUG_PRINTF(" Loc#: %d %s\n", client_CP_Location_Value, client_Sensor_Location_Str[client_CP_Location_Value]);
        else 
          DEBUG_PRINTF(" Loc#: %d \n", client_CP_Location_Value); 
#endif
      }
    }
    return true;    
}

void client_CSC_Measurement_Notify_Callback(NimBLERemoteCharacteristic* pNimBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify)
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
  DEBUG_PRINTF("-> Client Rec'd Raw Wahoo CSC Data: [%d] [%d] [ ", isNotify, length); 
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
    // Obtain a reference to the remote service.
    pRemote_CyclingSpeedCadence_Service = pClient_Wahoo->getService(UUID16_SVC_CYCLING_SPEED_AND_CADENCE);
    if (pRemote_CyclingSpeedCadence_Service == nullptr) {
      DEBUG_PRINTLN("client_CyclingSpeedCadence_Service: Not Found! Not Mandatory");
      return true;
    }
    DEBUG_PRINTLN("Client_CyclingSpeedCadence_Service: Found!");
    pRemote_CSC_Measurement_Chr = pRemote_CyclingSpeedCadence_Service->getCharacteristic(UUID16_CHR_CSC_MEASUREMENT);
    if (pRemote_CSC_Measurement_Chr == nullptr) {
      DEBUG_PRINTLN("Mandatory client_CSC_Measurement_Chr: Not Found!");
      return false;
    }
    DEBUG_PRINTLN("Client_CSC_Measurement_Chr: Found!");  
    if(!pRemote_CSC_Measurement_Chr->canNotify()) {
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
        client_CSC_Feature_Flags = pRemote_CSC_Feature_Chr->readValue<uint16_t>();
        server_CSC_Feature_Chr->setValue(client_CSC_Feature_Flags); // Transfer/Update the value to the server side
#ifdef DEBUG
        uint8_t cscfcData[CSC_FEATURE_FIXED_DATALEN] = { (uint8_t)(client_CSC_Feature_Flags & 0xff), (uint8_t)(client_CSC_Feature_Flags >> 8) }; //  Little Endian Representation
        DEBUG_PRINTF("-> Client Reads Raw CSC Feature bytes: [2] [ ");
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
        client_CSC_Location_Value = pRemote_CSC_Location_Chr->readValue<uint8_t>(); 
        server_CSC_Location_Chr->setValue(&client_CSC_Location_Value, 1); // Transfer/Update the value to the server side
        // CSC sensor location value is 8 bit
#ifdef DEBUG
        DEBUG_PRINT(" -> ESP32 Client Reads CSC Location Sensor:");
        if(client_CSC_Location_Value <= client_Sensor_Location_Str_Len) 
          DEBUG_PRINTF(" Loc#: %d %s\n", client_CSC_Location_Value, client_Sensor_Location_Str[client_CSC_Location_Value]);
        else 
          DEBUG_PRINTF(" Loc#: %d \n", client_CSC_Location_Value); 
#endif
      }
    }
    return true;    
}

bool ClientConnectServer(void) {
    // Connect to the Wahoo BLE Server.
    // Handle first time connect AND a reconnect. One fixed Peripheral (trainer) to account for!
    if(pClient_Wahoo == nullptr) { // First time -> create new pClient_Wahoo and service database!
      pClient_Wahoo = NimBLEDevice::createClient(); 
      pClient_Wahoo->setClientCallbacks(new client_Connection_Callbacks());
      // First Time Connect to the FTMS Trainer (Server/Peripheral)
      hasConnectPassed = pClient_Wahoo->connect(trainerDevice, true);   // Delete attribute objects and Create service database
    } else if(pClient_Wahoo == NimBLEDevice::getDisconnectedClient()) { // Allow for a streamlined reconnect
          // Reconnect to the disconnected FTMS Trainer (Server/Peripheral)
          hasConnectPassed = pClient_Wahoo->connect(trainerDevice, false);  // Just refresh the service database 
      } 

    if(!hasConnectPassed) 
      return hasConnectPassed; // Connect failed!

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
  if( !client_CyclingSpeedCadence_Connect() ) {
    pClient_Wahoo->disconnect();
    return false;    
  }
  if( !client_CyclingPower_Connect() ) {
    pClient_Wahoo->disconnect();
    return false;    
  }
  // When the client/trainer is RECONNECTING we need to enable/indicate all Remote Client Char's again!
  if(Laptop.IsConnected) {
    //Do NOT(!) allow for any possible delay (Regularly this is handled in loop() with DoCallClientEnable = true)
    clientSubscribeToAll(true);
  }
  // ----------------------------------------------------------------------------------------------
  Trainer.IsConnected = true; 
return true;
};

/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class clientScanCallbacks: public NimBLEScanCallbacks  {
/*
 * Called for each advertising BLE server.
*/
  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) {
    // We have found a server device, now see if it contains the CPS service we are looking for.
    if (advertisedDevice->haveServiceUUID() && advertisedDevice->isAdvertisingService(UUID16_SVC_CYCLING_POWER)) {
      NimBLEAddress MyAddress = advertisedDevice->getAddress();
      uint8_t RemoteAddress[6] = {};
      memcpy(&RemoteAddress, MyAddress.getBase()->val, 6);
      DEBUG_PRINTLN("Found advertising Peripheral with CPS enabled! See data:");
      DEBUG_PRINTLN(advertisedDevice->toString().c_str());
      // OK Server has CPS service exposed, now check for right mac adress 
      if ( !(memcmp((void*)RemoteAddress, Trainer.PeerAddress, 6) == 0) ) {
        char fullMacAddress[18] = {}; //
        ConvertMacAddress(fullMacAddress, RemoteAddress, false); // true -> Native representation!
        DEBUG_PRINTF("Keep Scanning! Unknown Trainer Mac Address: [%s]\n", fullMacAddress);
        return;
      }      
      trainerDevice = advertisedDevice;
      doClientConnectCall = true;         // Work-around via loop() 
      NimBLEDevice::getScan()->stop();          
    } // Found our server
  } // onResult
}; // clientScanCallbacks

void client_Connection_Callbacks::onConnect(NimBLEClient* pClient) {
    Trainer.PeerName = trainerDevice->getName().c_str();
    Trainer.conn_handle = pClient_Wahoo->getConnHandle();
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
    //pClient_Wahoo->updateConnParams(pClient_Wahoo->getConnId(), 24, 48, 0, 400);
    //DEBUG_PRINTLN("Client Updates Connection Parameters -> Min Interval: [24] Max Interval: [48] Latency: [0] Supervision Timeout: [400]");     
};

void client_Connection_Callbacks::onConnectFail(NimBLEClient* pClient, int reason) {
#ifdef CONFIG_NIMBLE_CPP_ENABLE_RETURN_CODE_TEXT
    DEBUG_PRINTF("Connection failure -> [%d][%s]\n", reason, NimBLEUtils::returnCodeToString(reason));
#else
    DEBUG_PRINTLN("");
#endif
};

bool client_Connection_Callbacks::onConnParamsUpdateRequest(NimBLEClient* pClient, ble_gap_upd_params *params) {
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

void client_Connection_Callbacks::onDisconnect(NimBLEClient* pClient, int reason) {
    Trainer.IsConnected = false;
    Trainer.conn_handle = BLE_HS_CONN_HANDLE_NONE; 
    char fullMacAddress[18] = {}; //
    ConvertMacAddress(fullMacAddress, Trainer.PeerAddress, false); // true -> Native representation!
    DEBUG_PRINTF("Client Disconnected from Server device with Name: [%s] Mac Address: [%s]",  Trainer.PeerName.c_str(), fullMacAddress);
#ifdef CONFIG_NIMBLE_CPP_ENABLE_RETURN_CODE_TEXT
    DEBUG_PRINTF(" -> [%d][%s]\n", reason, NimBLEUtils::returnCodeToString(reason));
#else
    DEBUG_PRINTLN("");
#endif 
    RestartScanningOnDisconnect = true;
};

void client_Start_Scanning(void)
{
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  pNimBLEScan = NimBLEDevice::getScan();
  pNimBLEScan->setScanCallbacks(new clientScanCallbacks());
  pNimBLEScan->setInterval(1349);
  pNimBLEScan->setWindow(449);
  pNimBLEScan->setActiveScan(true);
   DEBUG_PRINTLN("Client Starts Scanning for Server Device (Wahoo) with CPS!");  
  //pNimBLEScan->start(5000, false); // Scan for 5 seconds only
  pNimBLEScan->start(0, false);
}

// Handler class for Server Multi Characteristic actions limited to onSubscribe
void CharacteristicCallbacks::onSubscribe(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo, uint16_t subValue) {
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
    // Optional: set the transmit power, default is 3db
    NimBLEDevice::setPower(9); // +9db 
    pAdvertising = NimBLEDevice::getAdvertising(); 
    pAdvertising->addServiceUUID(UUID16_SVC_CYCLING_POWER);
    pAdvertising->addServiceUUID(UUID16_SVC_CYCLING_SPEED_AND_CADENCE);
    pAdvertising->setAppearance(client_GA_Appearance_Value);
    DEBUG_PRINTF("Setting Appearance in Advertised data to [%d]\n", client_GA_Appearance_Value);
    pAdvertising->setName(THISDEVICENAME);
    DEBUG_PRINTF("Setting DeviceName in Advertised data to [%s]\n", THISDEVICENAME);
    //Add the transmission power level to the advertisement packet.
    pAdvertising->addTxPower();
    pAdvertising->enableScanResponse(true);
    pAdvertising->setMinInterval(32); // in 0.625ms units, 0 = use default.
    pAdvertising->setMaxInterval(244); // in 0.625ms units, 0 = use default.
    DEBUG_PRINTLN("Server is advertising: CPS + Wahoo"); 
    NimBLEDevice::startAdvertising();    
    // Start Advertising 
}

void server_Connection_Callbacks::onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) {
    // Get some connection parameters of the peer device.
    uint16_t serverConnectionHandle = connInfo.getConnHandle();
    uint16_t serverConnectionInterval = connInfo.getConnInterval();   // Connection interval   
    uint16_t serverConnectionLatency = connInfo.getConnLatency();     // Connection latency
    uint16_t serverConnectionSupTimeout = connInfo.getConnTimeout();  // Connection supervision timeout
    uint8_t RemoteAddress[6];
    memcpy(&RemoteAddress, connInfo.getIdAddress().getBase()->val, 6);
    uint8_t addressType = connInfo.getIdAddress().getBase()->type;
    char fullMacAddress[18] = {}; 
    ConvertMacAddress(fullMacAddress, RemoteAddress, false); // true -> Native format!
    NimBLEDevice::stopAdvertising();
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
      DEBUG_PRINTF("Central (%s/Zwift) has to set CPS/CSC/NUS CCCD Notify/Indicate (enable) and start....\n", Laptop.PeerName.c_str());
      DoCallClientEnable = true;
      return; // We are done here!
    }
    // [2] Smartphone is connecting
    Smartphone.conn_handle = serverConnectionHandle;
    Smartphone.IsConnected = true;
    memcpy(Smartphone.PeerAddress, RemoteAddress, 6);
    DEBUG_PRINTF("Central (%s/Simcline App) has to set NUS CCCD 'Notify' (enable) and start....\n", Smartphone.PeerName.c_str());
};

void server_Connection_Callbacks::onMTUChange(uint16_t MTU, NimBLEConnInfo& connInfo) {
    DEBUG_PRINTF("MTU updated: %u for connection ID: %u\n", MTU, connInfo.getConnHandle());            
};

void server_Connection_Callbacks::onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) {
    // Get some Disconnection parameters of the peer device.
    uint16_t serverConnectionHandle = connInfo.getConnHandle();
    uint8_t RemoteAddress[6] = {};
    memcpy(&RemoteAddress, connInfo.getIdAddress().getBase()->val, 6);
    char fullMacAddress[18] = {}; //
    ConvertMacAddress(fullMacAddress, RemoteAddress, false); // true -> Native format!
    if (Laptop.conn_handle == serverConnectionHandle ) { // Laptop/Desktop is disconnected
      Laptop.conn_handle = BLE_HS_CONN_HANDLE_NONE;
      Laptop.IsConnected = false;
      DEBUG_PRINTF("ESP32 Server disconnected from Central (%s) Conn handle: [%d] Mac Address: [%s]", Laptop.PeerName.c_str(), \
                        serverConnectionHandle, fullMacAddress);
#ifdef CONFIG_NIMBLE_CPP_ENABLE_RETURN_CODE_TEXT
      DEBUG_PRINTF(" -> [%d][%s]\n", reason, NimBLEUtils::returnCodeToString(reason));
#else
      DEBUG_PRINTLN("");
#endif
      if(Trainer.IsConnected) DoCallClientDisable = true; // Tell the client not to send data!
    }
    if (Smartphone.conn_handle == serverConnectionHandle ) { // Smartphone is disconnected
      Smartphone.conn_handle = BLE_HS_CONN_HANDLE_NONE;
      Smartphone.IsConnected = false;
      DEBUG_PRINTF("ESP32 Server disconnected from Central (%s) Conn handle: [%d] Mac Address: [%s]\n", Smartphone.PeerName.c_str(), \
                    serverConnectionHandle, fullMacAddress);
#ifdef CONFIG_NIMBLE_CPP_ENABLE_RETURN_CODE_TEXT
      DEBUG_PRINTF(" -> [%d][%s]\n", reason, NimBLEUtils::returnCodeToString(reason));
#else
      DEBUG_PRINTLN("");
#endif
    }
    DEBUG_PRINTLN(" --> ESP32 Server is advertising again!");
    // NimBLe does auto advertise after disconnect 
};

// -------------------------------------------------------------------------------------
/* Return Code = 0x06 --> BLE_HS_ENOMEM --> Operation failed due to resource exhaustion.
* This issue occurs when we try to send too many packets before controller can process them. In this case, the packets get queued. 
* As we queue more number of packets, we reach a point where we run out of memory. In this case, BLE_HS_ENOMEM is returned.
*/
void xTaskClientWriteWithResponse(void *parameter) {
  while(1) { 
      if(xTaskClientWriteWithResponseDataReady) {
          if(!pRemote_Wahoo_ControlPoint_Chr->writeValue(xTaskClientWriteWithResponseData, true))
              DEBUG_PRINTLN("\n>>> Error: Failed to write characteristic -> Return Code = 0x06");
          xTaskClientWriteWithResponseDataReady = false; // Indicate: ready for next data transfer
      }
      vTaskSuspend(xTaskClientWriteWithResponseHandle);
  } // while
};

class server_Wahoo_ControlPoint_Chr_callback: public NimBLECharacteristicCallbacks {
void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo& connInfo) { 
  xTaskClientWriteWithResponseData = server_Wahoo_ControlPoint_Chr->getValue();
  uint8_t WahooCPDataLen = xTaskClientWriteWithResponseData.length();
  // Check for Empty xTaskClientWriteWithResponseData field
  if(!WahooCPDataLen) return; // We are Done!
  static uint8_t eventCnt = 0;
  // Server Wahoo Control Point data is tranferred to the Client
  if(Trainer.IsConnected && (xTaskClientWriteWithResponseDataReady == false) ) {
        xTaskClientWriteWithResponseDataReady = true;      // Server Control Point data are ready!
        vTaskResume(xTaskClientWriteWithResponseHandle);   // Resume xTask and write the data a.s.a.p.
        eventCnt = 0;  
  } else {  // Zwift keeps sending CP-Data -> Trainer is unresponsive!
    if(eventCnt++ < 2) DEBUG_PRINTLN("\n>>> Wahoo Trainer is unresponsive! Skipped Data!"); // Limit to 3
    return; // Done!
  }
#ifdef DEBUG_WAHOO_CONTROLPOINT_OPCODE_DATA
  memset(server_Wahoo_Control_Point_Data.bytes, 0, sizeof(server_Wahoo_Control_Point_Data.bytes));
  // Display the raw request packet 
  DEBUG_PRINTF("-> Server Rec'd Raw Wahoo Control Point Data [len: %d] ", WahooCPDataLen);
  // Transfer the contents of data to server_Wahoo_Control_Point_Data.bytes
  for (int i = 0; i < WahooCPDataLen; i++) {
      server_Wahoo_Control_Point_Data.bytes[i] = xTaskClientWriteWithResponseData[i];
  }
  /// Decodes an incoming Wahoo Control Point request
  DEBUG_PRINTF("[OpCode: %02X] [Values: ", server_Wahoo_Control_Point_Data.values.OPCODE, HEX);
  for (int i=0; i<WahooCPDataLen; i++) { 
    DEBUG_PRINTF("%02X ", server_Wahoo_Control_Point_Data.values.OCTETS[i], HEX); 
  }
  DEBUG_PRINTLN("]");
  // The documentation I found states that all write actions to this Wahoo CP characteristic are "Write with Response"
  // So we have formally to acknowledge the receipt of the trainer setting
  // Zwift does NOT care at all if one sets a response, it is still working !!!
  switch(server_Wahoo_Control_Point_Data.values.OPCODE) {
    case unlock: {
      /*
      WahooRespConfirmBuffer[1] = server_Wahoo_Control_Point_Data.values.OPCODE;
      server_Wahoo_ControlPoint_Chr->setValue(WahooRespConfirmBuffer, 2);
      server_Wahoo_ControlPoint_Chr->indicate();
      */
      DEBUG_PRINTLN("    Request to Unlock Machine!");
      break;
    }
    case setResistanceMode: {
       /* confirm OK!
      WahooRespConfirmBuffer[1] = server_Wahoo_Control_Point_Data.values.OPCODE;
      server_Wahoo_ControlPoint_Chr->setValue(WahooRespConfirmBuffer, 2);
      server_Wahoo_ControlPoint_Chr->indicate();
      */
      DEBUG_PRINTLN("    Set Resistance Mode!");
      break;
    }
    case setStandardMode: {
       /* confirm OK!
      WahooRespConfirmBuffer[1] = server_Wahoo_Control_Point_Data.values.OPCODE;
      server_Wahoo_ControlPoint_Chr->setValue(WahooRespConfirmBuffer, 2);
      server_Wahoo_ControlPoint_Chr->indicate();
      */
      DEBUG_PRINTLN("    Set Standard Mode!");
      break;
    }
    case setSimMode : {
      /* Confirm OK!
      WahooRespConfirmBuffer[1] = server_Wahoo_Control_Point_Data.values.OPCODE;
      server_Wahoo_ControlPoint_Chr->setValue(WahooRespConfirmBuffer, 2);
      server_Wahoo_ControlPoint_Chr->indicate();
      */      
      DEBUG_PRINT("    Set Simulation Mode! ");   
      uint16_t tmp = (server_Wahoo_Control_Point_Data.values.OCTETS[0]) + (server_Wahoo_Control_Point_Data.values.OCTETS[1] << 8);       
      float weight = (float(tmp) / 100); // Rider weight in Kg
      tmp = ( (server_Wahoo_Control_Point_Data.values.OCTETS[2]) + (server_Wahoo_Control_Point_Data.values.OCTETS[3] << 8) );
      float rrc = (float(tmp) / 1000);    // Rolling Resistance Coefficient
      tmp = ( (server_Wahoo_Control_Point_Data.values.OCTETS[4]) + (server_Wahoo_Control_Point_Data.values.OCTETS[5] << 8) );      
      float wrc = (float(tmp) / 1000);    // Wind Resistance Coefficient
      DEBUG_PRINTF(" --> Weight: %0.2f RRC: %f WRC: %f\n", weight, rrc, wrc); 
      break;
    }
    case setSimGrade: {
      /* Confirm OK!
      WahooRespConfirmBuffer[1] = server_Wahoo_Control_Point_Data.values.OPCODE;
      server_Wahoo_ControlPoint_Chr->setValue(WahooRespConfirmBuffer, 2);
      server_Wahoo_ControlPoint_Chr->indicate();
      */
      DEBUG_PRINT("    Set Simulation Grade! "); 
      uint16_t gr = ( server_Wahoo_Control_Point_Data.values.OCTETS[0] + (server_Wahoo_Control_Point_Data.values.OCTETS[1] << 8) );
      float SimGrade = 100 * float( ((gr * 2.0 / 65535) - 1.0) ); // Percentage of road grade --> range: between +1 and -1 (!)
      DEBUG_PRINTF(" --> Grade: %4.1f%%\n", SimGrade); //
      break;       
    }
    case setErgMode:
    case setSimCRR:
    case setSimWindResistance:
    case setSimWindSpeed:
    case setWheelCircumference:
    {
      /*        
      WahooRespUnknownBuffer[1] = server_Wahoo_Control_Point_Data.values.OPCODE;
      server_Wahoo_ControlPoint_Chr->setValue(WahooRespUnknownBuffer, 2);
      server_Wahoo_ControlPoint_Chr->indicate();
      */
      DEBUG_PRINTLN("    Unresolved OpCode!");
      break;
    }
    } // switch
#endif
  }; // onWrite
  
  void onSubscribe(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo, uint16_t subValue) {
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
    
    // Wahoo proprietary Control Point Characteristic --------------------------------------------------------------------------
    server_Wahoo_ControlPoint_Chr = server_CyclingPower_Service->createCharacteristic(UUID16_CHR_WAHOO_CONTROL_POINT, 
                                                                            NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::INDICATE);
    server_Wahoo_ControlPoint_Chr->setCallbacks(new server_Wahoo_ControlPoint_Chr_callback);
    // -------------------------------------------------------------------------------------------------------------------------
    
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

class server_NUS_Rxd_Chr_callback: public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo& connInfo) {
    // Read data received over NUS Rxd from Mobile Phone
#ifdef DEBUG
    std::string NusRxdData = server_NUS_Rxd_Chr->getValue();
    uint8_t NusRxdDataLen = NusRxdData.length();  // Get the actual length of data bytes
    // Display the raw packet data in actual length
    DEBUG_PRINTF("-> Server Rec'd Raw NUS Rxd Data [%d][%s]\n", NusRxdDataLen, NusRxdData.c_str());
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
  
    // Set Device Information varariables   
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

void clientSubscribeToAll(bool IsEnable)
{   
if(IsEnable) { // Enable Client Char's
  if( pRemote_Wahoo_ControlPoint_Chr != nullptr) { // Check: Is it exposed?
            pRemote_Wahoo_ControlPoint_Chr->subscribe(indications, client_Wahoo_ControlPoint_Indicate_Callback);
            // --------------------------WAHOO UNLOCK ----------------------------------------------
            // Unlock the client_Wahoo_ControlPoint Characteristic at the Wahoo trainer
            delay(50); // Time to settle after subscribe
            DEBUG_PRINTLN("Client sends to Wahoo Control Point: [ 20 EE FC ] -> Unlock Command Key");
            // write with response !!! writeValue(uint8_t* data, size_t length, bool response = false);
            pRemote_Wahoo_ControlPoint_Chr->writeValue(unlockCommand, 3, true);      
            delay(100); // Give the trainer some time to wake up  
            // --------------------------------------------------------------------------------------
         }
  if ( pRemote_CSC_Measurement_Chr != nullptr ) { 
            pRemote_CSC_Measurement_Chr->subscribe(notifications, client_CSC_Measurement_Notify_Callback);
          }          
  if ( pRemote_CP_Measurement_Chr != nullptr ) {
            pRemote_CP_Measurement_Chr->subscribe(notifications, client_CP_Measurement_Notify_Callback); 
          }                     
  DEBUG_PRINTLN("All Client (Trainer) Characteristics are Notify/Indicate Enabled!");
  } else { // Disable Client Char's
  if( pRemote_Wahoo_ControlPoint_Chr != nullptr) { // Check: Is it exposed?
            pRemote_Wahoo_ControlPoint_Chr->unsubscribe(false); 
          }
  if ( pRemote_CSC_Measurement_Chr != nullptr ) {
            pRemote_CSC_Measurement_Chr->unsubscribe(false);
          }                      
  if ( pRemote_CP_Measurement_Chr != nullptr ) {
            pRemote_CP_Measurement_Chr->unsubscribe(false);
          }
  DEBUG_PRINTLN("All Client (Trainer) Characteristics are Notify/Indicate Disabled!");
  }
} // end

void loop() { // loop() is used to start sort of Callback functions
 // If the flag "DoCallClientEnable" is true, we enable Notify and Indicate on ALL Client Char's of BLE server!
  if(Trainer.IsConnected && DoCallClientEnable) {
    DoCallClientEnable = false; 
    clientSubscribeToAll(true);
  }
 // If the flag "DoCallClientDisable" is true, we disable Notify and Indicate on ALL Client Char's of BLE server!
  if(Trainer.IsConnected && DoCallClientDisable) {
    DoCallClientDisable = false; 
    clientSubscribeToAll(false);
  }
  // If the flag "doClientConnectCall" is true, we connect to the BLE server!
  if (doClientConnectCall) {
    if( !ClientConnectServer() ) {
        DEBUG_PRINTLN(">>> Failed to connect Peripheral (Wahoo)!");
        NimBLEDevice::deleteClient(pClient_Wahoo); // Delete client object and clear from list
        pClient_Wahoo = nullptr;   // Clear to null
        if(!hasConnectPassed) RestartScanningOnDisconnect = true; 
    }
    doClientConnectCall = false;
  } // doClientConnectCall
  // If the flag "RestartScanningOnDisconnect" is true, we start scanning for a (new) BLE Server!  
  if(RestartScanningOnDisconnect) {   
        pNimBLEScan->clearResults();   // delete results from BLEScan buffer to release memory    
        DEBUG_PRINTLN("Client Restarts Scanning for Server Device (Wahoo) with CPS!");
        RestartScanningOnDisconnect = false;        
        pNimBLEScan->start(0, false);
  } 
 delay(200);  // DO NOT REMOVE or Task watchdog will be triggered!!!   
} // End of loop
