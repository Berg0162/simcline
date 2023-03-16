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
 *  Note: you will need an active and working TACX Trainer to test with.
 */
 
/*
 *               This version heavily relies on ANT+ FE-C over BLE !!!
 * ONLY the common BLE Device Information and Generic Access Services are used aside of
 * the dominant Tacx FE-C primary service and characteristics for BLE
 */
 
 /*
  * This code will do the following
  * 1) Scans for a TACX trainer advertising itself over BLE (trainer must have power and be swithed ON !!),
  * 2) Will try to connect to the trainer if present, if FAILED: see Serial Monitor for debugging info,
  * 3) Search for the FE-C characteristic services to be available,
  * 4) Poll and show the characteristic trainer values (see Serial Monitor),
  * 5) Run (until switched off) a working connection of FE-C over BLE and 
  * 6) Showing road grade percentage and cycling data, when trainer is being loaded in a regular workout!
  * 7) In principle one can run the Feather ESP32 V2 and this test code during a real life indoor workout, 
  *    having Zwift setting the road grade and you sweating on the trainer. You will see how Zwift and your 
  *    Feather are synched!
  */

 /* --------------------------------------------------------------------------------------------------------------------------------------
  *  NOTICE that many older smart trainer devices allow ANT+ and BLE, however, they only support 1 (ONE!) BLE connection at the time, 
  *  which means that in that case you cannot concurrently connect your trainer with ZWIFT AND with the Feather nRF52/ESP32 over BLE, since it 
  *  is considered a second device and will not connect over BLE. ANT+ supports by definition multiple devices to connect!!!
  *  --> Blame the economical manufacturer and not the messenger!
  *  Solution: Apply ANT+ connection for the regular Trainer-Zwift/PC-link and use the single BLE connection for connecting the Feather nRF52/ESP32.
  *  I connect in addition my Garmin cycling computer with the trainer over ANT+ and have the Feather nRF52/ESP32 use the single BLE connection!
  * ---------------------------------------------------------------------------------------------------------------------------------------
  */

// -------------------------------------------------------------------------------------------
// COMPILER DIRECTIVE to allow/suppress DEBUG messages that help debugging...
// Uncomment general "#define DEBUG" to activate
#define DEBUG
// Include these debug utility macros in all cases!
#include "DebugUtils.h"
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
static BLERemoteCharacteristic* pRemote_GA_DeviceName_Chr; // Read, Write
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
static BLERemoteCharacteristic* pRemote_DIS_ManufacturerName_Chr;  // Read
std::string client_DIS_Manufacturer_Str;
static BLERemoteCharacteristic* pRemote_DIS_ModelNumber_Chr;       // Read
std::string client_DIS_ModelNumber_Str;
static BLERemoteCharacteristic* pRemote_DIS_SerialNumber_Chr;      // Read
std::string client_DIS_SerialNumber_Str;
// -------------------------------------------------------------------------------------

#define MAXPAYLOAD 20

///////////////////////////////////////////////
/////////// TAXC FE-C ANT+ over BLE ///////////
///////////////////////////////////////////////
/* TAXC FE-C ANT+ over BLE---------------------------------------------------------------
 * TACX_FE-C_PRIMARY_SERVICE      is 128 bit:    6E40FEC1-B5A3-F393-E0A9-E50E24DCCA9E 
 * TACX_FE-C_READ_CHARACTERISTIC  is 128 bit:    6E40FEC2-B5A3-F393-E0A9-E50E24DCCA9E
 * TACX_FE-C_WRITE_CHARACTERISTIC is 128 bit:    6E40FEC3-B5A3-F393-E0A9-E50E24DCCA9E
 */
BLEUUID UUID_TACX_FEC_PRIMARY_SERVICE("6E40FEC1-B5A3-F393-E0A9-E50E24DCCA9E");
BLEUUID UUID_TACX_FEC_RXD_CHARACTERISTIC("6E40FEC2-B5A3-F393-E0A9-E50E24DCCA9E"); 
BLEUUID UUID_TACX_FEC_TXD_CHARACTERISTIC("6E40FEC3-B5A3-F393-E0A9-E50E24DCCA9E"); 
 
static BLERemoteService*        pRemote_Tacx_FEC_Service;
static BLERemoteCharacteristic* pRemote_Tacx_FEC_Rxd_Chr; // Read Notify
static BLERemoteCharacteristic* pRemote_Tacx_FEC_Txd_Chr; // Write No Response
// --------------------------------------------------------------------------------------

//Define the Request Page 51 Command to send
const uint8_t Page51Bytes[13] = {
    0xA4, //Sync
    0x09, //Length
    0x4F, //Acknowledge message type
    0x05, //Channel 
          //Data
    0x46, //Common Page 70
    0xFF,
    0xFF,
    0xFF, //Descriptor byte 1 (0xFF for no value)
    0xFF, //Descriptor byte 2 (0xFF for no value)
    0x80, //Requested transmission response
    0x33, //Requested page number 51 
    0x01, //Command type (0x01 for request data page, 0x02 for request ANT-FS session)
    0x47}; //Checksum;
    
// FE-C Requested Page 51 globals
unsigned long REQUEST_PAGE_51_DELAY = 4000; //Sample rate for Page 51 requests 4 seconds ?
unsigned long SendRequestPage51Event = millis(); //Millis of last Page 51 request event
// --------------------------------------------------------------------------------------

static BLEClient* pClient_Tacx;
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
  DEBUG_PRINTLN("ESP32 Code to Test Tacx Trainer Support of FE-C ANT+ over BLE");
  DEBUG_PRINTLN("------------------------ Version 001 ------------------------");
  BLEDevice::init("");
  pClient_Tacx  = BLEDevice::createClient();
  pClient_Tacx->setClientCallbacks(new client_Connection_Callbacks());    
  client_Start_Scanning();
} // End of setup.

void client_Connection_Callbacks::onConnect(BLEClient* pClient) {
    NimBLEAddress MyAddress = myDevice->getAddress();
    clientPeerName = myDevice->getName().c_str();
    //DEBUG_PRINTF("BLEDevice address in Little Endian order: [%s]\n", MyAddress->getAddress().toString().c_str());
    memcpy(&clientPeerAddress, MyAddress.getNative(), 6); 
#ifdef DEBUG
    DEBUG_PRINT("Client Connection Parameters -> ");
    uint16_t max_payload = pClient_Tacx->getMTU()-3;
    //DEBUG_PRINTF("Max Transmission Unit: [%d] ", max_payload);
    uint16_t clientConnectionInterval = pClient_Tacx->getConnInfo().getConnInterval();
    DEBUG_PRINTF("Interval: [%d] ", clientConnectionInterval);
    uint16_t clientConnectionLatency = pClient_Tacx->getConnInfo().getConnLatency();
    DEBUG_PRINTF("Latency: [%d] ", clientConnectionLatency);
    uint16_t clientConnectionSupTimeout = pClient_Tacx->getConnInfo().getConnTimeout();
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
    //pClient_Tacx->updateConnParams(pClient_Tacx->getConnId(), 24, 48, 0, 400); 
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
    pRemote_DeviceInformation_Service = pClient_Tacx->getService(UUID16_SVC_DEVICE_INFORMATION);    
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
    pRemote_GenericAccess_Service = pClient_Tacx->getService(UUID16_SVC_GENERIC_ACCESS);    
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

void Tacx_FEC_Rxd_Notify_Callback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) 
{
  // The FE-C Read charateristic of ANT+ packets
  // In TACX context receive or send arrays of data ranging from 1--20 bytes so FE-C
  // will not exceed the 20 byte maximum
  
  uint8_t buffer[MAXPAYLOAD+1];
  memset(buffer, 0, sizeof(buffer));
  DEBUG_PRINTF("Rec'd Raw FE-C Data len: [%02d] [", length);
  for (int i = 0; i < length; i++) {
    if ( i <= sizeof(buffer)) {
      buffer[i] = *pData++;
      DEBUG_PRINTF("%02X ", buffer[i], HEX);
    }
  }
  DEBUG_PRINT("] ");
  uint8_t PageValue = buffer[4]; // Get Page number from packet
  switch(PageValue)
  {
    case 0x47 :
    ////////////////////////////////////////////////////////////
    //////////////////// Handle PAGE 71 ////////////////////////
    ////////////// Requested PAGE 51 for grade info ////////////
    ////////////////////////////////////////////////////////////
    if ( buffer[5] == 0x33 ) // check for requested page 51
      {
      uint8_t lsb_gradeValue = buffer[9];
      uint8_t msb_gradeValue = buffer[10];
      long RawgradeValue = lsb_gradeValue + msb_gradeValue*256;
      float gradePercentValue = float((RawgradeValue - 20000))/100;
      // in steps of 0.01% and with an offset of -200%
      // gradeValue     gradePercentValue
      //     0                 -200%
      //  19000                 -10%
      //  20000                   0%
      //  22000                 +20%
      //  40000                +200%
      // -------------------------------------
      DEBUG_PRINTF("Page: %02d (0x%02X) Requested Page 51 (0x33) Data Received - RawgradeValue: %05d  ", PageValue, PageValue, RawgradeValue); 
      DEBUG_PRINTF("Grade percentage: [%05.1f]%%", gradePercentValue);
      DEBUG_PRINTLN();
      }  
    break;
  case 0x19 :
    {
    /////////////////////////////////////////////////
    /////////// Handle PAGE 25 Trainer/Bike Data ////
    /////////////////////////////////////////////////
    uint8_t UpdateEventCount = buffer[5];
    uint8_t InstantaneousCadence = buffer[6];
    uint8_t lsb_InstantaneousPower = buffer[9];
    // POWER is stored in 1.5 byte !!!
    uint8_t msb_InstantaneousPower = (buffer[10] & 0x0F); // bits 0:3 --> MSNibble only!!!
    long PowerValue = lsb_InstantaneousPower + msb_InstantaneousPower*256;
    DEBUG_PRINTF("Page: %02d (0x%02X) Bike Data - Event count: [%03d]", PageValue, PageValue, UpdateEventCount); 
    DEBUG_PRINTF(" - Cadence: [%03d]", InstantaneousCadence);
    DEBUG_PRINTF(" - Power in Watts: [%04d]", PowerValue);
    DEBUG_PRINTLN();
    }
    break;
  case 0x10 :
    {
    //////////////////////////////////////////////
    //////////// Handle PAGE 16 General FE Data //
    //////////////////////////////////////////////
    uint8_t ElapsedTime = (uint8_t)((buffer[6]/4) %60);  // units of 0.25 seconds --> 256 Rollover (every 64 seconds)
    uint8_t DistanceTravelled = buffer[7]; // in meters 256 m rollover 
    uint8_t lsb_SpeedValue = buffer[8];
    uint8_t msb_SpeedValue = buffer[9];
    long SpeedValue = ((lsb_SpeedValue + msb_SpeedValue*256)/1000)*3.6; // in units of 0,001 m/s naar km/h
    DEBUG_PRINTF("Page: %02d (0x%02X) General Data - Elapsed time: [%02d]s", PageValue, PageValue, ElapsedTime);
    DEBUG_PRINTF(" - Distance travelled: [%05d]m", DistanceTravelled); 
    DEBUG_PRINTF(" - Speed: [%02d]km/h", SpeedValue);
    DEBUG_PRINTLN();
    }
    break;
  case 0x80 :
    {
    // Manufacturer Identification Page
    }
  default :
    {
    DEBUG_PRINTF("Page: %02d (0x%02X) Undecoded\n", PageValue, PageValue); 
    return;
    }
  } // Switch
  //////////////////////// DONE! /////////////////////////
}

bool client_Tacx_FEC_Connect(void)
{
    // Obtain a reference to the remote CP service.
    pRemote_Tacx_FEC_Service = pClient_Tacx->getService(UUID_TACX_FEC_PRIMARY_SERVICE);
    if (pRemote_Tacx_FEC_Service == nullptr) {
      DEBUG_PRINTLN("Mandatory Tacx FE-C Service: Not Found!");
      return false;
    }
    DEBUG_PRINTLN("Client_Tacx_FEC_Service: Found!");

    pRemote_Tacx_FEC_Rxd_Chr = pRemote_Tacx_FEC_Service->getCharacteristic(UUID_TACX_FEC_RXD_CHARACTERISTIC);
    if (pRemote_Tacx_FEC_Rxd_Chr == nullptr) {
      DEBUG_PRINTLN("Mandatory Client_Tacx_FEC_Rxd_Chr: Not Found!");
      return false;
    }
    DEBUG_PRINTLN("Client_Tacx_FEC_Rxd_Chr: Found!");  
    if(pRemote_Tacx_FEC_Rxd_Chr->canNotify()) {
      pRemote_Tacx_FEC_Rxd_Chr->registerForNotify(Tacx_FEC_Rxd_Notify_Callback, false, true); // Notifications false
    } else {
      DEBUG_PRINTLN("Mandatory Client_Tacx_FEC_Rxd_Chr: Cannot Notify!");
      return false;
    }

    pRemote_Tacx_FEC_Txd_Chr = pRemote_Tacx_FEC_Service->getCharacteristic(UUID_TACX_FEC_TXD_CHARACTERISTIC);
    if (pRemote_Tacx_FEC_Txd_Chr == nullptr) {
      DEBUG_PRINTLN("Mandatory Client_Tacx_FEC_Rxd_Chr: Not Found!");
      return false;
    } 
    DEBUG_PRINTLN("Client_Tacx_FEC_Rxd_Chr: Found!");
   
    if(!pRemote_Tacx_FEC_Txd_Chr->canWrite()) {
        DEBUG_PRINTLN("Mandatory Client_Tacx_FEC_Rxd_Chr: Cannot Write!");
        return false;
      }

    // Now Separately -> Notify Enable 
    if ( pRemote_Tacx_FEC_Rxd_Chr != nullptr ) {
      pRemote_Tacx_FEC_Rxd_Chr->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true); 
      DEBUG_PRINTLN("Client_Tacx_FEC_Rxd_Chr: Notify Enabled!");
    }
    return true;    
}

// This is NOT really a Callback --> It should have been implemented that way (see for instance Adafruit Bluefruit BLE library), 
// however, now it is called from loop() ... a poor man's solution!
bool client_Connect_Callback() {
    // Connect to the Wahoo BLE Server.
    pClient_Tacx->connect(myDevice);  // if you pass BLEAdvertisedDevice it will recognize type of peer device address (public or private)     
    DEBUG_PRINTLN("Now checking all Client Services and Characteristics!");
    DEBUG_PRINTLN("If Mandatory Services Fail --> the Client will disconnect!");
  // Discover all relevant Services and Char's
  if( !client_GenericAccess_Connect() ) {
    pClient_Tacx->disconnect();
    return false;    
  }
  if( !client_DeviceInformation_Connect() ) {
    pClient_Tacx->disconnect();
    return false;    
  }
  if( !client_Tacx_FEC_Connect() ) {
    pClient_Tacx->disconnect();
    return false;    
  }
   // trainerisconnected
  clientIsConnected = true;
  sendRequestPage51Event = millis() + REQUEST_PAGE_51_DELAY;  
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
    // We have found a device, let us now see if it contains the Tacx FE-C service we are looking for.
    if (advertisedDevice->haveServiceUUID() && advertisedDevice->isAdvertisingService(UUID_TACX_FEC_PRIMARY_SERVICE)) { // NIMBLE
      BLEDevice::getScan()->stop();
      myDevice = advertisedDevice; // NIMBLE --> Just save the reference now, no need to copy the object         
      /* Connect to the Wahoo BLE Server -> Sorry you can't do that here!!! --------------------------------
      ** pClient_Tacx->connect(myDevice);  NOT ALLOWED TO CALL CONNECT --> CAUSES FATAL ERROR !!!! ???? */  
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

void SendRequestPage51()
{  
    DEBUG_PRINTLN("Send Common Page [70] (0x46) with Request for Data Page [51] (0x33)");
    // Page51Bytes are globally defined
    // const uint8_t* data, size_t length, bool response = false 
    pRemote_Tacx_FEC_Txd_Chr->writeValue(Page51Bytes, sizeof(Page51Bytes), false);
}


void loop() {
  // If the flag "doClientConnectCall" is true, we have found the desired BLE server!
  // Once we are connected and ALL is set the clientIsConnected flag is set true.
  if (doClientConnectCall == true) {
    bool dummy = client_Connect_Callback();
    doClientConnectCall = false;
  } // doClientConnectCall

  if (clientIsConnected) {
    // If time is there, send Request Page 51 to the Tacx trainer...
    if(millis() > sendRequestPage51Event) {
        SendRequestPage51();
        sendRequestPage51Event = millis() + REQUEST_PAGE_51_DELAY;
    } // sendRequestPage51Event
  } else { // client is NOT connected check for (re)start scanning
    if(RestartScanningOnDisconnect) {
        pBLEScan->clearResults();   // delete results from BLEScan buffer to release memory    
        DEBUG_PRINTLN("Client Starts Scanning for Server Device with Tacx FE-C Service!");
        RestartScanningOnDisconnect = false;        
        pBLEScan->start(0, false);
    }
  }      
} // End of loop