/*******************************************************************************
 This is an application for the Adafruit Feather nRF52 (Bluefruit Bluetooth LE) 
 --> Nordic Semiconductor SoftDevice installed: S132
 *******************************************************************************/

/*  This sketch shows how to use BLEClientService and BLEClientCharacteristic of the 
 *  Bluefruit library to implement a custom client (a.k.a. CENTRAL) that is used to listen 
 *  and talk with a Gatt server on the PERIPHERAL (i.e. a TACX Trainer). In our case a 
 *  TACX Indoor Bike Trainer of type NEO... a so called "smart trainer" that is capable 
 *  of working with BLE and ANT+ using the FE-C protocol for this type of equipment.
 *
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
  * 7) In principle one can run the Feather nRF52840 Express and this test code during a real life indoor workout, 
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
 
#include <bluefruit.h>

///////////////////////////////////////////////
/////////// TAXC FE-C ANT+ over BLE ///////////
///////////////////////////////////////////////
//
//Tacx FE-C primary service and characteristics Uuid's need special treatment !!
//
//TACX_FEC_PRIMARY_SERVICE UUID is 128 bit:    6E 40 FE C1 - B5 A3 - F3 93 - E0 A9 - E5 0E 24 DC CA 9E
// Declare in Reversed order !!!
uint8_t TACX_FEC_PRIMARY_SERVICE_Uuid[16]=     {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0xC1, 0xFE, 0x40, 0x6E,};
//TACX_FEC_READ_CHARACTERISTIC is 128 bit:     6E 40 FE C2 - B5 A3 - F3 93 - E0 A9 - E5 0E 24 DC CA 9E
// Declare in Reversed order !!!
uint8_t TACX_FEC_READ_CHARACTERISTIC_Uuid[16]= {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0xC2, 0xFE, 0x40, 0x6E,};
//TACX_FEC_WRITE_CHARACTERISTIC is 128 bit:    6E 40 FE C3 - B5 A3 - F3 93 - E0 A9 - E5 0E 24 DC CA 9E
// Declare in Reversed order !!!
uint8_t TACX_FEC_WRITE_CHARACTERISTIC_Uuid[16]={0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0xC3, 0xFE, 0x40, 0x6E,};
//
///////////////////////////////////////////////
/////////// TAXC FE-C ANT+ over BLE ///////////
///////////////////////////////////////////////

/* 
 *  Declare the BLE DIS Service and Characterics Uuid's
 */
#define SERVICE_DEVICE_INFORMATION                  0x180A
#define CHARACTERISTIC_MODEL_NUMBER_STRING          0x2A24
#define CHARACTERISTIC_SERIAL_NUMBER_STRING         0x2A25
#define CHARACTERISTIC_FIRMWARE_REVISION_STRING     0x2A26
#define CHARACTERISTIC_HARDWARE_REVISION_STRING     0x2A27
#define CHARACTERISTIC_SOFTWARE_REVISION_STRING     0x2A20
#define CHARACTERISTIC_MANUFACTURER_NAME_STRING     0x2A29

/* Declare services and charateristics for TACX FE-C trainer FEC
 *  
 */
BLEClientService        fecps(TACX_FEC_PRIMARY_SERVICE_Uuid);
BLEClientCharacteristic fecrd(TACX_FEC_READ_CHARACTERISTIC_Uuid);
BLEClientCharacteristic fecwr(TACX_FEC_WRITE_CHARACTERISTIC_Uuid);

/* Declare services and characteristics for Device Information Service DIS
 * 
 */
BLEClientService        diss(SERVICE_DEVICE_INFORMATION);
BLEClientCharacteristic disma(CHARACTERISTIC_MANUFACTURER_NAME_STRING);
BLEClientCharacteristic dismo(CHARACTERISTIC_MODEL_NUMBER_STRING);
BLEClientCharacteristic dissn(CHARACTERISTIC_SERIAL_NUMBER_STRING);
BLEClientCharacteristic disfi(CHARACTERISTIC_FIRMWARE_REVISION_STRING);
BLEClientCharacteristic disha(CHARACTERISTIC_HARDWARE_REVISION_STRING);
BLEClientCharacteristic disso(CHARACTERISTIC_SOFTWARE_REVISION_STRING);

//Global variables for measurement data
bool ConnectedToTACX = false;

//Define the Request Page 51 Command to send
const unsigned char Page51Bytes[13] = {
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
    
// FE-C Page 51 globals
unsigned long SendRequestPage51Delay = 4000; //Sample rate for Page 51 requests 4 seconds ?
unsigned long SendRequestPage51Event = millis(); //Millis of last Page 51 request event

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for feather nrf52832 with native usb
#endif  
  DEBUG_PRINTLN("nRF52 Code to Test Tacx Trainer Support of FE-C ANT+ over BLE");
  DEBUG_PRINTLN("------------------------ Version 001 ------------------------");

  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(0, 1);
  Bluefruit.setName("SIMCLINE");
  
  // TACX FE-C trainer service and characteristics initialization
  fecps.begin();
  // set up callback for receiving ANT+ FE-C packet
  fecrd.setNotifyCallback(fecrd_notify_callback);
  fecrd.begin();
  fecwr.begin();
  //
  // Initialize DISS client.
  // Note: Client Char will be added to the last service that is begin()ed.
  diss.begin();
  //
  // Initialize some characteristics of the Device Information Service.
  disma.begin();
  dismo.begin();
  dissn.begin();
  disfi.begin();
  disha.begin();
  disso.begin();
  
  // Blink in advertising mode
  Bluefruit.setConnLedInterval(150);

  // Callbacks for Central
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);
  Bluefruit.Central.setConnectCallback(connect_callback);

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Don't use active scan
   * - Filter only accept CPS service
   * - Start(timeout) with timeout = 0 will scan forever (until connected)
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.filterRssi(-70);      // original value of -80 , we want to scan only nearby peripherals, so get close to your TACX trainer !!
  Bluefruit.Scanner.setInterval(160, 80); // in units of 0.625 ms
  Bluefruit.Scanner.filterUuid(0x180A, 0x1816, 0x1818); // we are only interested in the services of the TACX Trainer
  Bluefruit.Scanner.useActiveScan(true);  // was false...
  Bluefruit.Scanner.start(0);             // 0 = Don't stop scanning after n seconds
  DEBUG_PRINTLN("Scanning for TACX ...");
}

void loop()
{
    // Send a request for Page 51 about every 4 seconds
    if (ConnectedToTACX)
      {
        long tmp = millis()-SendRequestPage51Delay;
        if (tmp >= SendRequestPage51Event)
          {
          SendRequestPage51Event = millis();
          SendRequestPage51();
          } 
      }
}

/**
 * Callback invoked when scanner picks up advertising data
 * @param report Structural advertising data
 */
void scan_callback(ble_gap_evt_adv_report_t* report)
{
  // Since we configure the scanner with filterUuid()
  // Scan callback only invoked for device with TACX services advertised
  // So connect to a device with these services in advertising
  // This is NOT FAIL PROOF if more than one trainer is in reach!
 
  Bluefruit.Central.connect(report);
#ifdef DEBUG  
  DEBUG_PRINTLN("Timestamp Addr              Rssi Data");
  DEBUG_PRINTF("%09d ", millis());
    // MAC is in little endian --> print reverse
  Serial.printBufferReverse(report->peer_addr.addr, 6, ':');
  DEBUG_PRINT(" ");
  DEBUG_PRINT(report->rssi);
  DEBUG_PRINT("  ");
  Serial.printBuffer(report->data.p_data, report->data.len, '-');
  DEBUG_PRINTLN();
#endif
  //-------------------------------
}

/**
 * Callback invoked when an connection is established
 * @param conn_handle
 */
void connect_callback(uint16_t conn_handle)
{
/* First check for TACX Trainer services to be present
 *  
 */

  DEBUG_PRINTLN("Connected! Now checking for critical services...");
  DEBUG_PRINTLN("Discovering FECPS Service ...");
  // If FECPS is not found, disconnect and return
  if ( !fecps.discover(conn_handle) )
  {
    DEBUG_PRINTLN("Failed and disconnecting ...");
    // disconnect since we couldn't find crucial service
    Bluefruit.disconnect(conn_handle);
    return;
  }
  DEBUG_PRINTLN("Found it !");
  DEBUG_PRINTLN("Discovering FECRD Characteristic ...");
  // If FECRD is not found, disconnect and return
  if ( !fecrd.discover() )
  {
    DEBUG_PRINTLN("Failed and disconnecting ...");
    // disconnect since we couldn't find crucial service
    Bluefruit.disconnect(conn_handle);
    return;
  }

  DEBUG_PRINTLN("Found it !");
  DEBUG_PRINTLN("Discovering FECWR Characteristic ...");
  // If FECWR is not found, disconnect and return
  if ( !fecwr.discover() )
  {
    DEBUG_PRINTLN("Failed and disconnecting ...");
    // disconnect since we couldn't find crucial service
    Bluefruit.disconnect(conn_handle);
    return;
  }
  DEBUG_PRINTLN("Found it !");
  DEBUG_PRINTLN("Discovering Device Information Service ...");
  
/*
 * Now check for the common BLE DIS Service
 */
  // If diss is not found then go on.... NOT FATAL !
  if ( diss.discover(conn_handle) )
  {
    DEBUG_PRINTLN("Found it !");
    char buffer[32+1];
    //  1
    if ( disma.discover() )
    {
    // read and print out Manufacturer
    memset(buffer, 0, sizeof(buffer));
    if ( disma.read(buffer, sizeof(buffer)) )
      {
      DEBUG_PRINT("Manufacturer: ");
      DEBUG_PRINTLN(buffer);
      }
    }
    //  2
    if ( dismo.discover() )
    {
    // read and print out Model Number
    memset(buffer, 0, sizeof(buffer));
    if ( dismo.read(buffer, sizeof(buffer)) )
      {
      DEBUG_PRINT("Model Number: ");
      DEBUG_PRINTLN(buffer);
       }
    }
    //  3
    if ( dissn.discover() )
    {
    // read and print out Serial Number
    memset(buffer, 0, sizeof(buffer));
    if ( dissn.read(buffer, sizeof(buffer)) )
      {
      DEBUG_PRINT("Serial Number: ");
      DEBUG_PRINTLN(buffer);
      }
    }
    //  4
    if ( disfi.discover() )
    {
    // read and print out Firmware Revision Number
    memset(buffer, 0, sizeof(buffer));
    if ( disfi.read(buffer, sizeof(buffer)) )
      {
      DEBUG_PRINT("Firmware Revision Number: ");
      DEBUG_PRINTLN(buffer);
      }
    }
    //  5   
    if ( disha.discover() )
    {
    // read and print out Harware Revision Number
    memset(buffer, 0, sizeof(buffer));
    if ( disha.read(buffer, sizeof(buffer)) )
      {
      DEBUG_PRINT("Hardware Revision Number: ");
      DEBUG_PRINTLN(buffer);
      }
    }    
    //  6
    if ( disso.discover() )
    {
    // read and print out Software Revision Number
    memset(buffer, 0, sizeof(buffer));
    if ( disso.read(buffer, sizeof(buffer)) )
      {
      DEBUG_PRINT("Software Revision Number: ");
      DEBUG_PRINTLN(buffer);
      }
    }
    // no more characteristics of Device Information Service
  }
  else 
  {   
    DEBUG_PRINTLN("Found NONE !");
  }
  
  // ---------------------------------------------------------------------------------
  // Reaching here means we are ready to go, let's enable notification on reading data
  // ---------------------------------------------------------------------------------
  // ANT+ FE-C protocol reading is started now ! -------------------------------------
  // ---------------------------------------------------------------------------------
    if ( fecrd.enableNotify() )
  {
  // To set ALL CLEAR: READY TO ROCK AND ROLL ---> calls handled in loop()
    ConnectedToTACX = true;  
    DEBUG_PRINTLN("Ready to receive FE-C messages");
  }
  else
  {
    DEBUG_PRINTLN("Couldn't enable notify for FE-C messages");
  }
} // Done --------------------------------------------


/**
 * Callback invoked when a connection is dropped
 * @param conn_handle
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
  DEBUG_PRINT("Disconnected, reason = 0x"); DEBUG_PRINTLN(reason, HEX);
}

void SendRequestPage51()
{  
    DEBUG_PRINTLN("Send Common Page: 70 (0x46) with Request for Data Page [51] (0x33)");
    // Page51Bytes are globally defined
    // uint16_t write(const void* data, uint16_t len);
    fecwr.write(Page51Bytes, sizeof(Page51Bytes));
}
  
/**
 * Hooked callback that is triggered when a measurement value is sent from TACX Trainer
 * @param chr   Pointer client characteristic that event occurred,
 *              in this example it should be FECRD
 * @param data  Pointer to received data
 * @param len   Length of received data
 */
void fecrd_notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
  // The FE-C Read charateristic of ANT+ packets
  // In TACX context receive or send arrays of data ranging from 1--20 bytes so FE-C
  // will not exceed the 20 byte maximum
  
  uint8_t buffer[20+1];
  memset(buffer, 0, sizeof(buffer));
  DEBUG_PRINTF("Rec'd Raw FE-C Data len: [%02d] [", len);
  for (int i = 0; i < len; i++) {
    if ( i <= sizeof(buffer)) {
      buffer[i] = *data++;
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
  }
  //////////////////////// DONE! /////////////////////////
}
