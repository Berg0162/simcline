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
 *  Test Code to explore unknown client_CPS Wahoo Characteristic [A0 26 E0 05 ..] --> 100% Wahoo proprietary
*/
 
/* 
 *  This Feather-nRF52 tested code scans for the Known Wahoo Cycling Power Service
 *  that the Wahoo Kickr trainer is advertising, it tries to connect and then 
 *  enables the Wahoo Unknown Characteristic for Trainer Control.....
 *  
 *  Requirements: Running Wahoo Kickr or alike and Feather nRF52 board
 *  1) Start the trainer and do NOT connect with other devices
 *  2) Upload and Run this code on the Feather-nRF52
 *  3) Start the Serial Monitor to catch verbose debugging and data info
 *  4) Trainer and Feather should successfully pair or disconnect...
 *  5) Keep the Serial Monitor visible on top of all windows 
 *  6) Move the trainer pedals and notice/feel changes in resistance at regular intervals...
 *  7) Inspect the info presented by Serial Monitor.....
 *  
 */

#include <bluefruit.h>
// -------------------------------------------------------------------------------------------
// COMPILER DIRECTIVE to allow/suppress DEBUG messages that help debugging...
// Uncomment general "#define DEBUG" to activate
#define DEBUG
// Include these debug utility macros in all cases!
#include "DebugUtils.h"

#define SERVICE_DEVICE_INFORMATION                  0x180A
#define CHARACTERISTIC_MANUFACTURER_NAME_STRING     0x2A29
#define CHARACTERISTIC_MODEL_NUMBER_STRING          0x2A24
#define CHARACTERISTIC_SERIAL_NUMBER_STRING         0x2A25

#define UUID16_SVC_CYCLING_POWER                    0x1818

/* Cycling Power Service
 * client_CP Service: 0x1818  
 * client_CP Characteristic: 0x2A63 (Measurement)
 * client_CP Characteristic: 0x2A65 (Feature)
 * client_CP Characteristic: 0x2A5D (Location)
 */
BLEClientService        client_cps(UUID16_SVC_CYCLING_POWER);
bool   Client_CPS_IsConnected = false; // Boolean to check for IsConnected
BLEClientCharacteristic client_cpmc(UUID16_CHR_CYCLING_POWER_MEASUREMENT);
BLEClientCharacteristic client_cpfc(UUID16_CHR_CYCLING_POWER_FEATURE);
uint32_t client_cpfcDef = 0;
BLEClientCharacteristic client_cplc(UUID16_CHR_SENSOR_LOCATION);
uint8_t client_cplc_loc_value = 0;
BLEClientCharacteristic client_cpcp(UUID16_CHR_CYCLING_POWER_CONTROL_POINT); // Indicate, write
// Unknown Wahoo Trainer characteristic part of the Cycling Power Service !
// Unknown Characteristic is 128 bit:            A0 26 E0 05 - 0A 7D - 4A B3 - 97 FA - F1 50 0F 9F EB 8B
// Declare in Reversed order !!!
uint8_t CYCLING_POWER_WAHOO_TRAINER_Uuid[16] = {0x8B, 0xEB, 0x9F, 0x0F, 0x50, 0xF1, 0xFA, 0x97, 0xB3, 0x4A, 0x7D, 0x0A, 0x05, 0xE0, 0x26, 0xA0};
BLEClientCharacteristic client_cpwt(CYCLING_POWER_WAHOO_TRAINER_Uuid);

bool client_CPWT_found = false; // Boolean: the Unknown client_CPWT Char is activated or not

// client_CPS Wahoo Trainer Operation Codes in Decimal
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
const uint8_t unlockCommand[3] = {unlock, 0xEE, 0xFC}; // Unlock codes

const char* Feature_str[] = { 
      "Pedal power balance supported",
      "Accumulated torque supported",
      "Wheel revolution data supported",
      "Crank revolution data supported",
      "Extreme magnatudes supported",
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

/*
 * ------------------------------------------------------------
 *    NOTICE this setup is working for all trainer devices
 *    that support client_CPS (mandatory) and the Wahoo Trainer 
 *    proprietary client_CP Characteristic
 *------------------------------------------------------------                  
 */

// Service Device Information
BLEClientService        client_diss(SERVICE_DEVICE_INFORMATION);
BLEClientCharacteristic client_disma(CHARACTERISTIC_MANUFACTURER_NAME_STRING);
BLEClientCharacteristic client_dismo(CHARACTERISTIC_MODEL_NUMBER_STRING);
BLEClientCharacteristic client_dissn(CHARACTERISTIC_SERIAL_NUMBER_STRING);

// some test variables
#define TIME_SPAN            3000 // Time span in millis 3000 = 3 seconds
unsigned long TimeInterval = 0;
float Grade = 0.0;

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb
#endif
  DEBUG_PRINTLN("Wahoo Kickr Client with Cycling Power Service only");
  DEBUG_PRINTLN("----------- Feather nRF52  Version 031 -----------");
  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(0, 1);

  Setup_Client_CPS();
  Client_Start_Scanning();
  while (Bluefruit.Scanner.isRunning()) { // do nothing else but scanning....
  }
  DEBUG_PRINTLN("Scanning stopped!");

  TimeInterval = millis() + TIME_SPAN;
  // wait the right amount of time or go on when is connected and set!
  while ( (TimeInterval < millis()) || (!Client_CPS_IsConnected) ) { 
   }

  DEBUG_PRINTLN("Up and running!");
  Client_Enable_Notify_Indicate(); // Only now enable data streams...
}

void loop()
{
  // do nothing
}

void Setup_Client_CPS(void)
{
  
  Bluefruit.setName("Bluefruit52 Central");
  // Initialize client_CPS client
  client_cps.begin();

  // Initialize CP Feature characteristics of client_CPS.
  // Note: Client Char will be added to the last service that is begin()ed.
  client_cpfc.begin();

  // Initialize CP Location characteristics of client_CPS.
  // Note: Client Char will be added to the last service that is begin()ed.
  client_cplc.begin();

   // Initialize Control Point and set up Indicate callback for receiving responses (indicate!)
  client_cpcp.setIndicateCallback(client_cpcp_indicate_callback);
  client_cpcp.begin();

  // set up callback for receiving measurement
  client_cpmc.setNotifyCallback(client_cpmc_notify_callback);
  client_cpmc.begin();
  
  // set up callback for receiving response messages (indicate!)
  client_cpwt.setIndicateCallback(client_cpwt_indicate_callback);
  client_cpwt.begin();
  
  // Initialize DISS client.
  client_diss.begin();
  // Initialize some characteristics of the Device Information Service.
  client_disma.begin();
  client_dismo.begin();
  client_dissn.begin();

  // Increase Blink rate to different from PrPh advertising mode
  Bluefruit.setConnLedInterval(250);

  // Callbacks for Central
  Bluefruit.Central.setDisconnectCallback(client_disconnect_callback);
  Bluefruit.Central.setConnectCallback(client_connect_callback);
} // End

void Client_Start_Scanning(void)
{
  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Don't use active scan
   * - Filter only accept client_CP service
   * - Start(timeout) with timeout = 0 will scan forever (until connected)
   */
  Bluefruit.Scanner.setRxCallback(client_scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true); // default is true !!! if false --> this stage we do not want to RESTART
  Bluefruit.Scanner.filterRssi(-70);   // original value of -80 , we want to scan only nearby peripherals, so get close to your device !!
  Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms
  Bluefruit.Scanner.filterUuid(UUID16_SVC_CYCLING_POWER); // only invoke callback if this specific service is advertised 
  Bluefruit.Scanner.useActiveScan(true);
  Bluefruit.Scanner.start(0);  // 0 = Don't stop scanning or n = after n/100 seconds
} // End


void client_scan_callback(ble_gap_evt_adv_report_t* report)
{
/*
* Callback invoked when scanner pick up an advertising data
* @param report Structural advertising data
*/
  // Since we configure the scanner with filterUuid()
  // Scan callback only is invoked for device with client_CPS service advertised
  // Connect to device with client_CPS service in advertising
  if (Bluefruit.Scanner.isRunning()) { Bluefruit.Scanner.stop(); }
  
  DEBUG_PRINTLN("Advertised Client Cycling Power Service is found! ... Raw data packet:");
  DEBUG_PRINTLN(F("Timestamp MAC Address       Rssi Data"));
  DEBUG_PRINTF("%09d ", millis());
  // MAC is in little endian --> print reverse
#ifdef DEBUG
  Serial.printBufferReverse(report->peer_addr.addr, 6, ':');
#endif
  DEBUG_PRINT(F(" "));
  DEBUG_PRINT(report->rssi);
  DEBUG_PRINT(F("  "));
#ifdef DEBUG
  Serial.printBuffer(report->data.p_data, report->data.len, '-');
#endif
  DEBUG_PRINTLN();
  
  Bluefruit.Central.connect(report);
}// End

void Set_SimMode(void)
{
  // Setting simulation mode variables (LEN = 7) Weight: 72.0 RRC: 0.004000 WRC: 0.368000
  uint8_t cpwtData[] = { 0x43, 0x20, 0x1C, 0x04, 0x00, 0x70, 0x01 }; // Set Sim Mode command string
  if (client_CPWT_found) { 
    client_cpwt.write_resp(cpwtData, 7);
    DEBUG_PRINTLN("Client Sets --> Raw Wahoo Control Point data:  [ 43 20 1C 04 00 70 01 ] --> setSimMode Weight: 72.0 RRC: 0.004000 WRC: 0.368000");
    }
}

void Set_SimGrade(float gr)
{
  // Grade is between -1 and +1 (div 100) and NOT in percentage !!!
  uint16_t norm =  uint16_t( (min(1, max(-1, gr/100)) + 1.0) * 65535 / 2.0 ) ; 
  uint8_t cpwtData[] = { 0x46, uint8_t(norm & 0xFF), uint8_t(norm >> 8 & 0xFF) };
  if (client_CPWT_found) { 
    client_cpwt.write_resp(cpwtData, 3); 
    DEBUG_PRINT("Client Sets --> Raw Wahoo Control Point data:  [ ");
    for (int i = 0; i < 3; i++) {
      if ( i <= sizeof(cpwtData)) {
        DEBUG_PRINTF("%02X ", cpwtData[i], HEX);
      }
    }
    DEBUG_PRINTF("] --> setSimGrade Grade: %5.2f%%", gr);
    DEBUG_PRINTLN();
    }
}

void Client_Enable_Notify_Indicate(void)
{
  // Reaching here means we are ready to go, let's enable notification on measurement chr
  if ( client_cpmc.enableNotify() )
  {
    DEBUG_PRINTLN("Ready to receive Client Cycling Power Measurement values");
  }else
  {
    DEBUG_PRINTLN("Couldn't enable notify for client_CP Measurement Characteristic. Increase DEBUG LEVEL for troubleshooting");
  }
  
  if ( client_cpcp.enableIndicate() )
  {
    DEBUG_PRINTLN("Ready to receive Client Cycling Control Point Responses");
  }else
  {
    DEBUG_PRINTLN("Couldn't enable indicate for client_CP Control Point Characteristic. Increase DEBUG LEVEL for troubleshooting");
  }

  if (client_CPWT_found) {
    if ( client_cpwt.enableIndicate() )
    {
      DEBUG_PRINTLN("Unlocking client_CPS Wahoo Trainer Characteristic --> Ready to receive Response Messages");
      // Activate the trainer
      // client_CPWT needs: "write with response" --> client_cpwt.write_resp() 
      client_cpwt.write_resp(unlockCommand, 3); // Unlock the client_CPS Wahoo Characteristic at the Wahoo trainer
      delay(300); // Give the trainer some time to wake up
      Set_SimMode(); // Set the trainers Simulation mode first time
    } else {
      DEBUG_PRINTLN("Couldn't enable indicate for Client Cycling Power Wahoo Trainer Characteristic. Increase DEBUG LEVEL for troubleshooting");
    }
  }
} // End

/*
 * Callback invoked when an connection is established
 * @param conn_handle
 */
void client_connect_callback(uint16_t conn_handle)
{
  Get_client_Diss(conn_handle);
  DEBUG_PRINT("Connecting and checking client_CPS Service ... ");
  // If client_CPS is not found, disconnect, resume scanning, and return
  if ( client_cps.discover(conn_handle) )
  {
    DEBUG_PRINTLN("Found it");
  } else {
    DEBUG_PRINTLN("Not Found!"); 
    DEBUG_PRINTLN("Disconnecting since Client Cyling Power Service is mandatory!");
    // MANDATORY so disconnect since we couldn't find the client_CPS service
    Bluefruit.disconnect(conn_handle);
    // For Softdevice v6: after receiving a report (scan_callback), scanner will be paused (!)
    // We need to call Scanner resume() to continue scanning since we did not find a client_CPS Service!
    // Bluefruit.Scanner.resume();
    return;
  }
 
  DEBUG_PRINT("Discovering client_CP Measurement characteristic ... ");
  if ( !client_cpmc.discover() )
  {
    // Measurement chr is mandatory, if it is not found (valid), then disconnect
    DEBUG_PRINTLN("Not Found!");  
    DEBUG_PRINTLN("Disconnecting since client_CP Measurement Characteristic is mandatory!");
    Bluefruit.disconnect(conn_handle);
    return;
  }
  DEBUG_PRINTLN("Found it");
  
  DEBUG_PRINT("Discovering client_CP Control Point characteristic ... ");
  if ( client_cpcp.discover() )
  {
    // CP Control Point chr is not mandatory
    DEBUG_PRINTLN("Found it");  
  } else
  {
  DEBUG_PRINTLN("Not Found!");  
  }

  DEBUG_PRINT("Discovering Client Wahoo Trainer specific characteristic (CPWT) ... ");
  if ( !client_cpwt.discover() ) //   CYCLING_POWER_WAHOO_TRAINER
  {
    // Is not mandatory in this test phase !!
    DEBUG_PRINTLN("Not Found! NO TRAINER CONTROL!"); 
    client_CPWT_found = false;
    /*
    DEBUG_PRINTLN("Not Found and disconnecting!"); 
    DEBUG_PRINTLN("Wahoo Trainer Characteristic is mandatory!");
    Bluefruit.disconnect(conn_handle);
    return;
    */
  } else {   
    DEBUG_PRINTLN("Found it"); 
    client_CPWT_found = true;
  }

  DEBUG_PRINT("Discovering Client Power Feature characteristic ... ");
  if ( client_cpfc.discover() )
  {
    DEBUG_PRINTLN("Found it");
    
  // Configure the Cycle Power Feature characteristic
  // See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.cycling_power_feature.xml
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
  
  // Read 32-bit client_cpfc value
  const uint8_t CPFC_FIXED_DATALEN = 4;
  client_cpfcDef = client_cpfc.read32();
  uint8_t cpfcData[CPFC_FIXED_DATALEN] = {(uint8_t)(client_cpfcDef & 0xff), (uint8_t)(client_cpfcDef >> 8), 
                                          (uint8_t)(client_cpfcDef >> 16), (uint8_t)(client_cpfcDef >> 24)};
  DEBUG_PRINT("Power Feature 4 bytes: [ ");
  for (int i = 0; i < CPFC_FIXED_DATALEN; i++) {
    if ( i <= sizeof(cpfcData)) {
      DEBUG_PRINTF("%02X ", cpfcData[i], HEX);
    }
  }
  DEBUG_PRINTLN("] ");
  for (int i = 0; i < 20; i++) {
    if ( client_cpfcDef & (1 << i) )
      {
       DEBUG_PRINTLN(Feature_str[i]);
      }
    }
  } else
  {
    DEBUG_PRINTLN("NOT Found!");
  }

  DEBUG_PRINT("Discovering Client Power Sensor Location characteristic ... ");
  if ( client_cplc.discover() )
  {
    DEBUG_PRINTLN("Found it");
  // The  Sensor Location characteristic
  // See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.sensor_location.xml
  // Properties = Read
  // Min Len    = 1
  // Max Len    = 1
  //  B0:1      = UINT16 - Sensor Location
    // power sensor location value is 16 bit
    const char* power_str[] = { "Other", "Top of shoe", "In shoe", "Hip", "Front wheel", "Left crank", "Right crank", "Left pedal",
    "Right pedal", "Front hub", "Rear dropout", "Chainstay", "Rear wheel", "Rear hub", "Chest", "Spider", "Chain ring"};
    // Read 8-bit client_cplc value from peripheral
    client_cplc_loc_value = client_cplc.read8();
    DEBUG_PRINT("Power Location Sensor: ");
    DEBUG_PRINTF("Loc#: %d %s\n", client_cplc_loc_value, power_str[client_cplc_loc_value]);
  } else
  {
    DEBUG_PRINTLN("NOT Found!");
  }
  Client_CPS_IsConnected = true;
} // End client_connect_callback

void Get_client_Diss(uint16_t conn_handle)
{
  // If diss is not found then go on.... NOT FATAL !
  if (!client_diss.discover(conn_handle) ) { return; }
  
  DEBUG_PRINT(F("Found Device Information: \n"));
  char dissDataBuf[20]; // Max len = 20 !!!
    //  1
  if ( client_disma.discover() ) {
      // read and print out Manufacturer
      memset(dissDataBuf, 0, sizeof(dissDataBuf));
      if ( client_disma.read(dissDataBuf, sizeof(dissDataBuf)) ) {
        DEBUG_PRINT("Manufacturer:  ");
        DEBUG_PRINTLN(dissDataBuf);
      }
  }
    //  2
  if ( client_dismo.discover() ) {
      // read and print out Model Number
      memset(dissDataBuf, 0, sizeof(dissDataBuf));
      if ( client_dismo.read(dissDataBuf, sizeof(dissDataBuf)) ) {
        DEBUG_PRINT("Model Number:  ");
        DEBUG_PRINTLN(dissDataBuf);
      }
  }
    //  3
  if ( client_dissn.discover() ) {
      // read and print out Serial Number
      memset(dissDataBuf, 0, sizeof(dissDataBuf));
      if ( client_dissn.read(dissDataBuf, sizeof(dissDataBuf)) ) {
        DEBUG_PRINT("Serial Number: ");
        DEBUG_PRINTLN(dissDataBuf);
      }
  }
} // End DISS

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void client_disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  DEBUG_PRINT("Client Disconnected Peripheral Device, reason = 0x"); DEBUG_PRINTLN(reason, HEX);
}

/**
 * Hooked callback that triggered when a measurement value is sent from peripheral
 * @param chr   Pointer client characteristic that even occurred,
 *              in this example it should be client_cpcp
 * @param data  Pointer to received data
 * @param len   Length of received data
 */
 // https://github.com/sputnikdev/bluetooth-gatt-parser/blob/master/src/main/resources/gatt/characteristic/org.bluetooth.characteristic.cycling_power_control_point.xml

void client_cpcp_indicate_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
/*
static const unsigned MAX_CONTROL_BYTES  = 5;
const uint8_t RESPONSE_SUCCESS = 1;
const uint8_t RESPONSE_OP_CODE_NOT_SUPPORTED = 2;
const uint8_t RESPONSE_INVALID_PARAMETER = 3;
const uint8_t RESPONSE_OPERATION_FAILED = 4;
*/
  uint8_t cpcpDataLen = (uint8_t)len;
  uint8_t cpcpData[cpcpDataLen]= {}; 
  // Transfer first the contents of data to buffer (array of chars)
  DEBUG_PRINT("Raw client Control Point Data: [ "); 
  for (int i = 0; i < cpcpDataLen; i++) {
    if ( i <= sizeof(cpcpData)) {
      cpcpData[i] = *data++;
      DEBUG_PRINTF("%02X ", cpcpData[i], HEX);
    }
  }
  DEBUG_PRINT("] ");
  // NO TREATMENT OF RESPONSE !!!!!
}

/**
 * Hooked callback that triggered when a measurement value is sent from peripheral
 * @param chr   Pointer client characteristic that even occurred,
 *              in this example it should be client_cpmc
 * @param data  Pointer to received data
 * @param len   Length of received data
 */
void client_cpmc_notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{

/*{                            (uint8_t)(cpmcDef & 0xff), (uint8_t)(cpmcDef >> 8),  // flags 
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
  uint8_t cpmcDataLen = (uint8_t)len;
  uint8_t cpmcDataBuf[cpmcDataLen] = {};
  // Transfer first the contents of data to cpmcDataBuf (array of chars)
  DEBUG_PRINT("-> Client Rec'd Raw Wahoo CPS Data: [ "); 
  for (int i = 0; i < cpmcDataLen; i++) {
    if ( i <= sizeof(cpmcDataBuf)) {
      cpmcDataBuf[i] = *data++;
      DEBUG_PRINTF("%02X ", cpmcDataBuf[i], HEX);
    }
  }
  DEBUG_PRINT("] ");
  long PowerValue = 0;
  uint8_t lsb_InstantaneousPower = cpmcDataBuf[2]; // Instantaneous Power LSB
  // POWER is stored in 2 bytes !!!
  uint8_t msb_InstantaneousPower = (cpmcDataBuf[3] & 0xFF); //  
  PowerValue = lsb_InstantaneousPower + msb_InstantaneousPower * 256;
  DEBUG_PRINTF("--> Power Value: %4d\n", PowerValue);
  
  // --------------- For testing control of the trainer ------------------ 
  // Test if it is time to set new simulated Wahoo trainer resistance values
  if (millis() > TimeInterval)
  {
    Grade += 1.0; // Increase the simulated road grade
    Set_SimGrade(Grade); 
    if (Grade > 9.0) {
      delay(150); // Wait for previous action to have completed
      Set_SimMode(); // Repeat every 10 events
      Grade = 0.0;
    }
    TimeInterval = millis() + TIME_SPAN;
  }
  // ----------------For testing control of the trainer ------------------
  
} // End cpmc_notify_callback

/**
 * Hooked callback that triggered when a measurement value is sent from peripheral
 * @param chr   Pointer client characteristic that even occurred,
 *              in this example it should be client_cpmc
 * @param data  Pointer to received data
 * @param len   Length of received data
 */
void client_cpwt_indicate_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
  // The receipt of operation settings is acknowledged by the trainer: handle it
  uint8_t RespBuffer[len] = {}; // Usually it is only 2 bytes long
  // Transfer first the contents of data to buffer (array of chars)
  DEBUG_PRINT("-> Client Rec'd Raw Wahoo Response Data: [ "); 
  for (int i = 0; i < len; i++) {
    if ( i <= sizeof(RespBuffer)) {
      RespBuffer[i] = *data++;
      DEBUG_PRINTF("%02X ", RespBuffer[i], HEX);
    }
  }
  DEBUG_PRINT(" ]");
  switch(RespBuffer[1]) {
    case setSimMode: { 
      DEBUG_PRINT(" --> setSimMode");
      break; }
    case setSimGrade: {
      DEBUG_PRINT(" --> setSimGrade");
      break; }
    case unlock: {
      DEBUG_PRINT(" --> Unlock");
      break; }
    default: {
      DEBUG_PRINT(" --> ? unknown ?");  
    }
  }
  if (  RespBuffer[0] = 0x01 ) {
    DEBUG_PRINTLN(" -> Successful");
  } else {
    DEBUG_PRINTLN(" -> Failed");
  }
}
