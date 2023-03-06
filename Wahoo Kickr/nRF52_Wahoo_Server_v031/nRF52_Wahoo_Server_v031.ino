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
 *  This Feather nRF52 Peripheral/Server code advertises and enables the Wahoo specific 
 *  Cycling Trainer Service: CPS plus the additional Wahoo proprietary characteristic and
 *  allows to connect to Cycling apps like Zwift. It simulates a connected Wahoo KICKR trainer.
 *  Requirements: PC/Laptop with Zwift app or alike and Feather nRF52 board
 *  
 *  1) Upload and Run this code on the Feather nRF52
 *  2) Start the Serial Monitor to catch debugging info
 *  3) Start Zwift and wait for the Devices Pairing Screen
 *  4) Unpair previously paired devices for POWER and CONTROLLABLE
 *  5) Search on Zwift pairing screen for the Feather nRF52 a.k.a. "Wahoo Sim"
 *  6) Pair: Power and Controllable (Wahoo does NOT support Speed nor Cadence)
 *  7) Start the default Zwift ride or any ride you wish
 *  8) Make Serial Monitor visible on top of the Zwift window 
 *  9) Inspect the info presented by Serial Monitor..... 
 *  10) See how your avatar starts moving in the Zwift world
 *  
 */

#include <bluefruit.h>

/* Cycling Power Service
 * CP Service: 0x1818  
 * CP Characteristic: 0x2A63 (Measurement)
 * CP Characteristic: 0x2A65 (Feature)
 * CP Characteristic: 0x2A5D (Location) NOT supported in Wahoo legacy trainers
 * UUID16_CHR_CYCLING_POWER_CONTROL_POINT  0x2A66 ???
 */
BLEService        server_cps = BLEService(UUID16_SVC_CYCLING_POWER);
// Define how Server CPS is advertised
const uint16_t Server_appearance = 0x0480;  // 1152 -> Cycling

BLECharacteristic server_cpmc = BLECharacteristic(UUID16_CHR_CYCLING_POWER_MEASUREMENT);
BLECharacteristic server_cpfc = BLECharacteristic(UUID16_CHR_CYCLING_POWER_FEATURE);

BLECharacteristic server_cplc = BLECharacteristic(UUID16_CHR_SENSOR_LOCATION);
// added the hidden Wahoo Trainer characteristic to the Cycling Power Service !
// Unknown Characteristic is 128 bit:            A0 26 E0 05 - 0A 7D - 4A B3 - 97 FA - F1 50 0F 9F EB 8B
// Declare in Reversed order !!!
uint8_t CYCLING_POWER_WAHOO_TRAINER_Uuid[16] = {0x8B, 0xEB, 0x9F, 0x0F, 0x50, 0xF1, 0xFA, 0x97, 0xB3, 0x4A, 0x7D, 0x0A, 0x05, 0xE0, 0x26, 0xA0};
BLECharacteristic server_cpwt = BLECharacteristic(CYCLING_POWER_WAHOO_TRAINER_Uuid);

// CPS Wahoo Trainer Operation Codes in Decimal
//const uint8_t unlock                     = 32;
const uint8_t setResistanceMode          = 64;
const uint8_t setStandardMode            = 65;
const uint8_t setErgMode                 = 66;
const uint8_t setSimMode                 = 67;
const uint8_t setSimCRR                  = 68;
const uint8_t setSimWindResistance       = 69;
const uint8_t setSimGrade                = 70;
const uint8_t setSimWindSpeed            = 71;
const uint8_t setWheelCircumference      = 72;
//const uint8_t UnlockCommandBuf[3]        = {unlock, 0xEE, 0xFC}; // Unlock codes

BLEDis bledis;    // DIS (Device Information Service) helper class instance

//global cycling definitions for test purposes ONLY!
const uint16_t  MAX_POWER_WATT =         200;
const uint16_t  MIN_POWER_WATT =         100;
uint16_t cps_sim_power_watt = MIN_POWER_WATT; // Power in Watts for simulation
// Timing variables
#define TIME_SPAN 200 // Time span in millis 200 = 0.2 second
unsigned long TimeInterval = 0;


/*
 * Setup
 */
void setup()
{
  Serial.begin(115200);
  while ( !Serial ) delay(100);   // for nrf52840 with native usb, milliseconds

  delay(1000);

  Serial.println("Wahoo Kickr Server with Cycling Power Service only");
  Serial.println("----------  Feather nRF52 Version 031  -----------");

// ------------------------------------------------------
#if defined(ARDUINO_NRF52840_FEATHER)
// Allow for extra memory usage nRF52840 has plenty!
    Bluefruit.configUuid128Count(2); // 1 Service and 1 Char CPWT
    Bluefruit.configCentralBandwidth(BANDWIDTH_HIGH);
    Bluefruit.configPrphBandwidth(BANDWIDTH_HIGH);
    Bluefruit.configPrphConn(BLE_GATT_ATT_MTU_DEFAULT, BLE_GAP_EVENT_LENGTH_DEFAULT * 2, BLE_GATTS_HVN_TX_QUEUE_SIZE_DEFAULT * 2, BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT * 2);
#endif
// -----------------------------------------------------------

#if defined(ARDUINO_NRF52832_FEATHER)
// Squeeze the memory to a minimum... to avoid nRF52832 out off memory errors
    Bluefruit.configUuid128Count(2); // 1 Service and 1 Char CPWT
    Bluefruit.configPrphBandwidth(BANDWIDTH_LOW); 
    Bluefruit.configCentralBandwidth(BANDWIDTH_LOW);
//    Bluefruit.configAttrTableSize(1024); // 1024
#endif

  // Initialise the Bluefruit module
  Serial.println("Initialise the Bluefruit nRF52 module");
  // begin (Peripheral = 1, Central = 0)
  Bluefruit.begin(1, 0);

  // Setup the Server Cycle Power Service
  // BLEService and BLECharacteristic classes initialized
  Serial.println("Configuring the Server Cycle Power Service");
  Setup_Server_CPS();
  // Setup the advertising packet(s)
  Serial.println("Setting up the Server advertising");
  Start_Server_Advertising();
  Serial.println("Advertising! --> Pair with Zwift now ....");  
  while (Bluefruit.Advertising.isRunning()) { // ONLY advertise!
  }
  if ( Bluefruit.connected() ) {
    Serial.println("Ready to Rock and Roll!");
  }
  TimeInterval = millis() + TIME_SPAN; // ADD just enough delay
} // End

/*
 * Functions
 */
void Start_Server_Advertising(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include Server CPS no others
  Bluefruit.Advertising.addService(server_cps);  // defined above

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
  Bluefruit.Advertising.restartOnDisconnect(false); // true is default --> auto restart
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

void Setup_Server_CPS(void)
{
// Default and fixed settings
// Server CPWT Cycling Power Wahoo Trainer Char data field -------------------------------------------------------------------------------------
const uint16_t CPWT_MAX_DATALEN = 20;  // Stay on the safe side: it usually is no longer than 4 or 6 bytes 
// Server CPFC Cycle Power Feature characteristic
const uint16_t CPFC_FIXED_DATALEN = 4;  // Notice: set to fixed for this Characteristic 4 bytes = 32 bits only !!!!!
// Server CPMC Cycling Power Measurement Characteristic
const uint16_t cpmcDef = 0x00; // Cycle power measurement config flags field is zero
const uint16_t CPMC_MAX_DATALEN = 20; // Notice: set to MAX for this Characteristic, the flags field value determines the actual cpmcDataLen !!!!!
const uint16_t Power_out = 50; // Power in Watts ( an arbitrary value, just to start with or to show our presence ;-) )
const uint8_t cpmcDataLen = 4; // First dataset contains cpmcDef = zero and only Instantaneous power data, so len = 4 !!!
const uint8_t cpmcData[cpmcDataLen] = {
                (uint8_t)(cpmcDef & 0xff),  // Flags Field
                (uint8_t)(cpmcDef >> 8),    // Flags Field
                (uint8_t)(Power_out & 0xff),// Instantaneous Power
                (uint8_t)(Power_out >> 8)}; // Instantaneous Power                               
// ---------------------------------------------------------------------------------------------------------------------------------------------
  
  server_cps.begin();

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
  //    B2:3    = SINT16 - Instataineous power, Watts (decimal)
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
  
  server_cpmc.setProperties(CHR_PROPS_NOTIFY);  // type is set to "notify"
  server_cpmc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS); // readAccess, writeAccess
  server_cpmc.setMaxLen(CPMC_MAX_DATALEN); // Notice that this value is set to MAX but the actual length varies!
  server_cpmc.setCccdWriteCallback(server_cccd_callback);  // Optionally capture CCCD updates
  server_cpmc.begin();
  server_cpmc.notify(cpmcData, cpmcDataLen);  

  // Wahoo Trainer hidden characteristic ------------------------------------------------------------------
  server_cpwt.setProperties(CHR_PROPS_INDICATE | CHR_PROPS_WRITE); // Indicate and Write !!
  server_cpwt.setPermission(SECMODE_OPEN, SECMODE_OPEN); // readAccess, writeAccess DO NOT SET: SECMODE_NO_ACCESS !
  server_cpwt.setMaxLen(CPWT_MAX_DATALEN); // The charactersitic's data set varies in length
  server_cpwt.setCccdWriteCallback(server_cccd_callback);  // Optionally capture CCCD updates
  server_cpwt.begin();
  server_cpwt.setWriteCallback(server_cpwt_callback); // Respond to events with "Write with Response" !!

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

  uint32_t cpfcDef = 0x00; // None of the cycling power features are set to TRUE!!!
  uint8_t cpfcData[CPFC_FIXED_DATALEN] = {(uint8_t)(cpfcDef & 0xff), (uint8_t)(cpfcDef >> 8), (uint8_t)(cpfcDef >> 16), (uint8_t)(cpfcDef >> 24)};
  server_cpfc.setProperties(CHR_PROPS_READ);
  server_cpfc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  server_cpfc.setFixedLen(CPFC_FIXED_DATALEN);
  server_cpfc.begin();
  server_cpfc.write(cpfcData, CPFC_FIXED_DATALEN);

  // Configure the Sensor Location characteristic
  // See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.sensor_location.xml
  // Properties = Read
  // Min Len    = 1
  // Max Len    = 1
  //  B0:1      = UINT16 - Sensor Location
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
   
  server_cplc.setProperties(CHR_PROPS_READ);
  server_cplc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  server_cplc.setFixedLen(1);
  server_cplc.begin();
  server_cplc.write8(12);  // Set the characteristic to 'Rear wheel' (12)

    // Set the advertised device name (keep it short!)
  Serial.println("Setting Device Name to 'Wahoo Sim'");
  Bluefruit.setName("Wahoo Sim");
  if (Bluefruit.setAppearance(Server_appearance))
  {
    Serial.printf("Setting Appearance to [%d] Generic: Cycling\n", Server_appearance);
  }

  // Set the connect/disconnect callback handlers
  Bluefruit.Periph.setConnectCallback(server_connect_callback);
  Bluefruit.Periph.setDisconnectCallback(server_disconnect_callback);

//
  // Configure and Start the Device Information Service
  Serial.println("Configuring the Device Information Service");
  bledis.setManufacturer("Wahoo Fitness"); 
  //  Serial.println("Manufacturer: Wahoo Fitness");
  bledis.setModel("KICKR");
  //  Serial.println("Model: KICKR");
  // Notice that the Firmware Revision string is default set to 
  // the value of the Feather-nRF52 Board being used!
  bledis.setFirmwareRev("0.0.1");
  //  Serial.println("Rev Str: 0.0.1");
  bledis.setSerialNum("1234");
  //  Serial.println("Serial Num: 1234");
  bledis.setHardwareRev("0.0.1");
  //  Serial.println("Hardware Rev: 0.0.1");
  bledis.setSoftwareRev("0.0.1");
  //  Serial.println("Software Rev: 0.0.1");
  bledis.begin();
    Serial.println("Done!");
//
} // End

void server_cpwt_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  // Server CPWT Cycling Power Wahoo Trainer data field
  uint8_t cpwtDataLen = 20;  // Stay on the safe side: it usually is no longer than 7 bytes 
  uint8_t cpwtData[20];

  cpwtDataLen = (uint8_t)len;    // OperationCode plus min 2 and max 6 data bytes (uint8_t)
  memset(cpwtData, 0, cpwtDataLen); //sizeof(cpwtData)); // Size was defined at setup, it is NOT dynamic
  // Display the raw request packet actual length
  Serial.printf("Server CPWT Data [Len: %d] [Data:  ", len);
  // Transfer the contents of data to cpwtData
  for (int i = 0; i < cpwtDataLen; i++) {
    cpwtData[i] = *data++;
    // Display the raw request packet byte by byte in HEX
    Serial.printf("%02X ", cpwtData[i], HEX);
  }
  Serial.print("] "); 

  // The documentation I found states that all write actions to this CPWT characteristic are "Write with Response"
  // So we have formally to acknowledge the receipt of the setting
  // Zwift does NOT care at all if one comments this response code part it is still working !!!
  uint8_t RespBuffer[2] = {};
  RespBuffer[0] = 0x01; // set for success = 1
  RespBuffer[1] = cpwtData[0]; // return the received opCode
  server_cpwt.indicate(RespBuffer, 2);
  
  // do something ---------------- with the raw data ---------
  // Notice that the Unlock command is {0x20, 0xEE, 0xFC} --> is NOT treated
  if (cpwtData[0] == setSimMode) {
    uint16_t tmp = ( cpwtData[1] + (cpwtData[2]*256) );  // This works perfect !!!
    float weight = (float(tmp) / 100); // Rider weight in Kg
    tmp = ( cpwtData[3] + (cpwtData[4]*256) );  // This works perfect !!!
    float rrc = (float(tmp) / 1000);    // Rolling Resistance Coefficient
    tmp = ( cpwtData[5] + (cpwtData[6]*256) );  // This works perfect !!!
    float wrc = (float(tmp) / 1000);    // Wind Resistance Coefficient
    Serial.printf(" SimMode:  Weight: %0.1f RRC: %f WRC: %f", weight, rrc, wrc);
  }
  if (cpwtData[0] == setSimGrade) {
    uint16_t gr = ( cpwtData[1] + (cpwtData[2]*256) ); // This works perfect !!!
    float SimGrade = 100 * float( ((gr * 2.0 / 65535) - 1.0) ); // Percentage of road grade --> range: between +1 and -1 (!)
    Serial.printf(" SimGrade: %4.1f%%", SimGrade); // 
  }
  Serial.println();
  //SetNewPowerValue(); // To simulate cycling
}

void PrintPeerAddress(uint8_t addr[6])
{
  for (int i = 1; i < 6; i++) {
      // Display byte by byte in HEX reverse: little Endian
//    if ( i <= sizeof(addr)) {
      Serial.printf("%02X:",addr[(6-i)], HEX);
//    }
  }
   Serial.printf("%02X ",addr[0], HEX);
}

void server_connect_callback(uint16_t conn_handle)
{
  char central_name[32] = {0};
  uint8_t central_addr[6] = {0};
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);
  connection->getPeerName(central_name, sizeof(central_name));
  ble_gap_addr_t peer_address = connection->getPeerAddr();
  memcpy(central_addr, peer_address.addr, 6);
  Serial.printf("Feather nRF52 (Peripheral) connected to Laptop/PC (Central) device: %s MAC Address: ", central_name);
  PrintPeerAddress(central_addr);
  Serial.println();
  Serial.println("Waiting for Central Device to set CCCD Notify (enable) and start reading....");
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

  Serial.print("Server Disconnected from Central Device reason = 0x"); Serial.println(reason, HEX);
  Serial.println("Stopped, reset and start again!");
}

void server_cccd_callback(uint16_t conn_handle, BLECharacteristic* chr, uint16_t cccd_value)
{
    // When changed, display the Notify Status for all NOTIFY charcteristics
    Serial.printf("Server CCCD Updated to: [%d] --> ", cccd_value);
    // Check the characteristic UUID this CCCD callback is associated with,
    // in case this handler is used for multiple CCCD records.
    if (chr->uuid == server_cpmc.uuid) { 
        if (chr->notifyEnabled(conn_handle)) { 
            Serial.print("Server CPMC 'Notify' enabled");
        } else {
            Serial.print("Server CPMC 'Notify' disabled");
        }
    }
    if (chr->uuid == server_cpwt.uuid) { 
        if (chr->indicateEnabled(conn_handle)) { // was notify --> WRONG!!!
            Serial.print("Server CPWT 'Indicate' enabled");
        } else {
            Serial.print("Server CPWT 'Indicate' disabled");
        }
    }
    Serial.println();
} // end CCCD callback

// To simulate you are cycling
void SetNewPowerValue(void)
{
// Set new power value and send to Server: for testing ONLY
//----------------------------------------------
    // Simulate increasing Power generation
    cps_sim_power_watt += 1; // In steps of 1 watt!
    if (cps_sim_power_watt >= MAX_POWER_WATT) {
         cps_sim_power_watt = MIN_POWER_WATT;
    }
//--------------------------------------------------
// Handle Power measurement --------------------
    uint16_t cpmcDef = 0;
    uint8_t cpmcDataLen = 4;
    uint8_t cpmcData[cpmcDataLen] = {};
    cpmcData[0] = (uint8_t)(cpmcDef & 0xff); // flags
    cpmcData[1] = (uint8_t)(cpmcDef >> 8);   // flags 
    cpmcData[2] = (uint8_t)(cps_sim_power_watt & 0xff);// inst. power 
    cpmcData[3] = (uint8_t)(cps_sim_power_watt >> 8);  // inst. power                                   
    // If it is connected but CCCD is not enabled
    // The characteristic's value is still updated although notification is not sent
    if ( server_cpmc.notify(cpmcData, cpmcDataLen) ){
      Serial.print("Server Cycle Power Measurement updated to: "); Serial.println(cps_sim_power_watt); 
    }
// Handle Power measurement --------------------
    // Set time interval to wait for next setting
    TimeInterval = millis() + TIME_SPAN; // Set new delay
}

void loop()
{
    digitalToggle(LED_RED); 
    // wait for TIME_SPAN expired
    if ( millis() > TimeInterval )  { 
      SetNewPowerValue();
    }
} // END of Loop and program
