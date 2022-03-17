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
 * This is a version that heavily relies on ANT+ FE-C over BLE !!!
 * ONLY the common BLE Device Information Service is used aside of
 * the dominant Tacx FE-C primary service and characteristics for BLE
 */
 
/*
 *  Having a SSD1306 Oled display connected is very practical but NOT essential for
 *  the test proces. If DEBUGSIM is ON (default) the Serial Monitor will receive a constant
 *  stream of messages that detail the progress of advertise, connect and polling processes!
 */
 
 /*
  * This code will do the following
  * 1) Scans for a TACX trainer advertising itself over BLE (trainer must have power and be swithed ON !!),
  * 2) Will try to connect to the trainer if present, if FAILED: see Serial Monitor for debugging info,
  * 3) Search for the FE-C characteristic services to be available,
  * 4) Poll and show the characteristic trainer values (on Oled and/or Serial Monitor),
  * 5) Run (until switched off) a working connection of FE-C over BLE and 
  * 6) Showing road grade percentage and cycling data, when trainer is being loaded in a regular workout!
  * 7) In principle one can run the Feather nRF52 and this test code during a real life indoor workout, 
  *    having Zwift setting the road grade and you sweating on the trainer. This is when a connected 1306 Oled 
  *    display is extremely usefull..... You will see how Zwift and your Feather are completely synched!
  */

 /* --------------------------------------------------------------------------------------------------------------------------------------
  *  NOTICE that many older smart trainer devices allow ANT+ and BLE, however, they only support 1 (ONE) BLE connection at the time, 
  *  which means that in that case you cannot concurrently connect your trainer with ZWIFT AND with the Feather nRF52 over BLE, since it 
  *  is considered a second device and will not connect over BLE. ANT+ supports by definition multiple devices to connect!!!
  *  --> Blame the economical manufacturer and not the messenger!
  *  Solution: Apply ANT+ connection for the regular Trainer-Zwift/PC-link and use the single BLE connection for connecting the Feather nRF52.
  *  I connect in addition my Garmin cycling computer with the trainer over ANT+ and have the Feather nRF52 use the single BLE connection!
  * ---------------------------------------------------------------------------------------------------------------------------------------
  */
  
#include <bluefruit.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <avr/dtostrf.h>

#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 64     // OLED display height, in pixels

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

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1         // No reset pin on this OLED display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//Global variables for measurement data
bool ConnectedToTACX = false;
// RawgradeValue varies between 0 (-200% grade) and 40000 (+200% grade)
// SIMCLINE is mechanically working between -10% and +20% --> 19000 and 22000
// correction for measuring plane difference and midth wheel axis position (2 cm offset is an MEASUREOFFSET of 200)
#define MEASUREOFFSET 100 
// Raw Grade Value Minimally (Mechanically: the lowest position of wheel axis)  19000 is equiv. of -10% road grade
#define RGVMIN 19000 
// Raw Grade Value Maximally (Mechanically: the highest position of wheel axis) 22000 is equiv. of +20% road grade
#define RGVMAX 22000 
// Besides what is mechanically possible there are also limits in what is physically pleasant
// The following Min and Max values should be within the limits of the mechanically feasible values of above !!!
// Minimally Allowed Raw Grade Value that should not be exceeded: -5%!
#define ARGVMIN 19500
// Maximally Allowed Raw Grade Value that should not be exceeded: 15%!
#define ARGVMAX 21500
// set value for a flat road = 0% grade 
// 1000 is a 10% road grade --> added to the minimal position is equiv. of 0% road grade
// result needs to be corrected for the measure offset
long RawgradeValue = (RGVMIN + 1000) - MEASUREOFFSET; 
float gradePercentValue = 0;

// TACX trainer calculated & measured basic cycling data
long PowerValue = 0;
uint8_t InstantaneousCadence = 0;
long SpeedValue = 0;

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
// Boundaries defined by SCALING = 3 of the VL6180X used in class Lifter (not included!)
// Safe Range of at least 30 cm
#define BANDWIDTH 4     // safe range!
#define MINPOSITION 265 // highest value microswitch activated: 267
#define MAXPOSITION 535 // lowest value microswitch activated: 540

// Global variables for Up/Down position control
// set default value for a flat road at start (RawgradeValue is first set accordingly)!
  int16_t TargetPosition = map(RawgradeValue, RGVMIN, RGVMAX, MAXPOSITION, MINPOSITION); 

// COMPILER DIRECTIVE to allow/suppress SERIAL.PRINT
// By declaring DEBUGSIM, it is ON
// Inert DEBUGSIM (switch OFF) by turning "#define DEBUGSIM" into a comment --> comment out
#define DEBUGSIM 

void setup()
{
#ifdef DEBUGSIM
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for feather nrf52832 with native usb
#endif
   
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) 
   { // Address 0x3C for 128x64
#ifdef DEBUGSIM
   Serial.println("SSD1306 allocation failed!");
#endif
   }
  else
   {
#ifdef DEBUGSIM
   Serial.println("Serial Feedback over COM3 Initialized!");
#endif
   // Show Oled with initial display buffer contents on the screen --
   // the library initializes this with a Adafruit splash screen (edit the splash.h in the library).
   display.display();
   delay(500);                    // Pause some time
   }

  // Clear and set Oled display for further use
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);  // Large characters
  display.setCursor(16,22);
  display.print("SIMCLINE"); 
  display.setCursor(16,44);
  display.print("  2.0"); 
  display.display(); 
  delay(500);                    // Pause some time
  
#ifdef DEBUGSIM
  Serial.println("SSD1306 is running...");
#endif  

#ifdef DEBUGSIM
  Serial.println("Bluefruit52 Central ");
  Serial.println("--------------------\n");
#endif
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
#ifdef DEBUGSIM  
  Serial.println("Scanning for TACX ...");
#endif  
  // Show the message on the Oled
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);  // Large characters
  display.setCursor(1,22);
  display.print("Scanning.."); 
  display.display(); 
  delay(200);                    // Pause some time
}

void loop()
{
    // Map RawgradeValue ranging from 0 to 40.000 on the 
    // TargetPosition (between RGVMIN and RGVMAX) of the Linear Actuator
    // Notice 22000 is equivalent to +20% incline and 19000 to -10% incline
    RawgradeValue = constrain(RawgradeValue, RGVMIN, RGVMAX);
    TargetPosition = map(RawgradeValue, RGVMIN, RGVMAX, MAXPOSITION, MINPOSITION);
#ifdef DEBUGSIM
    //Serial.printf("ms: %8d", millis()); Serial.printf("  Raw: %05d ", RawgradeValue, DEC); Serial.printf(" Tpos: %03d", TargetPosition, DEC); Serial.println();
#endif
    // Send a request for Page 51 about every 4 seconds
    if (ConnectedToTACX)
      {
 #ifdef DEBUGSIM
    //  Serial.printf("Connected to TACX? %2X", ConnectedToTACX); Serial.printf("  ms: %6d", millis());
    //  Serial.println();
 #endif
        long tmp = millis()-SendRequestPage51Delay;
 #ifdef DEBUGSIM
    //  Serial.print(" tmp: "); Serial.print(tmp);
 #endif 
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
#ifdef DEBUGSIM  
  //Serial.println("Timestamp Addr              Rssi Data");
  //Serial.printf("%09d ", millis());
    // MAC is in little endian --> print reverse
  //Serial.printBufferReverse(report->peer_addr.addr, 6, ':');
  //Serial.print(" ");
  //Serial.print(report->rssi);
  //Serial.print("  ");
  //Serial.printBuffer(report->data.p_data, report->data.len, '-');
  //Serial.println();
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
  // Show the message on the Oled
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(1, 22);
  display.print("Connecting");
  display.setCursor(1, 42);
  display.print("over BLE!");
  display.display(); 

#ifdef DEBUGSIM
  Serial.println("Connected! Now checking for critical services...");
  Serial.printf("%12s", "Discovering FECPS Service ...\n");
#endif
  // If FECPS is not found, disconnect and return
  if ( !fecps.discover(conn_handle) )
  {
#ifdef DEBUGSIM
    Serial.println("Failed and disconnecting ...\n");
#endif
    // disconnect since we couldn't find crucial service
    Bluefruit.disconnect(conn_handle);
    return;
  }
#ifdef DEBUGSIM
  Serial.print("Found it !\n");
  Serial.printf("%12s", "Discovering FECRD Characteristic ...\n");
#endif
  // If FECRD is not found, disconnect and return
  if ( !fecrd.discover() )
  {
    Serial.println("Failed and disconnecting ...\n");
    // disconnect since we couldn't find crucial service
    Bluefruit.disconnect(conn_handle);
    return;
  }

#ifdef DEBUGSIM
  Serial.print("Found it !\n");
  Serial.printf("%12s", "Discovering FECWR Characteristic ...\n");
#endif
  // If FECWR is not found, disconnect and return
  if ( !fecwr.discover() )
  {
    Serial.println("Failed and disconnecting ...\n");
    // disconnect since we couldn't find crucial service
    Bluefruit.disconnect(conn_handle);
    return;
  }
#ifdef DEBUGSIM
  Serial.print("Found it !\n");
  Serial.printf("%12s", "Discovering Device Information Service ...\n");
#endif
  
/*
 * Now check for the common BLE DIS Service
 */
  // Clear and set Oled display
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // If diss is not found then go on.... NOT FATAL !
  if ( diss.discover(conn_handle) )
  {
 #ifdef DEBUGSIM
    Serial.print("Found it !\n");
 #endif
    char buffer[32+1];
    //  1
    if ( disma.discover() )
    {
    // read and print out Manufacturer
    memset(buffer, 0, sizeof(buffer));
    if ( disma.read(buffer, sizeof(buffer)) )
      {
#ifdef DEBUGSIM
      Serial.printf("%12s", "Manufacturer: ");
      Serial.println(buffer);
#endif
    // Show the message on the Oled     
    // Manufacturer:
      display.setTextSize(2);
      display.setCursor(2, 2);
      display.print(buffer);
      display.display(); 
      }
    }
    //  2
    if ( dismo.discover() )
    {
    // read and print out Model Number
    memset(buffer, 0, sizeof(buffer));
    if ( dismo.read(buffer, sizeof(buffer)) )
      {
#ifdef DEBUGSIM
      Serial.printf("%12s", "Model Number: ");
      Serial.println(buffer);
#endif
      // Show the message on the Oled
      // model number
      display.setTextSize(2);
      display.print(F(" "));
      display.print(buffer);
      display.display(); 
      }
    }
    //  3
    if ( dissn.discover() )
    {
    // read and print out Serial Number
    memset(buffer, 0, sizeof(buffer));
    if ( dissn.read(buffer, sizeof(buffer)) )
      {
#ifdef DEBUGSIM
      Serial.printf("%12s", "Serial Number: ");
      Serial.println(buffer);
#endif
      // Show the message on the Oled
      display.setTextSize(2);
      display.setCursor(24, 22); // 2, 40
      display.print(buffer);
      display.display(); 
      }
    }
    //  4
    if ( disfi.discover() )
    {
    // read and print out Firmware Revision Number
    memset(buffer, 0, sizeof(buffer));
    if ( disfi.read(buffer, sizeof(buffer)) )
      {
#ifdef DEBUGSIM
      Serial.printf("%12s", "Firmware Revision Number: ");
      Serial.println(buffer);
#endif
      // Show the message on the Oled
      display.setTextSize(1);
      display.setCursor(2, 50);
      display.print(F("Fw: "));
      display.print(buffer);
      display.display();     
      }
    }
    //  5   
    if ( disha.discover() )
    {
    // read and print out Harware Revision Number
    memset(buffer, 0, sizeof(buffer));
    if ( disha.read(buffer, sizeof(buffer)) )
      {
#ifdef DEBUGSIM
      Serial.printf("%12s", "Hardware Revision Number: ");
      Serial.println(buffer);
#endif
      // Show the message on the Oled
      display.setTextSize(1);
      display.print(F(" Hw: "));
      display.print(buffer);
      display.display(); 
      }
    }    
    //  6
    if ( disso.discover() )
    {
    // read and print out Software Revision Number
    memset(buffer, 0, sizeof(buffer));
    if ( disso.read(buffer, sizeof(buffer)) )
      {
#ifdef DEBUGSIM
      Serial.printf("%12s", "Software Revision Number: ");
      Serial.println(buffer);
#endif
      // Show the message on the Oled
      display.setTextSize(1);
      display.print(F(" Sw: "));
      display.print(buffer);
      display.display(); 
      }
    }
    // no more characteristics of Device Information Service
  }
  else 
  {   
#ifdef DEBUGSIM
    Serial.println("Found NONE !\n");
#endif
  }
  
  // Oled screen actions
  delay(1000); // pause some time
  
  // ---------------------------------------------------------------------------------
  // Reaching here means we are ready to go, let's enable notification on reading data
  // ---------------------------------------------------------------------------------
  // ANT+ FE-C protocol reading is started now ! -------------------------------------
  // ---------------------------------------------------------------------------------
    if ( fecrd.enableNotify() )
  {
  // To set ALL CLEAR: READY TO ROCK AND ROLL ---> calls handled in loop()
  ConnectedToTACX = true;  
  // 
#ifdef DEBUGSIM
    Serial.println("Ready to receive FE-C messages");
#endif
  }
  else
  {
#ifdef DEBUGSIM
    Serial.println("Couldn't enable notify for FE-C messages");
#endif
  }
} // Done --------------------------------------------

// Funtion to show measurement data: Grade, Power, Cadence and Speed on Oled screen
void ShowValuesOnOled()
  {
  display.clearDisplay(); // clear the oled screen
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(4, 29);
  display.print(F("PWR"));
  display.setCursor(50, 29);
  display.print(F("RPM"));
  display.setCursor(92, 29);
  display.print(F("KPH"));
  display.setCursor(102, 10);
  display.setTextSize(2); 
  display.print(F("%"));

    char grade[7];   
   dtostrf(gradePercentValue, 5, 1, grade); // show sign only if negative
   display.setCursor(10, 6);  
   display.setTextSize(3);
   display.print(grade);

   char pscs[10];
   sprintf(pscs, "%03d %03d %02d", PowerValue, InstantaneousCadence, SpeedValue);
   display.setCursor(1, 44);
   display.setTextSize(2);
   display.print(pscs);       
   display.display(); 
  }

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
#ifdef DEBUGSIM
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
#endif
}

void SendRequestPage51()
{  
#ifdef DEBUGSIM
    Serial.printf("%d  Sending Request for data page 51\n ", SendRequestPage51Event);
#endif
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
#ifdef DEBUGSIM 
  //Serial.printf("Dump of FE-C Data packets [len: %02d]  ", len);
#endif
  for (int i = 0; i < len; i++) 
  {
    if ( i <= sizeof(buffer)) 
   {
    buffer[i] = *data++;
#ifdef DEBUGSIM
    //Serial.printf("%02X ", buffer[i], HEX);
#endif
   }
  }
 
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
      RawgradeValue = lsb_gradeValue + msb_gradeValue*256;
      gradePercentValue = float((RawgradeValue - 20000))/100;
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
      if (RawgradeValue < ARGVMIN) { RawgradeValue = ARGVMIN; } // Do not allow lower values than ARGVMIN !!
      if (RawgradeValue > ARGVMAX) { RawgradeValue = ARGVMAX; } // Do not allow values to exceed ARGVMAX !!
      // End test --> continue ---------------------------------------------------------------------------
#ifdef DEBUGSIM
      Serial.printf("--Page 51 received - RawgradeValue: %05d  ", RawgradeValue); 
      Serial.printf("Grade percentage: %02.1f %%", gradePercentValue);
      Serial.println();
#endif
      }  
    break;
  case 0x19 :
    {
    /////////////////////////////////////////////////
    /////////// Handle PAGE 25 Trainer/Bike Data ////
    /////////////////////////////////////////////////
    uint8_t UpdateEventCount = buffer[5];
    InstantaneousCadence = buffer[6];
    uint8_t lsb_InstantaneousPower = buffer[9];
    // POWER is stored in 1.5 byte !!!
    uint8_t msb_InstantaneousPower = (buffer[10] & 0x0F); // bits 0:3 --> MSNibble only!!!
    PowerValue = lsb_InstantaneousPower + msb_InstantaneousPower*256;
#ifdef DEBUGSIM  
    Serial.printf("Event count: %03d  ", UpdateEventCount); 
    Serial.printf(" - Cadence: %03d  ", InstantaneousCadence);
    Serial.printf(" - Power in Watts: %04d  ", PowerValue);
    Serial.println();
#endif
    }
    break;
  case 0x10 :
    {
    //////////////////////////////////////////////
    //////////// Handle PAGE 16 General FE Data //
    //////////////////////////////////////////////
    uint8_t ElapsedTime = buffer[6];  // units of 0.25 seconds
    uint8_t DistanceTravelled = buffer[7]; // in meters 256 m rollover 
    uint8_t lsb_SpeedValue = buffer[8];
    uint8_t msb_SpeedValue = buffer[9];
    SpeedValue = ((lsb_SpeedValue + msb_SpeedValue*256)/1000)*3.6; // in units of 0,001 m/s naar km/h
#ifdef DEBUGSIM
    Serial.printf("Elapsed time: %05d s  ", ElapsedTime);
    Serial.printf(" - Distance travelled: %05d m ", DistanceTravelled); 
    Serial.printf(" - Speed: %02d km/h", SpeedValue);
    Serial.println();
#endif
    }
    break;
  case 0x80 :
    {
    // Manufacturer Identification Page
    }
  default :
    {
#ifdef DEBUGSIM
    Serial.printf("Page: %2d ", PageValue); Serial.println(" Received");
#endif
    return;
    }
  }
  // Show the actual values of the trainer on the Oled     
  ShowValuesOnOled();
  //////////////////////// DONE! /////////////////////////
}
