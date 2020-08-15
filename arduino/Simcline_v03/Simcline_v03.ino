/*******************************************************************************
  This is an application for the Adafruit Feather nRF52 (Bluefruit Bluetooth LE)
  --> Nordic Semiconductor SoftDevice installed: S132
 *******************************************************************************/

/*  This sketch heavily uses the BLEClientService and BLEClientCharacteristic of the
    Bluefruit library to implement a custom client (a.k.a. CENTRAL) that is used to listen
    and talk with a Gatt server on the FE-C capable indoor cycling trainer. In our case a
    TACX Indoor Bike Trainer of type NEO... a so called "smart trainer" that is capable
    of working with BLE and ANT+ using the standard FE-C protocol for this type of equipment.

    In addition a mobile phone can be (optionally) connected over BLE UART, (a) to set
    critical and persistent values that constrain high level functions, (b) to manually
    control, up and down, movement (in absence of a connected trainer), and (c) to present
    the most relevant real time cycling data (from the trainer) on the display of the mobile phone...
    In analogy to Zwift you need to download and install the SIMCLINE Companion App on your phone!

    Note: you need a TACX Trainer AND/OR Mobile Phone to exploit this sketch to its max.
    Stand alone (i.e. with no BLE connectable peripherals) the sketch tests well functioning and
    generates messages on the " internal" Oled display to signal its findings and status. Even when
    the Actuator is not connected to 12 V power the sketch is in control and visibly functioning....
*/

#include <bluefruit.h>
// Libraries for use of I2C devices (Oled and VL6180X distance sensor)
#include <SPI.h>
#include <Wire.h>

// Necessary libraries for use of Oled display(s)
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// Additional splash screen bitmap and icon(s) for Oled
#include "Adafruit_SSD1306_Icons.h" // needs to be in directory of main code
// Declarations for an SSD1306 128x64 display connected to I2C (SDA, SCL pins)
#define SCREEN_WIDTH 128            // SSD1306-OLED display width, in pixels
#define SCREEN_HEIGHT 64            // SSD1306-OLED display height, in pixels
#define OLED_RESET -1               // No reset pin on this OLED display
#define SSD1306_I2C_ADDRESS 0x3C    // I2C Address for SSD1306-OLED display
// Declare the display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// Declare Global var for OLED Display selection 1 (Cycling data) or 2 (Road Grade)
uint8_t OledDisplaySelection = 1; // default cycling data to show
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

// Library code for low level measuring (VL6180X) and controlling UP and down movement
#include <Lifter.h>

/*
   Thisketch heavily relies on ANT+ FE-C over BLE, see documentation!!!
   ONLY the common BLE Device Information Service is used aside of
   the dominant Tacx FE-C primary service and characteristics for BLE!
*/

///////////////////////////////////////////////
/////////// TAXC FE-C ANT+ over BLE ///////////
///////////////////////////////////////////////
//
//Tacx FE-C primary service and characteristics Uuid's need special treatment !!
//
//TACX_FEC_PRIMARY_SERVICE UUID is 128 bit:    6E 40 FE C1 - B5 A3 - F3 93 - E0 A9 - E5 0E 24 DC CA 9E
// Declare in Reversed order !!!
uint8_t TACX_FEC_PRIMARY_SERVICE_Uuid[16] =     {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0xC1, 0xFE, 0x40, 0x6E,};
//TACX_FEC_READ_CHARACTERISTIC is 128 bit:     6E 40 FE C2 - B5 A3 - F3 93 - E0 A9 - E5 0E 24 DC CA 9E
// Declare in Reversed order !!!
uint8_t TACX_FEC_READ_CHARACTERISTIC_Uuid[16] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0xC2, 0xFE, 0x40, 0x6E,};
//TACX_FEC_WRITE_CHARACTERISTIC is 128 bit:    6E 40 FE C3 - B5 A3 - F3 93 - E0 A9 - E5 0E 24 DC CA 9E
// Declare in Reversed order !!!
uint8_t TACX_FEC_WRITE_CHARACTERISTIC_Uuid[16] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0xC3, 0xFE, 0x40, 0x6E,};
//
///////////////////////////////////////////////
/////////// TAXC FE-C ANT+ over BLE ///////////
///////////////////////////////////////////////
/*
    Declare the BLE DIS Service and Characterics Uuid's
*/
#define SERVICE_DEVICE_INFORMATION                  0x180A
#define CHARACTERISTIC_MANUFACTURER_NAME_STRING     0x2A29
#define CHARACTERISTIC_MODEL_NUMBER_STRING          0x2A24
#define CHARACTERISTIC_SERIAL_NUMBER_STRING         0x2A25
#define CHARACTERISTIC_FIRMWARE_REVISION_STRING     0x2A26
// ---------------------------------------------------------
// COMPILER DIRECTIVE to allow/suppress additional DIS characteristics to be defined and processed
//#define DEBUG_DIS_EXT
// --------------------------------------------------------------------------------------------
#ifdef DEBUG_DIS_EXT
#define CHARACTERISTIC_HARDWARE_REVISION_STRING     0x2A27
#define CHARACTERISTIC_SOFTWARE_REVISION_STRING     0x2A20
#endif
/* Declare crucial services and charateristics for TACX FE-C trainer

*/
BLEClientService        fecps(TACX_FEC_PRIMARY_SERVICE_Uuid);
BLEClientCharacteristic fecrd(TACX_FEC_READ_CHARACTERISTIC_Uuid);
BLEClientCharacteristic fecwr(TACX_FEC_WRITE_CHARACTERISTIC_Uuid);
/* Declare additional services and characteristics for Device Information Service DIS

*/
BLEClientService        diss(SERVICE_DEVICE_INFORMATION);
BLEClientCharacteristic disma(CHARACTERISTIC_MANUFACTURER_NAME_STRING);
BLEClientCharacteristic dismo(CHARACTERISTIC_MODEL_NUMBER_STRING);
BLEClientCharacteristic dissn(CHARACTERISTIC_SERIAL_NUMBER_STRING);
BLEClientCharacteristic disfi(CHARACTERISTIC_FIRMWARE_REVISION_STRING);
#ifdef DEBUG_DIS_EXT
BLEClientCharacteristic disha(CHARACTERISTIC_HARDWARE_REVISION_STRING);
BLEClientCharacteristic disso(CHARACTERISTIC_SOFTWARE_REVISION_STRING);
#endif

// Declare bleuart for UART communication over BLE with mobile Phone
BLEUart bleuart;
bool ConnectedToMobilePhone = false;
// -------------------------------------------------------------

// Global variable definitions for high level movement control
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
// Minimally Allowed Raw Grade Value that should not be exceeded: -5%! -> Descent grade Limit
int aRGVmin = 19500;
// Maximally Allowed Raw Grade Value that should not be exceeded: 15%! -> Ascent grade limit
int aRGVmax = 21500;
// set value for a flat road = 0% grade
// 1000 is a 10% road grade --> added to the minimal position is equiv. of 0% road grade
// result needs to be corrected for the measure offset
long RawgradeValue = (RGVMIN + 1000) - MEASUREOFFSET;
int GradeChangeFactor = 100; // 100% means no effect, 50% means only halved up/down steps --> Road Grade Change Factor
// Grade of a road is defined as a measure of the road's steepness as it rises and falls along its route
float gradePercentValue = 0;
//-----------------------------------------------------------------

// TACX trainer specific calculated & measured basic cycling data
long PowerValue = 0;
uint8_t InstantaneousCadence = 0;
float SpeedValue = 0;
long AccumulatedElapsedTime = 0;
uint8_t PreviousETValue = 0;
float AccumulatedDistanceTravelled = 0;
uint8_t PreviousDTValue = 0;
bool IsTrainerMoving = false;
//--------------------------------------------------------------------

//Define the FE-C ANT+ Request Page #51 Command
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
  0x47
}; //Checksum;

// FE-C ANT+ Page 51 globals
unsigned long SendRequestPage51Delay = 4000; //Sample rate for Page 51 requests every 4 seconds
unsigned long SendRequestPage51Event = millis(); //Millis of last Page 51 request event

// Feather nRF52 I/O Pin declarations for connection to Motor driver board MDD3A
#define actuatorOutPin1 2    // --> A0/P0.02 connected to pin M1A of the MDD3A Motor Driver board
#define actuatorOutPin2 3    // --> A1/P0.03 connected to the M1B of the MDD3A Motor Driver board

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

// Decalaration of Lifter Class for control of the low level up/down movement
Lifter lift;
// Global variables for LIFTER position control --> RawGradeValue has been defined/set previously!!
int16_t TargetPosition = map(RawgradeValue, RGVMIN, RGVMAX, MAXPOSITION, MINPOSITION);
bool IsBasicMotorFunctions = false; // Mechanical motor functions
//
// Declaration of Function Prototypes -----------------------------------------------
bool getPRSdata(void);
void setPRSdata(void);
void prph_connect_callback(uint16_t conn_handle);
void prph_disconnect_callback(uint16_t conn_handle, uint8_t reason);
void prph_bleuart_rx_callback(uint16_t conn_handle);
void prph_bleuart_TX_Grade(void);
void prph_bleuart_TX_PWR_CAD(void);
void prph_bleuart_TX_ADT_SPD_AET(void);
void scan_stop_callback(void);
void adv_stop_callback(void);
void scan_callback(ble_gap_evt_adv_report_t* report);
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void SendRequestPage51(void);
void fecrd_notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len);
void SetNeutralValues(void);
bool ControlUpDownMovement(void);
void ShowOnOledLarge(char *Line1, char *Line2, char *Line3, uint16_t Pause);
void BuildBasicOledScreen(void);
void ShowValuesOnOled(void);
void ShowSlopeTriangleOnOled(void);

// ---------------------------------------------------------------------------------------
//
// -------------------------------------------------------------------------------------------
// COMPILER DIRECTIVE to allow/suppress SERIAL.PRINT messages that help debugging...
// Uncomment to activate
// #define DEBUGSIM
// --------------------------------------------------------------------------------------------

void setup()
{
#ifdef DEBUGSIM
  Serial.begin(115200);
  while ( !Serial ) {
    delay(10); // for Feather nRF52 with native usb
  }
#endif

  // LittleFS start the Littlefilesystem lib and see if we have persistent data ----
  InternalFS.begin();
  // WARNING --------- Uncomment only when strictly necessary!!! ---------
  //  InternalFS.format();
  //#ifdef DEBUGSIM
  //  Serial.println("Wipe out all persistent data, including file(s)....");
  //#endif
  // WARNING --------- Uncomment only when strictly necessary!!! ---------
  // Get or set (first time only) the values of relevant and crucial variables
  // to persistence, whith the Companion App the user can set these on the fly!
  // get or set the values of aRGVmax, aRGVmin, GradeChangeFactor in PRSdata.
  if (!getPRSdata()) {
    setPRSdata();
  }
  // LittleFS------------------------------------------------------------------------

  // Start the show for the SSD1306 Oled display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS)) {
#ifdef DEBUGSIM
    Serial.println(F("SSD1306 allocation failed!"));
#endif
  }
  else {
#ifdef DEBUGSIM
    Serial.println(F("SSD1306 is running..."));
#endif
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
  ShowOnOledLarge("", "SIMCLINE", "2.1", 500);

  // Initialize Lifter Class data, variables, test and set to work !
  lift.Init(actuatorOutPin1, actuatorOutPin2, MINPOSITION, MAXPOSITION, BANDWIDTH);

  // Test Actuator and VL8106X for proper functioning
  ShowOnOledLarge("Testing", "Up & Down", "Functions", 100);
  if (!lift.TestBasicMotorFunctions()) {
    ShowOnOledLarge("Testing", "Functions", "Failed!", 500);
    IsBasicMotorFunctions = false; // Not working properly
  }
  else {
    ShowOnOledLarge("Testing", "Functions", "Succes!", 500);
    // Is working properly
    IsBasicMotorFunctions = true;
    // Put Simcline in neutral: flat road position
    SetNeutralValues(); // set relevant flat road values
    while (ControlUpDownMovement()) { // wait until flat road position is reached
    }
  }

  //---------------------------------------------------------------------------------------------
  //Enable and setup connections over BLE, first with Tacx Neo and then try to connect smartphone
  //---------------------------------------------------------------------------------------------
#ifdef DEBUGSIM
  Serial.println(F("Bluefruit-nRF52 Central Simcline v 2.0"));
  Serial.println(F("----------------------------------\n"));
#endif
  // TACX trainer connection is of type Central BUT for Smartphone connection add 1 Peripheral !!!!
  // Initialize Bluefruit with maximum connections as Peripheral = 1, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(1, 1);
  Bluefruit.setTxPower(4); // Check bluefruit.h for supported values
  Bluefruit.setName("Bluefruit-nRF52");

  // Set the LED interval for blinky pattern on BLUE LED during Advertising
  Bluefruit.setConnLedInterval(250);

  // Declare Callbacks for Peripheral (smartphone connection)
  Bluefruit.Periph.setConnectCallback(prph_connect_callback);
  Bluefruit.Periph.setDisconnectCallback(prph_disconnect_callback);

  // Callbacks for Central (trainer connection)
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.setStopCallback(scan_stop_callback);

  /* Setup Central Scanning for an advertising TACX trainer...
     - Enable auto scan if disconnected
     - Interval = 100 ms, window = 80 ms
     - Filter only to accept CPS service
     - We use active scan
  */
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.filterRssi(-70);      // original value of -80 , we want to scan only nearby peripherals, so get close to your TACX trainer !!
  Bluefruit.Scanner.setInterval(160, 80); // in units of 0.625 ms
  /*
    We are only interested in the services of the TACX Trainer
  */
  Bluefruit.Scanner.filterUuid(TACX_FEC_PRIMARY_SERVICE_Uuid);
  Bluefruit.Scanner.useActiveScan(true);  // ...
  // Initialize TACX FE-C trainer services and characteristics
  fecps.begin();
  fecrd.begin();
  fecwr.begin();
  // set up callback for receiving ANT+ FE-C packets; this is the main work horse!
  fecrd.setNotifyCallback(fecrd_notify_callback);
  // Initialize DISS client.
  diss.begin();
  // Initialize some characteristics of the Device Information Service.
  disma.begin();
  dismo.begin();
  dissn.begin();
  disfi.begin();
#ifdef DEBUG_DIS_EXT
  disha.begin(); // Turn out to be empty with Tacx
  disso.begin(); // Turn out to be empty with Tacx
#endif
  // ---------------  All initialized --> Start the actual scanning   -------------
#ifdef DEBUGSIM
  Serial.println(F("Scanning for TACX ..."));
#endif
  // Show Scanning message on the Oled
  ShowOnOledLarge("Scanning", "for", "Trainer", 500);
  Bluefruit.Scanner.start(300); // 0 = Don't stop scanning or after n, in units of hundredth of a second (n/100)
  while (Bluefruit.Scanner.isRunning()) { // do nothing else but scanning....
  }

#ifdef DEBUGSIM
  Serial.println(F("Use Simcline Companion App on your Phone!"));
#endif
  // Initialize and setup BLE Uart functionality for connecting to smartphone
  bleuart.begin();
  bleuart.setRxCallback(prph_bleuart_rx_callback);
  // Advertising packet construction
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  // Include the BLE UART (AKA 'NUS') 128-bit UUID
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.Advertising.setStopCallback(adv_stop_callback);
  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  // A.K.A. secondary response packet
  Bluefruit.ScanResponse.addName();
  /* Declare further advertising properties
     - Enable auto advertising if disconnected
     - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
     - Timeout for fast mode is 30 seconds
     - Start(timeout) with timeout = 0 will advertise forever (until connected)
  */
  Bluefruit.Advertising.restartOnDisconnect(true);  // Initiative (autoreconnect) with smartphone
  // but peripheral must advertise!
  Bluefruit.Advertising.setInterval(32, 244);       // in units of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);         // number of seconds in fast mode
  // Start advertising: to be picked up by a Smartphone with the Companion App!
#ifdef DEBUGSIM
  Serial.println(F("Start Advertising..."));
#endif
  Bluefruit.Advertising.start(60);                  // 0 = Don't stop advertising or after n (!) seconds -> 1 minuut
}

void loop()
{ // Do not use ... !!!
  // -------------------------------------------------------
  // The callback functions are dominating completely the
  // processing and loop() would never have been called,
  // since there is a constant stream of FE-C packets that
  // are coming in! fecrd_notify_callback does the bulk of the work!
  // -------------------------------------------------------
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
#ifdef DEBUGSIM
    Serial.print(F("Get & Set PRSdata to "));
    Serial.printf("Max: %d Min: %d Perc.: %d Displ.: %d\n", aRGVmax, aRGVmin, GradeChangeFactor, OledDisplaySelection);
#endif
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
#ifdef DEBUGSIM
    Serial.print(F("Set new values of PRSdata in persistent storage: "));
    Serial.println(buffer);
#endif
  }
}

// LittleFS --------------------------------------------------

/*------------------------------------------------------------------*/
/* Peripheral callback functions
  ------------------------------------------------------------------*/
void prph_connect_callback(uint16_t conn_handle) {
  // stop advertising... now that we are connected to avoid messing up until timeout!
  if (Bluefruit.Advertising.isRunning()) {
    Bluefruit.Advertising.stop();
  }
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);
  char peer_name[32] = { 0 };
  connection->getPeerName(peer_name, sizeof(peer_name));
  ConnectedToMobilePhone = true;
#ifdef DEBUGSIM
  Serial.print(F("[Prph] Connected to "));
  Serial.println(peer_name);
#endif
  peer_name[10] = 0; // only 11 char long allowed
  // Show the message on the Oled
  ShowOnOledLarge("Paired", "with", peer_name, 500);
  // nRF52 code needs about 750 ms to setup an UART Peripheral Service with
  // a working TX & RX connection, before we can use it for transmission !
  while (!bleuart.notifyEnabled()) {
  } // !!!! wait until ready !!!!
  // Send persistent values to Mobile Phone for correct settings!
  // recalculate the values for use on the Phone
  char TXpacketBuffer[16] = { 0 };
  int iMax, iMin, iPerc, iDispl;
  // set aRGVmax to Ascent Grade Limit in whole number
  iMax = map(aRGVmax, 20000, 22000, 0, 20);
  // set aRGVmin to Descent Grade Limit in whole number
  iMin = map(aRGVmin, 19000, 20000, 10, 0);
  // set GradeChangeFactor to Road Grade Change Factor
  iPerc = GradeChangeFactor;
  iDispl = OledDisplaySelection;
  sprintf(TXpacketBuffer, "!S%d;%d;%d;%d;", iMax, iMin, iPerc, iDispl);
  // send these persistent data to the Settings page on the smartphone
  bleuart.print(String(TXpacketBuffer));
  //
#ifdef DEBUGSIM
  Serial.print(F("Persistent data sent to Phone: ")); Serial.println(TXpacketBuffer);
#endif
}

void prph_disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void) conn_handle;
  (void) reason;
  ConnectedToMobilePhone = false;
  // Show the message on the Oled
  ShowOnOledLarge("Phone", "Connection", "Lost!", 500);
#ifdef DEBUGSIM
  Serial.println(F("[Prph] Disconnected"));
#endif
}

void prph_bleuart_rx_callback(uint16_t conn_handle) {
  (void) conn_handle;
  // Read data received over BLE Uart from Mobile Phone
  char RXpacketBuffer[20 + 1] = { 0 };
  bleuart.read(RXpacketBuffer, 20);
#ifdef DEBUGSIM
  Serial.print(F("[Prph] RX: "));
  Serial.println(RXpacketBuffer);
#endif
  // The following routines parse and process the incoming commands
  // Every RXpacket starts with a '!' otherwise corrupt/invalid
  if (RXpacketBuffer[0] != '!') {
#ifdef DEBUGSIM
    Serial.println(F("[Prph] RX: packet does not start with a ! "));
#endif
    return; // invalid RXpacket: do not further parse and process
  }
  // RXpacket buffer has IdCode = "S"
  if (RXpacketBuffer[1] == 'S') { // Settings packet
    // Besides what is mechanically possible there are also limits in what is physically pleasant/comfortable
    // The allowed Raw Grade Value min and max values should be within the limits of the mechanically feasible values !!!
    // Minimally allowed Raw Grade Value that should not be exceeded: -5%!
    // default: aRGVmin is default set to 19500
    // Maximally allowed Raw Grade Value that should not be exceeded: 15%!
    // default: aRGVmax is default set to: 21500
    // New Settings values have arrived --> parse, set values and store persistently
    uint8_t iMax = 0, iMin = 0, iPerc = 0, iDispl = 0;
    sscanf(RXpacketBuffer, "!S%d;%d;%d;%d;", &iMax, &iMin, &iPerc, &iDispl);
    // set Ascent Grade Limit to aRGVmax
    aRGVmax = map(iMax, 0, 20, 20000, 22000);
    // set Descent Grade Limit to aRGVmin
    aRGVmin = map(iMin, 10, 0, 19000, 20000);
    // set Road Grade Change Factor
    GradeChangeFactor = iPerc;
    // set OledDisplaySelection
    OledDisplaySelection = iDispl;
    // LittleFS for persistent storage of these values
    setPRSdata();
    // LittleFS --------------------------------------
#ifdef DEBUGSIM
    Serial.print(F("RX Settings Max: ")); Serial.print(aRGVmax);
    Serial.print(F(" Min: ")); Serial.print(aRGVmin);
    Serial.print(F(" Perc: ")); Serial.print(iPerc);
    Serial.print(F(" Displ: ")); Serial.println(iDispl);
#endif
    // Confirm to the PHONE: settings rcvd and set to persistent
    // send message to phone
    bleuart.print("!SSet & Stored!;");
    return; // Settings rcvd and set to persistent
  }
  // Manual Control Buttons Up Down get parsed and processed!
  // ONLY when the Actuator plus sensor are working well!
  // i.e. low level up/down movement functions work !!
  if (RXpacketBuffer[1] == 'U' && IsBasicMotorFunctions) {
#ifdef DEBUGSIM
    Serial.println("Set UPward moving!");
#endif
    RawgradeValue = RawgradeValue + 100;
    while (ControlUpDownMovement()) { // Up activated
    }
    return;
  } else { // send message to phone
    if (RXpacketBuffer[1] == 'U') {
      bleuart.print("!UOut of Order!;");
      return;
    }
  }
  if (RXpacketBuffer[1] == 'D' && IsBasicMotorFunctions) {
#ifdef DEBUGSIM
    Serial.println("Set DOWNward moving!");
#endif
    RawgradeValue = RawgradeValue - 100;
    while (ControlUpDownMovement()) { // Down activated
    }
    return;
  } else { // send message to phone
    if (RXpacketBuffer[1] == 'D') {
      bleuart.print("!DOut of Order!;");
    }
  }
}

void prph_bleuart_TX_Grade(void) {
  char TX_GRD_Str[9] = { 0 };
  sprintf(TX_GRD_Str, "!G%.1f;", gradePercentValue);
#ifdef DEBUGSIM
  Serial.print(F("Bleuart TX Grade: ")); Serial.printf("|%s| Ismoving: %2X", TX_GRD_Str, IsTrainerMoving);
#endif
  if (IsTrainerMoving) {
    // Needs Type Casting to String for UART
    bleuart.print(String(TX_GRD_Str));
  } else {
    // do nothing!
  }
  return;
}

void prph_bleuart_TX_PWR_CAD(void) {
  char TX_PC_Str[20 + 1] = { 0 };
  sprintf(TX_PC_Str, "!C%d;%d;", PowerValue, InstantaneousCadence);
#ifdef DEBUGSIM
  Serial.print(F("Bleuart TX PC: ")); Serial.printf("%d - |%s| IsMoving: %2X\n", strlen(TX_PC_Str), TX_PC_Str, IsTrainerMoving);
#endif
  if (IsTrainerMoving) {
    // Needs Type Casting to String for UART
    bleuart.print(String(TX_PC_Str));
  } else {
    bleuart.print("!CPaused..;");
  }
  return;
}

void prph_bleuart_TX_ADT_SPD_AET(void) {
  char TX_ADT_SPD_AET_Str[20 + 1] = { 0 };
  char StrTime[10] = { 0 } ;
  sprintf(StrTime, "%02d:%02d", ((AccumulatedElapsedTime / 4) / 3600), ((AccumulatedElapsedTime / 4) % 3600 / 60));
  sprintf(TX_ADT_SPD_AET_Str, "!A%.1f;%.1f;%s;", (AccumulatedDistanceTravelled / 1000), SpeedValue, StrTime);
#ifdef DEBUGSIM
  Serial.print(F("Bleuart TX ADT-SPD-AET: ")); Serial.printf("%d - |%s| IsMoving: %2X\n", strlen(TX_ADT_SPD_AET_Str), TX_ADT_SPD_AET_Str, IsTrainerMoving);
#endif
  if (IsTrainerMoving) {
    // Needs Type Casting to String for UART
    bleuart.print(String(TX_ADT_SPD_AET_Str));
  } else {
    bleuart.print("!APaused.;");
  }
  return;
}

void SetNeutralValues(void) {
  RawgradeValue = (RGVMIN + 1000) - MEASUREOFFSET;
  TargetPosition = map(RawgradeValue, RGVMIN, RGVMAX, MAXPOSITION, MINPOSITION);
  lift.SetTargetPosition(TargetPosition);
}

bool ControlUpDownMovement(void) {
  // Handle mechanical movement i.e. wheel position in accordance with Road Inclination
  // Map RawgradeValue ranging from 0 to 40.000 on the
  // TargetPosition (between MINPOSITION and MAXPOSITION) of the Lifter
  // Notice 22000 is equivalent to +20% incline and 19000 to -10% incline
  RawgradeValue = constrain(RawgradeValue, RGVMIN, RGVMAX); // Keep values within the safe range
  TargetPosition = map(RawgradeValue, RGVMIN, RGVMAX, MAXPOSITION, MINPOSITION);
#ifdef DEBUGSIM
  Serial.println(); Serial.printf("RawgradeValue: %05d ", RawgradeValue, DEC); Serial.printf(" TargetPosition: %03d", TargetPosition, DEC);
#endif
  lift.SetTargetPosition(TargetPosition);
  int OnOffsetAction = lift.GetOffsetPosition(); // calculate offset and determine which action is needed
  switch (OnOffsetAction)
  {
    case 0 :
      lift.brakeActuator();
#ifdef DEBUGSIM
      Serial.println(F(" -> Brake"));
#endif
      break;
    case 1 :
      lift.moveActuatorUp();
#ifdef DEBUGSIM
      Serial.println(F(" -> Upward"));
#endif
      break;
    case 2 :
      lift.moveActuatorDown();
#ifdef DEBUGSIM
      Serial.println(F(" -> Downward"));
#endif
      break;
    case 3 :
      // Timeout --> OffsetPosition is undetermined --> do nothing
#ifdef DEBUGSIM
      Serial.println(F(" -> Timeout"));
#endif
      break;
  }

  if (OnOffsetAction == 0) {
    return false;
  } else {
    return true;
  }
}

void scan_stop_callback(void) {
#ifdef DEBUGSIM
  Serial.println(F("Scanning by Central is stopped after timeout..."));
#endif
  // Show the message on the Oled
  ShowOnOledLarge("Trainer", "NOT", "Connected!", 500);
  ConnectedToTACX = false;
}

void adv_stop_callback(void) {
#ifdef DEBUGSIM
  Serial.println(F("Advertising by Peripheral is stopped after timeout..."));
#endif
  ConnectedToMobilePhone = false;
  // Show the message on the Oled
  if (!ConnectedToTACX) {
    ShowOnOledLarge("Phone", "NOT", "Paired!", 500);
  }
}

/**
   Callback invoked when scanner picks up advertising data
   @param report Structural advertising data
*/
void scan_callback(ble_gap_evt_adv_report_t* report) {
  // Since we configured the scanner with filterUuid, scan_callback is only invoked for
  // devices with the specific Tacx Uuid advertised! --> No use in looking any further!
  // The TACX trainer is advertising and We've got him!
  // It is critical to stop further scanning in the scan process before timeout!
  Bluefruit.Scanner.stop();
#ifdef DEBUGSIM
  Serial.println(F("Stopped Scanning...."));
#endif
  // Connect to the device with the Uuid of a TACX Trainer
  Bluefruit.Central.connect(report);
  // --------------------------------------------------------------------------------------
#ifdef DEBUGSIM
  Serial.println(F("TACX trainer found..."));
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
}

/**
   Callback invoked when a connection is established
   @param conn_handle
*/
void connect_callback(uint16_t conn_handle) {
  char Manufacturer[32 + 1];
  char ModelNum[32 + 1];
  char SerialNum[32 + 1];
  char FirmwareRevNum[32 + 1];

  /* First check for TACX Trainer services to be present

  */
#ifdef DEBUGSIM
  Serial.println(F("Now checking for FE-C critical services..."));
  Serial.print(F("Discovering FECPS Service ... "));
#endif
  // If FECPS is not found, disconnect and return
  if ( !fecps.discover(conn_handle) ) {
#ifdef DEBUGSIM
    Serial.println(F("Failed and disconnecting ..."));
#endif
    // disconnect since we couldn't find crucial service
    Bluefruit.disconnect(conn_handle);
    return;
  }
#ifdef DEBUGSIM
  Serial.println(F("Found it!"));
  Serial.print("Discovering FECRD Characteristic ... ");
#endif
  // If FECRD is not found, disconnect and return
  if ( !fecrd.discover() ) {
#ifdef DEBUGSIM
    Serial.println(F("Failed and disconnecting ..."));
#endif
    // disconnect since we couldn't find crucial service
    Bluefruit.disconnect(conn_handle);
    return;
  }
#ifdef DEBUGSIM
  Serial.print(F("Found it !\n"));
  Serial.print("Discovering FECWR Characteristic ... ");
#endif
  // If FECWR is not found, disconnect and return
  if ( !fecwr.discover() ) {
#ifdef DEBUGSIM
    Serial.println(F("Failed and disconnecting ..."));
#endif
    // disconnect since we couldn't find crucial service
    Bluefruit.disconnect(conn_handle);
    return;
  }
#ifdef DEBUGSIM
  Serial.print(F("Found it !\n"));
  Serial.print("Discovering Device Information Service ... ");
#endif
  /*
     Now check for the common BLE DIS Service
  */
  // If diss is not found then go on.... NOT FATAL !
  if ( diss.discover(conn_handle) ) {
#ifdef DEBUGSIM
    Serial.print(F("Found it !\n"));
#endif
    char buffer[32 + 1];
    //  1
    if ( disma.discover() ) {
      // read and print out Manufacturer
      memset(buffer, 0, sizeof(buffer));
      if ( disma.read(buffer, sizeof(buffer)) ) {
#ifdef DEBUGSIM
        Serial.print("Manufacturer: ");
        Serial.println(buffer);
#endif
        strcpy(Manufacturer, buffer);
      }
    }
    //  2
    if ( dismo.discover() ) {
      // read and print out Model Number
      memset(buffer, 0, sizeof(buffer));
      if ( dismo.read(buffer, sizeof(buffer)) ) {
#ifdef DEBUGSIM
        Serial.print("Model Number: ");
        Serial.println(buffer);
#endif
        strcpy(ModelNum, buffer);
      }
    }
    //  3
    if ( dissn.discover() ) {
      // read and print out Serial Number
      memset(buffer, 0, sizeof(buffer));
      if ( dissn.read(buffer, sizeof(buffer)) ) {
#ifdef DEBUGSIM
        Serial.print("Serial Number: ");
        Serial.println(buffer);
#endif
        strcpy(SerialNum, buffer);
      }
    }
    //  4
    if ( disfi.discover() ) {
      // read and print out Firmware Revision Number
      memset(buffer, 0, sizeof(buffer));
      if ( disfi.read(buffer, sizeof(buffer)) ) {
#ifdef DEBUGSIM
        Serial.print("Firmware Revision Number: ");
        Serial.println(buffer);
#endif
        strcpy(FirmwareRevNum, buffer);
      }
    }
    //  5
#ifdef DEBUG_DIS_EXT
    if ( disha.discover() ) {
      // read and print out Harware Revision Number
      memset(buffer, 0, sizeof(buffer));
      if ( disha.read(buffer, sizeof(buffer)) ) {
#ifdef DEBUGSIM
        Serial.print("Hardware Revision Number: ");
        Serial.println(buffer);
#endif
      }
    }
#endif
    //  6
#ifdef DEBUG_DIS_EXT
    if ( disso.discover() ) {
      // read and print out Software Revision Number
      memset(buffer, 0, sizeof(buffer));
      if ( disso.read(buffer, sizeof(buffer)) ) {
#ifdef DEBUGSIM
        Serial.print("Software Revision Number: ");
        Serial.println(buffer);
#endif
      }
    }
#endif
    // no more characteristics of Device Information Service
  }
  else {
#ifdef DEBUGSIM
    Serial.println(F("Found NONE !"));
#endif
  }
  strcat(Manufacturer, " ");
  strcat(Manufacturer, ModelNum);
  ShowOnOledLarge(Manufacturer, SerialNum, FirmwareRevNum, 2000);
  // ---------------------------------------------------------------------------------
  // Reaching here means we are ready to go, let's enable notification on reading data
  // ---------------------------------------------------------------------------------
  // ANT+ FE-C protocol reading is started now ! -------------------------------------
  // ---------------------------------------------------------------------------------
  if ( fecrd.enableNotify() ) { // To set ALL CLEAR: READY TO ROCK AND ROLL
    ConnectedToTACX = true;
#ifdef DEBUGSIM
    Serial.println(F("Ready to receive FE-C messages"));
#endif
  }
  else {
#ifdef DEBUGSIM
    Serial.println(F("Couldn't enable notify for FE-C messages"));
#endif
  }
} // Finally Done --------------------------------------------

void ShowOnOledLarge(char *Line1, char *Line2, char *Line3, uint16_t Pause) {
  // Clear and set Oled to display 3 line info -> centered
  int pos = 1;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  if (ConnectedToMobilePhone) { // show BLE icon
    display.drawBitmap(116, 0, bluetooth_icon16x16, 16, 16, 1);
  } // shift icon to the right as much as possible
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

// Create basic Oled screen for measurement data
void BuildBasicOledScreen(void) {
  display.clearDisplay(); // clean the oled screen
  display.setTextColor(SSD1306_WHITE);
  if (ConnectedToMobilePhone) { // show BLE icon
    display.drawBitmap(116, 0, bluetooth_icon16x16, 16, 16, 1);
  } // shift icon to the right as much as possible
  display.setTextSize(1);
  display.setCursor(14, 32);
  display.print(F("Watt"));
  display.setCursor(62, 32);
  display.print(F("Rpm"));
  display.setCursor(99, 32);
  display.print(F("Kph"));
  display.setCursor(102, 10);
  display.setTextSize(2);
  display.print(F("%"));
} // ---------------------------

// Funtion to show measurement data: Grade, Power, Cadence and Speed on Oled screen
void ShowValuesOnOled(void) {
  BuildBasicOledScreen();
  display.setTextColor(SSD1306_WHITE);
  char tmp[12];
  dtostrf(gradePercentValue, 5, 1, tmp); // show sign only if negative
  display.setCursor(10, 6);
  display.setTextSize(3);
  display.print(tmp);
  sprintf(tmp, "%03d %03d %02d", PowerValue, InstantaneousCadence, int(SpeedValue + 0.5));
  display.setCursor(4, 44);
  display.setTextSize(2);
  display.print(tmp);
  display.display();
}// -----------------------------------

void ShowSlopeTriangleOnOled(void) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  if (ConnectedToMobilePhone) { // show BLE icon
    display.drawBitmap(116, 0, bluetooth_icon16x16, 16, 16, 1);
  } // shift icon to the right as much as possible
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

/**
   Callback invoked when a connection is dropped
   @param conn_handle
   @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
*/
void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void) conn_handle;
  (void) reason;
  ConnectedToTACX = false;
  // Show the message on the Oled
  ShowOnOledLarge("Trainer", "Connection", "Lost!", 500);
#ifdef DEBUGSIM
  Serial.print(F("Disconnected, reason = 0x")); Serial.println(reason, HEX);
#endif
}

void SendRequestPage51(void) {
#ifdef DEBUGSIM
  Serial.printf("%d  Sending Request for data page 51 ", SendRequestPage51Event); Serial.println();
#endif
  // Page51Bytes are globally defined
  // uint16_t write(const void* data, uint16_t len);
  fecwr.write(Page51Bytes, sizeof(Page51Bytes));
}

/**
   Hooked callback that is triggered when any ANT+ message is sent from TACX Trainer
   @param chr   Pointer client characteristic that event occurred,
                in this code it should be FECRD
   @param data  Pointer to received data
   @param len   Length of received data
*/
void fecrd_notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len) {
  // The FE-C Read charateristic of ANT+ packets
  // In TACX context receive or send arrays of data ranging from 1--20 bytes so FE-C
  // will not exceed the 20 byte maximum...
  // Data pages are broadcast (by the trainer) at 4Hz message rate
  uint8_t buffer[20 + 1];
  memset(buffer, 0, sizeof(buffer)); // fill with zero
#ifdef DEBUGSIM
  Serial.printf("Dump of FE-C Data packets [len: %02d]  ", len);
#endif
  // Transfer first the contents of data to buffer (array of chars)
  for (int i = 0; i < len; i++) {
    if ( i <= sizeof(buffer)) {
      buffer[i] = *data++;
#ifdef DEBUGSIM
      Serial.printf("%02X ", buffer[i], HEX);
#endif
    }
  }
#ifdef DEBUGSIM
  Serial.println();
#endif
  // Standard FE-C Data Message Format
  // buffer[0]  ->  Sync
  // buffer[1]  ->  Msg Length
  // buffer[2]  ->  Msg ID
  // buffer[3]  ->  Channel Number
  // buffer[4]-buffer[12] Payload of 8 bytes --> buffer[4] is byte(0) of Payload bytes
  // buffer[13] ->  Checksum
  // --------------------------------
  uint8_t DataPageNumber = buffer[4]; // Get Data Page Number from ANT FE-C packet
  // process only the data pages we are interested in, ignore others !
  switch (DataPageNumber) {
    ///////////////////////////////////////////////////////////////
    //////////////////// Handle PAGE 71 ///////////////////////////
    ////////////// Requested PAGE 51 for grade info ///////////////
    //At a regular rate Page 51 is requested for, so process here//
    ///////////////////////////////////////////////////////////////
    case 0x47 :
      // buffer[4] -> Contains 71 (0x47) -> Common Page 71 -> Command status
      // buffer[5] -> Last Received Command ID
      // buffer[6] -> Sequence #
      // buffer[7] -> Command Status
      // buffer[8]-buffer[12] -> Response data 5 bytes
      // buffer[13] -> Checksum
      if ( buffer[5] == 0x33 ) { // check for Requested Page 51
        // We are interested in the Requested Page 51 (0x33) --> Track Resistance
        // in that case the packet contains:
        // buffer[5] -> Last Received Command ID --> Data Page Number 51 (0x33)
        // buffer[6] -> Reserved and set to 0xFF
        // buffer[7] -> Reserved and set to 0xFF
        // buffer[8] -> Reserved and set to 0xFF
        // buffer[9]-buffer[12] -> Response data 4 bytes
        // buffer[13] -> Checksum
        uint8_t lsb_GradeValue = buffer[9]; // Grade (Slope) LSB
        uint8_t msb_GradeValue = buffer[10];// Grade (Slope) MSB
        RawgradeValue = lsb_GradeValue + msb_GradeValue * 256;
        // buffer[11] -> Coefficient of Rolling Resistance
        // ----- Recalculate to relevant values for this project ------
        // Take into account the allowed Increase Percentage of the inclination
        // 100% has no effect, 50% means every increase or decrease is divided by 2
        // --> the increase or decrease of inclination is in 50% smaller steps...
        long ReferenceValue = (RGVMIN + 1000); // Reference is a flat road 0% inclination
        RawgradeValue = ReferenceValue + long((RawgradeValue - ReferenceValue) * GradeChangeFactor / 100);
        gradePercentValue = float((RawgradeValue - 20000)) / 100;
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
        if (ConnectedToMobilePhone) { // Update Phone display data changed!
          prph_bleuart_TX_Grade();
        }
#ifdef DEBUGSIM
        Serial.printf("--Page 51 received - RawgradeValue: %05d  ", RawgradeValue);
        Serial.printf("Grade percentage: %02.1f %%", gradePercentValue);
        Serial.println();
#endif
      }
      break;
    /////////////////////////////////////////////////
    /////////// Handle PAGE 25 Trainer/Bike Data ////
    /////////////////////////////////////////////////
    case 0x19 : {
        // buffer[4] -> Data Page Number -> 25 (0x19)
        uint8_t UpdateEventCount = buffer[5]; // Update Event Count -> rollover 256
        InstantaneousCadence = buffer[6];     // Instanteous Cadence 0-254 RPM
        // buffer[7] -> Accumulated Power LSB
        // buffer[8] -> Accumulated Power MSB
        uint8_t lsb_InstantaneousPower = buffer[9]; // Instantaneous Power LSB
        // POWER is stored in 1.5 byte !!!
        uint8_t msb_InstantaneousPower = (buffer[10] & 0x0F); // bits 0:3 --> MSNibble only!!!
        PowerValue = lsb_InstantaneousPower + msb_InstantaneousPower * 256;
        // buffer[10] --> bits 4:7 --> Trainer Status Bit Field
        // buffer[11] -> Flags bit field bits (0:3) and FE state Bit Field bits (4:7)
        // buffer[12] -> ???
        // buffer[13] -> Checksum
        if (ConnectedToMobilePhone) {
          prph_bleuart_TX_PWR_CAD();
        }
#ifdef DEBUGSIM
        //Serial.printf("Event count: %03d  ", UpdateEventCount);
        //Serial.printf(" - Cadence: %03d  ", InstantaneousCadence);
        //Serial.printf(" - Power in Watts: %04d  ", PowerValue);
        //Serial.println();
#endif
      }
      break;
    //////////////////////////////////////////////
    //////////// Handle PAGE 16 General FE Data //
    //////////////////////////////////////////////
    case 0x10 : {
        // buffer[4] -> Data Page Number 16 (0x10)
        // buffer[5] -> Equipment Type Bit Field
        uint8_t ReceivedValue = buffer[6];  // Elapsed Time --> in units of 0.25 seconds --> 256 rollover (every 64 seconds!)
        // process Elapsed Time ... since the start of a workout
        AccumulatedElapsedTime += ReceivedValue - PreviousETValue;
        if (PreviousETValue > ReceivedValue) { // rollover took place
          AccumulatedElapsedTime += 256;
        }
        PreviousETValue = ReceivedValue;
        // -----------------------------
        ReceivedValue = buffer[7]; // Distance Travelled -> in meters -> 256 rollover (every 256 m!)
        // process distance travelled ... since the start of a workout
        AccumulatedDistanceTravelled += ReceivedValue - PreviousDTValue;
        if (PreviousDTValue > ReceivedValue) { // rollover took place
          AccumulatedDistanceTravelled += 256;
        }
        PreviousDTValue = ReceivedValue;
        // ------------------------------
        uint8_t lsb_SpeedValue = buffer[8]; // Speed LSB -> in units of 0.001 m/s
        uint8_t msb_SpeedValue = buffer[9]; // Speed MSB -> in units of 0.001 m/s
        // Calculate Instantaneous Speed
        SpeedValue = (((lsb_SpeedValue + msb_SpeedValue * 256)) / 1000) * 3.6; // in units of 0,001 m/s to km/h (kp/h)
        // buffer[10] -> Heart rate measured by the trainer -> hand-contact
        // buffer[11] -> Capabilities Bit Field bits(0:3) and FE State Bit Field bits(4-7)
        // 0010  LSBits always same values
        // 00    Invalid -> no heart rate
        //   1   Distance Travelled is Enabled
        //    0  Virtual Speed Flag is always 0 --> values in bytes 4-5 (buffer[8-9]) are real speed (not virtual)
        // Test FE State Bit Field bits(4-7) & interpret -> 1 = "asleep(OFF)", 2 = "paused", 3 is "in use", 4 = "Finished(PAUSED)"
        // Used to limit sending null packets and to show trainer status if "Paused"
        if ((buffer[11] >> 4) == 3) { // make it the LS Nibble and check for IN USE = moving!
          IsTrainerMoving = true;
        } else {
          IsTrainerMoving = false;
        }
        // buffer[12] -> ????
        // buffer[13] -> Checksum
        if (ConnectedToMobilePhone) {
          prph_bleuart_TX_ADT_SPD_AET();
        }
#ifdef DEBUGSIM
        Serial.print("Buffer[11]: 0"); Serial.print(buffer[11], BIN); Serial.printf(" In Use: %2X \n", IsTrainerMoving);
        Serial.printf(" Elapsed time: %dh:%dm:%ds", ((AccumulatedElapsedTime / 4) / 3600), ((AccumulatedElapsedTime / 4) % 3600 / 60), ((AccumulatedElapsedTime / 4) % 60));
        Serial.printf(" - Accumulated Distance: %03.2f km ", AccumulatedDistanceTravelled / 1000);
        Serial.printf(" - Speed: %.1f km/h\n", SpeedValue);
#endif
      }
      break;
    case 0x80 : {
        // Manufacturer Identification Page
      }
    default : {
#ifdef DEBUGSIM
        //Serial.printf("Page: %2d ", DataPageNumber); Serial.println(F(" Received"));
#endif
        return;
      }
  } // end switch Data Page Number ------------------------------

  // Show the actual values of the trainer on the Oled
  if (OledDisplaySelection == 1) {
    ShowValuesOnOled();
  } else {
    ShowSlopeTriangleOnOled();
  }
  // Check and control motor up/down movement within settings!
  if (IsBasicMotorFunctions) {
    while (ControlUpDownMovement()) {
    }
  }
  // Send a request for Page 51 about every 4 seconds
#ifdef DEBUGSIM
  Serial.printf("Connected to TACX? %2X", ConnectedToTACX); Serial.printf("  ms: %6d", millis());
#endif
  if (ConnectedToTACX) {
    long tmp = millis() - SendRequestPage51Delay;
#ifdef DEBUGSIM
    Serial.print(F(" tmp: ")); Serial.println(tmp);
#endif
    if (tmp >= SendRequestPage51Event) { // Test timing for next Page 51 request
      SendRequestPage51Event = millis();
      SendRequestPage51();
    }
  }
  //////////////////////// DONE! /////////////////////////
}
