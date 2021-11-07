/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/* Program to test Android Smartphone <-> Feather connection over BLE
 * Use the SIMCLINE Companion app to make a connection with the Feather
 * When a connection is established fake/test road grade and cycling data 
 * are sent from the Feather to the SIMCLINE App; if all goes well these are
 * rendered on the screen Monitor Operation. User can select Manual Control
 * and sent test/fake values to Move UP or Move Down, manual operation....
 */

#include <bluefruit.h>

// Uart over BLE service
BLEUart bleuart;
char RXpacketBuffer[20+1]={0};

void setup(void)
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println(F("Bluefruit-nRF52 Test of Android Phone App <-> Feather connection"));
  Serial.println(F("----------------------------------------------------------------"));
  
  // TACX trainer connection is of type Central for Smartphone connection add 1 Peripheral !!!!
  // Initialize Bluefruit with maximum connections as Peripheral = 1, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(1, 1);
  
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName("Bluefruit-nRF52");

  /* Set the LED interval for blinky pattern on BLUE LED */
  Bluefruit.setConnLedInterval(250);

  // Callbacks for Peripheral
  Bluefruit.Periph.setConnectCallback(prph_connect_callback);
  Bluefruit.Periph.setDisconnectCallback(prph_disconnect_callback);

  // Configure and start the BLE Uart service
  bleuart.begin();
  bleuart.setRxCallback(prph_bleuart_rx_callback);

  // Set up and start advertising
  startAdv();

  Serial.println("Advertising for Phone connection..."); 
  Serial.println("Activate the Simcline Companion App on your Phone!");
  Serial.println("Connect to Feather: Bluefruit-nRF52");
  Serial.println("Select Monitor Operation: grade and cycling data will change!");
  Serial.println("Select Manual Control: press UP/DOWN!");
  Serial.println();  
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  
  // Include the BLE UART (AKA 'NUS') 128-bit UUID
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  // A.K.A. secondary response packet 
  Bluefruit.ScanResponse.addName();

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

/*------------------------------------------------------------------*/
/* Peripheral
 *------------------------------------------------------------------*/
void prph_connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char peer_name[32] = { 0 };
  connection->getPeerName(peer_name, sizeof(peer_name));

  Serial.print("[Prph] Connected to ");
  Serial.println(peer_name);
  
  delay(2000);
  Serial.println("[Prph] New Default Settings sent");
  //test settings to start with these values....
  bleuart.println("!S13;7;77;1;");
  delay(1000);
  Serial.println("[Prph] Some Cycling data sent");
  //test section for sending data values to different pages....
  // START of sending varying Grade data, intermixed with P S C data, to show 
  // graphic inclination grade change appropriately
   
  //Send some test data (POWER, CADENCE) for de Monitor page
  delay(2000);
  bleuart.println("!C255;92;");

  // some additional Distance Speed Time data
  bleuart.println("!A2.4;24.5;00:08;");
  delay(2000); 

  bleuart.println("!G1.5;");
  delay(2000);
  bleuart.println("!G20;"); 
  delay(2000); 
  bleuart.println("!G10.5;");
  delay(2000);
  bleuart.println("!G8.8;"); 
  delay(2000);

  // some additional P C data
  bleuart.println("!C240;89;");
  delay(2000); 

  // some additional Distance Speed Time data
  bleuart.println("!A5.4;34.2;00:45;");
  delay(2000); 
  
  bleuart.println("!G6.5;");
  delay(2000);
  bleuart.println("!G2.0;"); 
  delay(2000); 
  bleuart.println("!G15.8;");
  delay(2000);
  bleuart.println("!G19.5;"); 
  delay(2000); 
  bleuart.println("!G-1.5;");
  delay(2000);
  bleuart.println("!G-20;"); 
  delay(2000); 
  bleuart.println("!G-10.5;");
  delay(2000); 

  // some additional P C data
  bleuart.println("!C180;93;");
  delay(2000);

  // some additional Distance Speed Time data
  bleuart.println("!A10.7;31.6;00:30;");
  delay(2000); 

  bleuart.println("!G8.8;"); 
  delay(2000); 
  bleuart.println("!G6.5;");
  delay(2000);
  bleuart.println("!G-2.0;"); 
  delay(2000); 
  bleuart.println("!G-15.8;");
  delay(2000);
  bleuart.println("!G-19.5;"); 
  delay(2000); 

  // some additional P C data
  bleuart.println("!C200;96;");
  delay(2000); 

  // some additional Distance Speed Time data
  bleuart.println("!A15.3;28.0;01:05;");
  delay(2000); 

  // END of sending varying Grade data to show graphic inclination grade change appropriately
}

void prph_disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.println("[Prph] Disconnected");
}

void prph_bleuart_rx_callback(uint16_t conn_handle)
{
  (void) conn_handle;
  // Read data from Mobile Phone
  // char str[20+1] = { 0 };
  bleuart.read(RXpacketBuffer, 20);

  Serial.print("[Prph] RX: ");
  Serial.println(RXpacketBuffer); 
  // Echo RX data to Mobile Phone
  bleuart.print(RXpacketBuffer); 
  
  // parse Received packet buffer
    if (RXpacketBuffer[1] == 'S') {
    // get MaxUpwardInclination
    // get MinDownwardInclination
    // get PercIncreaseInclination
    // etcetera
    }

  // Buttons
  if (RXpacketBuffer[1] == 'U') {
    Serial.println ("Button UPward pressed!");
    }
 if (RXpacketBuffer[1] == 'P') {
    Serial.println ("Button PAUSE pressed!");
    }
 if (RXpacketBuffer[1] == 'D') {
    Serial.println ("Button DOWNward pressed!");
    }
}

/**************************************************************************/
/*!
    @brief  In LOOP Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
// do nothing...... 
}
