# Changelog nRF52 FTMS Simcline

## v0.2.0

### Added or Changed
- Extra waiting directly after client connection, delay(poll_delay) moved after debug prints....
- Inserted full HBM support
- Corrected versions of client_enable and client_connect
- Corrected version of Indoor Bike Data
- updated with code of Client version 3.0
- Inserted setTxPower(4) after Bluefruit.begin() for scanning that was missing....
- Inserted Bluefruit.configCentralConn for central and peripheral
- Inserted Bluefruit.getName for determining the board name!
- changed filterRSSI to -80
- After Client or Server disconnect: scanning and advertising again!!
- Machine Status and Trainingstatus added missing if(...Chr.notifyEnabled)
- Inserted wait for connection in client_connect_callback 

## v0.2.1

### Added or Changed
- Corrected unsecure use of sizeof() with string arrays! Feature: FTM, CP and CSC !!!

## v0.2.2

### Added or Changed
- Phone upload with Server_Sends_NUS_TXD_Persistent_Settings realized after Central CCCD Notify enabled on NUS_TXD
- Trainer.IsConnected = true only at the end of client_connect_callback to avoid cluttering of messages
- Inserted more selective setting of Connection Parameters at start of scanning and at start of advertising
- Inserted requestConnectionParameter after client connection and synch with poll_delay
- Inserted #define REQUEST_NEW_CONNECTION_PARAMETERS and allows for selective setting of ConnectionInterval
  
# Changelog ESP32 FTMS Simcline

## v0.1.1

### Added or Changed
- Modified xControlUpDownMovement task; Runs now on "Core 0"; User needs to pin "Core 1" in Tools Menu to Events & Arduino tasks
- Modified vTaskDelay() to be more in synch with 10Hz sample rate of VL6180X
- Changed Stack Depth values from 2048 to 4096 for Server Control Point Indicate (modified) and Write w Response

## v0.1.2
- Inserted check (boolean) on Control Point Write-Response out of synch...

## v0.1.3
- Changed CSC service to NOT Mandatory

## v0.1.4
- Inserted checks on the input values of sensor location and location description array sizes (CP, CSC and HBM)

## v0.1.5
- Corrected invalid (enable/disable) setting of HR measurement Characteristic in client_Set_All_NotificationIndication()
- Changed device identification naming to a simpler scheme: SIM32 instead of "SIM DevName"
- Server Characteristic values (read only) are now updated when a new client connection is established
- NimBLE registerForNotify() has been deprecated and is replaced with subscribe() / unsubscribe()

### Removed
-

# Changelog EXPERIMENTAL_esp32s3_T-Display

## v0.1.2
- Conforms ESP32 FTMS Simcline version 0.1.2

### Added or Changed
- Added experimental code version of ESP32 FTMS Simcline v0.1.2 with a <b>LilyGo ESP32S3 T-Display</b> development board
- Added experimental code version of ESP32S3 T-Display Simcline Diagnostics Test
