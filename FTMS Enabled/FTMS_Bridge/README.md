# Changelog nRF52 FTMS_Bridge --> ESP-code is deprecated!

## v0.3.1
### Added or Changed
- Corrected unsecure use of sizeof() with string arrays! Feature: FTM, CP and CSC !!!

## v0.3.2
### Added or Changed
- Trainer.IsConnected = true only at the end of client_connect_callback to avoid cluttering of messages
- Inserted more selective setting of Connection Parameters at start of scanning and at start of advertising
- Inserted requestConnectionParameter after client connection and synch with poll_delay
- Inserted #define REQUEST_NEW_CONNECTION_PARAMETERS and allows for selective setting of ConnectionInterval

# Changelog ESP32 FTMS_Bridge

## v0.1.1
### Added or Changed
Changed Stack Depth values from 2048 to 4096 for Server Control Point Indicate (modified) and Write w Response

## v0.1.2
### Added or Changed
Inserted check (boolean) on Control Point Write-Response out of synch...

## v0.1.3
### Added or Changed
Cycling Speed Cadence Service changed to NOT Mandatory

## v0.1.4
### Added or Changed
Inserted checks on the input values of sensor location and location description array sizes (CP, CSC and HBM)

## v0.1.5
### Added or Changed
Corrected invalid (enable/disable) setting of HR measurement Characteristic in client_Set_All_NotificationIndication()<br>
Changed device identification naming to a simpler scheme: SIM32 or SIM52 instead of "Sim DevName"<br>
Server Characteristic values (read only) are now updated when a new client connection is established<br>
NimBLE registerForNotify() has been deprecated and is replaced with subscribe() / unsubscribe()<br>

### Removed
- Some older versions
