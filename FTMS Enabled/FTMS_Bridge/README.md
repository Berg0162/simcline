# Changelog nRF52 FTMS_Bridge

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

## v0.1.0

### Added or Changed
- Added first version

### Removed
- Some older versions
