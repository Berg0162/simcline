# Changelog nRF52 FTMS_Client

## v0.3.6

### Added or Changed

- Inserted bool clientIsReadyToOperate instead of wait loop interval in setup()
- Inserted more selective setting of Connection Parameters at start of scanning
- Inserted requestConnectionParameter after client connection and synch with poll_delay
- Inserted #define REQUEST_NEW_CONNECTION_PARAMETERS and allows for selective setting of ConnectionInterval

### Removed
- Some older versions

# Changelog ESP32 FTMS_Client

## v0.1.0
### Added or Changed
- Added first version

## v0.1.1
### Added or Changed
- Cycling Speed Cadence Service changed to NOT Mandatory

## v0.1.2
### Added or Changed
- Inserted checks on the input value of sensor location and location description array sizes (CPS, CSC and HBM)

### Removed
- Some older versions
