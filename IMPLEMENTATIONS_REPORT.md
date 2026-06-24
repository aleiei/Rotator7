# Rotator7 Implementations Report

## Scope
This document summarizes the effective implementations added to the current Rotator7 PlatformIO project compared with the original baseline.

## Implementations Completed

### LSM303AGR support
- Added full support for the replacement Adafruit LSM303AGR sensor.
- Added AGR accelerometer and magnetometer register definitions.
- Added AGR initialization and readout support for the relevant I2C address variants.
- Kept the original Rotator7 axis transformation model compatible with the new sensor.

### Sensor and I2C robustness
- Added 16-bit sample assembly helpers to avoid sign and byte-order issues.
- Added alternate address support for compatible boards.
- Added I2C timeout protection where supported by Wire.
- Removed per-sample sensor reinitialization that was interfering with serial responsiveness.
- Reduced boot-time filter priming.
- Added calibration fallback handling for invalid or near-zero scale values.
- Added fallback use of normalized raw acceleration when accelerometer calibration quality is poor.

### EEPROM and calibration safety
- Added validation of calibration data loaded from EEPROM.
- Added safe default calibration recovery when EEPROM data is invalid or uninitialized.
- Added a factory-clear path for calibration data.
- Added the `w` command to clear calibration EEPROM.

### Serial command handling
- Reworked command parsing to restore reliable command handling.
- Changed parser entry points to use `const String&` where needed.
- Added strict magnetic declination parsing.
- Added support for positive and negative declination values.
- Added support for both decimal dot and decimal comma input.
- Added clearer feedback for accepted and rejected commands.

### Motion control and runtime behavior
- Changed reset behavior so the controller starts in a safe paused state.
- Added protection against unintended motor movement before a valid target is received.
- Corrected azimuth and elevation error sign handling.
- Restricted motor drive activity to valid active-control modes only.
- Ensured diagnostic and paused states keep the motors stopped.
- Preserved measured position after reset until a new target is explicitly received.

### Easycomm, Hamlib and SDRangel compatibility
- Added reliable Easycomm command prioritization.
- Prevented `AZ...` commands from being misinterpreted as single-letter commands.
- Added support for multiple practical Easycomm input formats.
- Added explicit support for the combined `AZ EL` query used by Hamlib.
- Added fresh sensor acquisition before Easycomm position replies.
- Clamped reported elevation to a valid range before external reporting.

### RS485 support
- Added RS485 support on Arduino Nano using SoftwareSerial.
- Added explicit MAX485 pin mapping.
- Added half-duplex DE/RE direction control.
- Fixed the previous condition where the RS485 transceiver could remain stuck in transmit mode.
- Added a compile-time selector to choose RS485 transport.

### Calibration and diagnostics improvements
- Updated calibration mode to acquire fresh samples before each calibration step.
- Updated buzzer handling to use `tone()` and `noTone()`.
- Extended the help text with the calibration EEPROM clear command.
- Added an I2C scan helper for diagnostics.

### Project layout and build cleanup
- Renamed the main firmware source to `main.cpp` for better VS Code / PlatformIO alignment.
- Verified that the current PlatformIO configuration builds correctly with the renamed main source file.
- Cleaned the reported warnings by making vector math methods const-correct and removing an unused motor-control variable.

## Test Results

### PlatformIO build and upload
- The firmware builds successfully with PlatformIO for the Arduino Nano target.
- The final firmware image was uploaded successfully to the board.

### Direct serial and Easycomm validation
- USB serial command handling was restored and validated.
- Magnetic declination commands were validated with positive, negative, dot-decimal and comma-decimal values.
- Declination persistence after save and reset was validated.
- Easycomm `AZ`, `EL` and combined `AZ EL` queries were validated.

### SDRangel and Star Tracker plugin
- SDRangel integration was validated through Hamlib `rotctld`, not by direct connection to the firmware.
- The Star Tracker plugin works when configured to use the `rotctld` protocol over TCP.
- Valid working configuration:
  - Protocol: `rotctld`
  - Connection: `TCP`
  - Host: `127.0.0.1`
  - Port: `4533`

### rotctld terminal command
Use the following command to expose the rotator to SDRangel through Hamlib:

```bash
rotctld -m 202 -r /dev/ttyUSB1 -s 9600 -t 4533
```

Notes:
- `-m 202` selects EasycommII.
- `/dev/ttyUSB1` is the validated RS485 adapter path used during testing.
- `4533` is the TCP port used by SDRangel Star Tracker in the validated setup.

### RS485 validation
- RS485 communication was validated on `/dev/ttyUSB1`.
- A live Easycomm query returned a valid response.

Verified example response:

```text
AZ284.8 EL0.0
```

## Current Configuration Note

At the time of this report, the firmware is configured to use RS485 transport by default through `USE_RS485_SERIAL = 1`.
If direct USB serial is needed again, the transport selector can be changed in the main firmware configuration.
