# Rotator7

Antenna rotator controller derived from the SARCnet MK1 Mini Rotator project by Julie VK3FOWL and Joe VK3YSP (http://www.sarcnet.org). Modified to support azimuth/elevation control over RS-485 using the rotctld protocol from Hamlib, with compatibility for SDRangel Star Tracker, Satellite Tracker, and Radio Astronomy plugins.

MCU: ATmega328P (Arduino Nano form factor).  
Sensor: LSM303AGR (ST Microelectronics) — 3-axis magnetometer (LIS2MDL) + 3-axis accelerometer (LIS2DH12) via I2C. Replaces the original LSM303DLHC, which is discontinued. The firmware retains backward-compatible support for LSM303DLHC and LSM303D via compile-time selection (`SensorType` constant in `main.cpp`).

RS-485 is implemented in half-duplex mode using a MAX485 transceiver. Direction control (DE/RE) is managed in software via a dedicated GPIO pin, with automatic switching between transmit and receive mode handled by a custom `HalfDuplexRs485Serial` class wrapping SoftwareSerial.

---

## Changes from upstream

This project remains structurally close to the original SARCnet Mk1 implementation. The main changes were introduced to keep the controller practical to build and usable with current hardware and software.

- RS-485 transport replaced the original local USB/TTL usage for operational control at the rotator side.
  Reason: the controller can now be used at a practical distance from the host system without requiring the PC or USB adapter to remain physically close to the rotator.
- A half-duplex MAX485 interface was added, with DE/RE direction control handled in firmware.
  Reason: this provides a simple and inexpensive wired link suitable for remote operation.
- The original LSM303DLHC-based sensor path was extended with LSM303AGR support, using the currently available Adafruit board.
  Reason: the sensors referenced by the original project are difficult to source or discontinued.
- Sensor initialization, register handling, and calibration behavior were adapted for the LSM303AGR.
  Reason: the replacement sensor is not register-compatible with the original hardware in all respects, so direct substitution without firmware changes would not be reliable.
- Hamlib `rotctld` interoperability was validated for use with SDRangel Star Tracker, Satellite Tracker, and Radio Astronomy plugins.
  Reason: the project is intended for practical integration with current tracking workflows rather than only direct serial use.

The original control model, general project structure, and operating logic were preserved wherever possible.

---

## RS-485 Wiring

### Block diagram

```
PC (Linux)
    |
  CP2102 (USB to TTL)
    |
  MAX485 module (PC side)
    |
  2-wire cable (A / B)
    |
  MAX485 module (MCU side)
    |
  ATmega328P
```

### PC side — CP2102 to MAX485

| CP2102 | MAX485 |
|--------|--------|
| TX     | DI     |
| RX     | RO     |
| 5V     | VCC    |
| GND    | GND    |
| —      | DE+RE tied to VCC (always enabled) |

### MCU side — MAX485 to ATmega328P

| MAX485 | Pin  | Notes               |
|--------|------|---------------------|
| RO     | D2   | SoftwareSerial RX   |
| DI     | D3   | SoftwareSerial TX   |
| DE+RE  | D4   | Direction control   |
| VCC    | 5V   |                     |
| GND    | GND  |                     |

### RS-485 cable (2-wire)

| MAX485 PC side | MAX485 MCU side |
|----------------|-----------------|
| A              | A               |
| B              | B               |

Twisted pair cable is recommended. For runs longer than 100 m, add a 120 Ω termination resistor between A and B at both ends.

---

## Serial port parameters (PC side)

| Parameter    | Value                                           |
|--------------|-------------------------------------------------|
| Baud rate    | 9600                                            |
| Data bits    | 8                                               |
| Parity       | N                                               |
| Stop bits    | 1                                               |
| Flow control | None                                            |
| Linux port   | `/dev/ttyUSB0` (verify with `ls /dev/ttyUSB*`) |

---

## Usage with Hamlib (rotctld)

```bash
rotctld -m 202 -r /dev/ttyUSB0 -s 9600
```

Connection test:
```bash
rotctl -m 2 -r /dev/ttyUSB0 -s 9600
```

---

## Usage with SDRangel

In the Rotator Controller panel:
- Port: `/dev/ttyUSB0`
- Speed: `9600`
- Format: `8N1`
- Flow control: `None`

---

## ATmega328P pin assignment

| Pin  | Function                        |
|------|---------------------------------|
| D0   | USB Serial RX (programming)     |
| D1   | USB Serial TX (programming)     |
| D2   | RS-485 RX (SoftwareSerial)      |
| D3   | RS-485 TX (SoftwareSerial)      |
| D4   | RS-485 DE/RE                    |
| D5   | AZ motor — FWD (PWM)            |
| D6   | AZ motor — REV (PWM)            |
| D7   | AZ motor — BRK                  |
| D8   | EL motor — BRK                  |
| D9   | EL motor — FWD (PWM)            |
| D10  | EL motor — REV (PWM)            |
| D11  | Piezo buzzer                    |
| D12  | Buzzer GND                      |
| A4   | I2C SDA (LSM303DLHC)            |
| A5   | I2C SCL (LSM303DLHC)            |
