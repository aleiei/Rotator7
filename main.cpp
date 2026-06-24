//Rotator7.ino - Mini Satellite-Antenna Rotator.
//Copyright (c) 2015-2025 Julie VK3FOWL and Joe VK3YSP
//Released under the GNU General Public License.
//For more information please visit http://www.sarcnet.org
//Submitted for publication in Amateur Radio magazine: December 2015
//First published in Amateur Radio magazine: May 2016
//Upgraded Mk2 version published in Amateur Radio magazine: October 2017
//Release history:
//Release 1: Original release
//Release 2: Added support for hamlib 3.0.1. Added debug mode.
//Release 3: Improved calibration and operation
//  Added a low pass filter to the sensor data to improve calibration.
//  Reinitialised the I2C bus and sensor prior to each read to avoid I2C lockups caused by power glitches
//  Added support for RS-422 operation
//Release 4: Improved calibration and operation
//  Fixed a bug reading the EEPROM calibration data on some versions
//  Made the SerialPort configurable to support both the Mk1 (USB) and Mk2 (RS422) rotators
//  Added a speaker output to help with the calibration process
//  Removed the overshoot inherent in the anti-windup algorithm
//  Initialised the sensor filters at start up
//  Added a pause command, as requested
//  Added a help menu, as requested
//  Clarified sensor axis definitions
//  Please note that Gpredict position feedback does not work with the USB version on Windows due to a handshaking bug
//  You can fix this by using Linux or the Serial1 port with a TTL to USB converter.
//Release 5: Changes to support the half-price Mk1b version
//  Added support for either the original LMD18200T or the cheaper L298N DC Motor H-Bridge Driver Boards
//  Added compiler option to set the driver board type and the pins used
//  Note: Only PWM pins 5, 6, 9 or 10 can be used for PWM motor drive output
//  Added support for either the original LSM303D or the cheaper LSM303DLHC 3D Accelerometer/Magnetometer
//  Replaced the passive piezo speaker with an active piezo buzzer to help with the calibration process (since there were not enough PWM outputs)
//Release 6: Changes to support the AC Motor triac controller
//  Included the non-blocking timer class to replace the delay() function
//Release 7: Changes to support the Arduino Nano. 18 February 2025.

//Includes
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <ctype.h>
#include "timer.h"
#include "lsm.h"
#include "mot.h"

//Constants
//User configuration section:
//Please uncomment only one of each of the following MotorTypes, SensorTypes and SerialPort types:
//const int MotorType = PWMDIR;     //Please uncomment this line for the LMD18200T DC motor driver.
const int MotorType = FWDREV;     //Please uncomment this line for the L298N DC motor driver.
//const int MotorType = ACMOTR;       //Please uncomment this line for the triac AC motor driver.
const int SensorType = LSM303AGR;   //Please uncomment this line to use the LSM303AGR sensor.
//const int SensorType = LSM303D;   //Please uncomment this line to use the LSM303D sensor.
//const int SensorType = LSM303DLHC;  //Please uncomment this line to use the LSM303DLHC sensor.
#define USE_RS485_SERIAL 1
//#define USE_RS485_SERIAL 0
#define WINDUP_LIMIT 450            //Sets the total number of degrees azimuth rotation in any direction before resetting to zero

//RS-485 SoftwareSerial pins
const int rs485RxPin = 2;   //Connect to RO of MAX485
const int rs485TxPin = 3;   //Connect to DI of MAX485
const int rs485DeRePin = 4; //Connect to DE+RE of MAX485

#if USE_RS485_SERIAL
SoftwareSerial rs485RawSerial(rs485RxPin, rs485TxPin);

class HalfDuplexRs485Serial : public Stream {
public:
  HalfDuplexRs485Serial(SoftwareSerial &port, int directionPin)
    : port_(port), directionPin_(directionPin), transmitting_(false) {}

  void begin(unsigned long baud) {
    pinMode(directionPin_, OUTPUT);
    setReceiveMode();
    port_.begin(baud);
    port_.listen();
  }

  int available() override {
    if (transmitting_) setReceiveMode();
    return port_.available();
  }

  int read() override {
    return port_.read();
  }

  int peek() override {
    return port_.peek();
  }

  void flush() override {
    port_.flush();
    if (transmitting_) setReceiveMode();
  }

  size_t write(uint8_t value) override {
    setTransmitMode();
    return port_.write(value);
  }

  size_t write(const uint8_t *buffer, size_t size) override {
    setTransmitMode();
    return port_.write(buffer, size);
  }

  using Print::write;

private:
  void setTransmitMode() {
    if (!transmitting_) {
      digitalWrite(directionPin_, HIGH);
      transmitting_ = true;
    }
  }

  void setReceiveMode() {
    port_.flush();
    digitalWrite(directionPin_, LOW);
    transmitting_ = false;
    port_.listen();
  }

  SoftwareSerial &port_;
  int directionPin_;
  bool transmitting_;
};

HalfDuplexRs485Serial rs485Serial(rs485RawSerial, rs485DeRePin);
#define SerialPort rs485Serial      //Use RS-485 via SoftwareSerial.
#else
#define SerialPort Serial           //Use USB port.
//#define SerialPort Serial1        //Uncomment to use TTL port.
#endif

//Motor pins - Don't change
const int azFwdPin = 5;
const int azRevPin = 6;
const int azBrkPin = 7;
const int elBrkPin = 8;
const int elFwdPin = 9;
const int elRevPin = 10;
//Speaker pins
const int spkPin = 11;    //Attach a piezo buzzer to this pin. It beeps when new calibration data arrives.
const int gndPin = 12;    //Makes a convenient ground pin adjacent to the speaker pin

//Motor drive gains. These set the amount of motor drive close to the set point
const int azGain = 25;   //Azimuth motor gain
const int elGain = 25;   //Elevation motor gain
//Filter constants
const float azAlpha = 0.5; //Alpha value for AZ motor filter: Decrease to slow response time and reduce motor dither.
const float elAlpha = 0.5; //Alpha value for EL motor filter: Decrease to slow response time and reduce motor dither.
const float lsmAlpha = 0.02; //Alpha value for sensor filter: Decrease to slow response time and ease calibration process.

//Modes
enum Modes {tracking, monitoring, demonstrating, calibrating, debugging, pausing, manualdrive};    //Rotator controller modes

//Global variables
float az;               //Antenna azimuth
float el;               //Antenna elevation
String line;            //Command line
float azSet;            //Antenna azimuth set point
float elSet;            //Antenna elevation set point
float azLast;           //Last antenna azimuth reading
float elLast;           //Last antenna element reading
float azWindup;         //Antenna windup angle from startup azimuth position
float azOffset;         //Antenna azimuth offset for whole revolutions
bool windup;            //Antenna windup condition
float azSpeed;          //Antenna azimuth motor speed
float elSpeed;          //Antenna elevation motor speed
float azError;          //Antenna azimuth error
float elError;          //Antenna elevation error
float azInc;            //AZ increment for demo mode
float elInc;            //EL increment for demo mode
float azManualErr;      //Manual AZ drive command (-180..180)
float elManualErr;      //Manual EL drive command (-180..180)
bool trackingArmed;     //Track only after a valid position command
Modes mode;             //Rotator mode

//Objects

//Motor driver object: Mot xxMot(Driver-Type, Filter-Alpha, Gain, Fwd-Pin, Rev/Dir-Pin)
Mot azMot(MotorType, azAlpha, azGain, azFwdPin, azRevPin); //AZ motor driver object
Mot elMot(MotorType, elAlpha, elGain, elFwdPin, elRevPin); //EL motor driver object

//LSM sensor object: Lsm lsm(Sensor-Type, Filter-Alpha)
Lsm lsm(SensorType,lsmAlpha);

//Non-blocking Timer object
Timer t1(100);

//Forward declarations
void restore();
void printCal();
void processEasycommCommands(const String &line);
void processEasycommCommands(const String &line, bool fromUserCommand);
void setDefaultCalibration();
void clearCalibration();
bool isCalibrationValid();
void scanI2C();

bool isFiniteFloat(float value) {
  return !isnan(value) && !isinf(value);
}

bool isFiniteVec(const Vec &v) {
  return isFiniteFloat(v.i) && isFiniteFloat(v.j) && isFiniteFloat(v.k);
}

bool parseSignedFloat(String value, float *out) {
  // Parse signed decimal values in a strict way to avoid silent toFloat() failures.
  value.trim();
  value.replace(',', '.');
  if (value.length() == 0) return false;

  size_t i = 0;
  bool hasDigit = false;
  bool hasDot = false;

  if (value.charAt(i) == '+' || value.charAt(i) == '-') {
    i++;
    if (i >= (size_t)value.length()) return false;
  }

  for (; i < (size_t)value.length(); i++) {
    char c = value.charAt(i);
    if (c >= '0' && c <= '9') {
      hasDigit = true;
      continue;
    }
    if (c == '.' && !hasDot) {
      hasDot = true;
      continue;
    }
    return false;
  }

  if (!hasDigit) return false;
  *out = value.toFloat();
  return isFiniteFloat(*out);
}

//Functions
void reset(bool getCal) {
  //Reset the rotator, initialize its variables and optionally get the stored calibration
  azSet = NAN;
  elSet = NAN;
  line = "";
  azLast = 0.0;
  elLast = 0.0;
  azWindup = 0.0;
  azOffset = 0.0;
  azSpeed = 0.0;
  elSpeed = 0.0;
  mode = pausing;
  windup = false;
  if (getCal) restore();
  azError = 0.0;
  elError = 0.0;
  azInc = 0.05;
  elInc = 0.05;
  azManualErr = 0.0;
  elManualErr = 0.0;
  trackingArmed = false;
  t1.reset(100);
  printCal();
  lsm.calStart(); //Reset the axis calibration objects
}

float diffAngle(float a, float b) {
  //Calculate the acute angle between two angles in -180..180 degree format
  float diff = a - b;
  if (diff < -180) diff += 360;
  if (diff > 180) diff -= 360;
  return diff;
}

void save() {
  //Save the calibration data to EEPROM
  EEPROM.put(0, lsm.cal);
}

void setDefaultCalibration() {
  //Set a safe baseline calibration.
  lsm.cal.md = 0.0;
  lsm.cal.me = Vec(0.0, 0.0, 0.0);
  lsm.cal.ge = Vec(0.0, 0.0, 0.0);
  lsm.cal.ms = Vec(1.0, 1.0, 1.0);
  lsm.cal.gs = Vec(1.0, 1.0, 1.0);
}

bool isCalibrationValid() {
  //Reject uninitialized/corrupt EEPROM calibration values.
  if (!isFiniteFloat(lsm.cal.md)) return false;
  if (!isFiniteVec(lsm.cal.me) || !isFiniteVec(lsm.cal.ge)) return false;
  if (!isFiniteVec(lsm.cal.ms) || !isFiniteVec(lsm.cal.gs)) return false;
  if (abs(lsm.cal.ms.i) < 0.001f || abs(lsm.cal.ms.j) < 0.001f || abs(lsm.cal.ms.k) < 0.001f) return false;
  if (abs(lsm.cal.gs.i) < 0.001f || abs(lsm.cal.gs.j) < 0.001f || abs(lsm.cal.gs.k) < 0.001f) return false;
  return true;
}

void clearCalibration() {
  //Factory-clear calibration and persist defaults.
  setDefaultCalibration();
  save();
  lsm.calStart();
}

void scanI2C() {
  //Scan I2C bus and print responding addresses.
  byte found = 0;
  SerialPort.println("I2C scan start");
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte err = Wire.endTransmission();
    if (err == 0) {
      SerialPort.print("I2C device at 0x");
      if (addr < 16) SerialPort.print("0");
      SerialPort.println(addr, HEX);
      found++;
    }
  }
  SerialPort.print("I2C devices found: ");
  SerialPort.println(found);
}

void restore() {
  //Restore the calibration data from EEPROM
  EEPROM.get(0, lsm.cal);
  if (!isCalibrationValid()) {
    setDefaultCalibration();
    save();
  }
}

void printDebug(void) {
  //Print raw sensor data
  SerialPort.print(lsm.mx); SerialPort.print(",");
  SerialPort.print(lsm.my); SerialPort.print(",");
  SerialPort.print(lsm.mz); SerialPort.print(",");
  SerialPort.print(lsm.gx); SerialPort.print(",");
  SerialPort.print(lsm.gy); SerialPort.print(",");
  SerialPort.println(lsm.gz);
}

void printCal(void) {
  //Print the calibration data
  SerialPort.print(lsm.cal.md, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.me.i, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.me.j, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.me.k, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.ge.i, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.ge.j, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.ge.k, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.ms.i, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.ms.j, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.ms.k, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.gs.i, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.gs.j, 1); SerialPort.print(",");
  SerialPort.println(lsm.cal.gs.k, 1);
}

void printMon(float az, float el, float azSet, float elSet, float azWindup, float azError, float elError) {
  //Print the monitor data
  SerialPort.print(az, 0); SerialPort.print(",");
  SerialPort.print(el, 0); SerialPort.print(",");
  SerialPort.print(azSet, 0); SerialPort.print(",");
  SerialPort.print(elSet, 0); SerialPort.print(",");
  SerialPort.print(azWindup, 0); SerialPort.print(",");
  SerialPort.print(windup); SerialPort.print(",");
  SerialPort.print(azError, 0); SerialPort.print(",");
  SerialPort.println(elError, 0);
}

void printAzEl() {
  //Print the rotator feedback data in Easycomm II format
  float elOut = el;
  if (elOut < 0.0) elOut = 0.0;
  if (elOut > 180.0) elOut = 180.0;
  SerialPort.print("AZ");
  SerialPort.print((az < 0) ? (az + 360) : az, 1);
  SerialPort.print(" EL");
  SerialPort.print(elOut, 1);
  SerialPort.print("\n");
}

void printAz() {
  //Print the rotator feedback data in Easycomm II format
  SerialPort.print("AZ");
  SerialPort.print((az < 0) ? (az + 360) : az, 1);
  SerialPort.print("\n");
}

void printEl() {
  //Print the rotator feedback data in Easycomm II format
  float elOut = el;
  if (elOut < 0.0) elOut = 0.0;
  if (elOut > 180.0) elOut = 180.0;
  SerialPort.print("EL");
  SerialPort.print(elOut, 1);
  SerialPort.print("\n");
}

void calibrate() {
  //Refresh accelerometer and magnetometer samples before updating calibration.
  lsm.readGM();
  //Process raw accelerometer and magnetometer samples
  bool changed = lsm.calibrate();
  //Print any changes and beep the speaker to facilitate manual calibration
  if (changed) {
    tone(spkPin, 2000, 80);         //Sound the passive piezo buzzer
    printCal();                     //Print the calibration data
  } else {
    noTone(spkPin);                 //Silence the piezo buzzer
  }
}

void getWindup(bool *windup,  float *azWindup, float *azOffset, float *azLast, float *elLast, float az, float elSet) {
  //Get the accumulated windup angle from the home position (startup or last reset position) and set the windup state if greater than the limit.
  //Get the raw difference angle between the current and last azimuth reading from the sensor
  float azDiff = az - *azLast;

  //Detect crossing South: azDiff jumps 360 for a clockwise crossing or -360 for an anticlockwise crossing
  //Increment the azimuth offset accordingly
  if (azDiff < -180) *azOffset += 360;
  if (azDiff > 180) *azOffset -= 360;

  //Save the current azimuth reading for the next iteration
  *azLast = az;

  //Compute the azimuth wind-up angle, i.e. the absolute number of degrees from the home position
  *azWindup = az + *azOffset;

  //Detect a windup condition where the antenna has rotated more than 450 degrees from home
  if (abs(*azWindup) > WINDUP_LIMIT) *windup = true;    //Set the windup condition - it is reset later when the antenna nears home

  //Perform the anti-windup procedure at the end of each pass - This is overkill unless you absolutely don't want anti-windup during a pass
  //  if (elSet <= 0)
  //    if (elLast > 0)
  //      if (mode == tracking) {
  //        *windup = true;
  //      }

  //Save the current elevation reading for the next iteration
  *elLast = elSet;
}

void getAzElDemo(float *azSet, float *elSet, float *azInc, float *elInc) {
  //Autoincrement the azimuth and elevation to demo the rotator operation
  if (*azSet > 180.0) *azInc = -*azInc;
  if (*azSet < -180.0) *azInc = -*azInc;
  if (*elSet > 90.0) *elInc = -*elInc;
  if (*elSet < 0.0) *elInc = -*elInc;
  *azSet += *azInc;
  *elSet += *elInc;
  SerialPort.print(*azSet, 0); SerialPort.print(",");
  SerialPort.println(*elSet, 0);
}

void getAzElError(float *azError, float *elError, bool *windup, float *azSet, float elSet, float az, float el) {
  //Compute the azimuth and elevation antenna pointing errors, i.e. angular offsets from set positions
  //Compute the azimuth antenna pointing error: Normally via the shortest path; opposite if windup detected.
  if (*windup) {                               //Check for a windup condition
    //To unwind the antenna set an azError in the appropriate direction to home
    *azError = constrain(azWindup, -180, 180); //Limit the maximum azimuth error to -180..180 degrees
    //Cancel the windup condition when the antenna is within 180 degrees of home (Actually 175 degrees to avoid rotation direction ambiguity)
    //Set a zero home position by default, but return azumith control to the computer if still connected
    if (abs(*azError) < 175) *windup = false; //Cancel windup and permit computer control
  }
  else {
    //Compute the normal azimuth antenna pointing error when there is no windup condition
    *azError = diffAngle(*azSet, az);
  }

  //Compute the elevation antenna pointing error
  *elError = diffAngle(elSet, el);
}

void processPosition() {
  //Perform the main operation of positioning the rotator under different modes
  switch (mode) {
    case debugging:
      lsm.readGM(); //Refresh raw sensor readings before printing debug output
      printDebug(); //Print the raw sensor data for debug purposes
      break;
    case calibrating:
      calibrate();  //Process calibration data
      break;
    case pausing:
      azMot.halt(); //Stop the AZ motor
      elMot.halt(); //Stop the EL motor
      break;
    default:
      lsm.readGM();  //Read accelerometer and magnetometer only when needed
      lsm.getAzEl();  //Get the azimuth and elevation of the antenna                                                              //Get the antenna AZ and EL
      az = lsm.az;
      el = lsm.el;
      //After reset, hold position until a tracking/demo command updates the set points.
      if (isnan(azSet) || isnan(elSet)) {
        azSet = az;
        elSet = el;
      }
      getWindup(&windup, &azWindup, &azOffset, &azLast, &elLast, az, elSet);      //Get the AZ windup angle and windup state
      if (mode == demonstrating) getAzElDemo(&azSet, &elSet, &azInc, &elInc);     //Set the AZ and EL automatically if in demo mode
      getAzElError(&azError, &elError, &windup, &azSet, elSet, az, el);           //Get the antenna pointing error
      if (mode == monitoring) printMon(az, el, azSet, elSet, azWindup, azError, elError); //Print the data if in monitor mode
  }
}

void processMotors() {
  //Only drive in active control modes; all diagnostic modes must keep motors stopped.
  switch (mode) {
    case tracking:
      if (trackingArmed) {
        azMot.drive(azError);
        elMot.drive(elError);
      } else {
        azMot.halt();
        elMot.halt();
      }
      break;
    case demonstrating:
      azMot.drive(azError);
      elMot.drive(elError);
      break;
    case manualdrive:
      azMot.drive(azManualErr);
      elMot.drive(elManualErr);
      break;
    default:
      azMot.halt();
      elMot.halt();
      break;
  }
}

void processUserCommands(const String &line) {
  //Process user commands
  //User command type 1: r, b, m, c, a, d, s, d, h, p or e<decl> followed by a carriage return
  //User command type 2: <az> <el> followed by a carriage return
  String normalized = line;
  normalized.trim();
  String normalizedUpper = normalized;
  normalizedUpper.toUpperCase();

  // Give priority to Easycomm-style commands so "az ..." is not treated as single-letter "a".
  if (normalizedUpper == "AZ" || normalizedUpper == "EL" ||
      normalizedUpper.startsWith("AZ ") || normalizedUpper.startsWith("EL ")) {
    processEasycommCommands(normalized, true);
    return;
  }

  String param;                                           //Parameter value
  int firstSpace;                                         //Position of the first space in the command line
  char command = line.charAt(0);                          //Get the first character
  if (command >= 'A' && command <= 'Z') command = command + ('a' - 'A');
  switch (command) {                                      //Process type 1 user commands
    case 'r':                                             //Reset command
      SerialPort.println("Reset in progress");
      reset(true);  //Reset the rotator and load calibration from EEPROM
      SerialPort.println("Reset complete");
      break;
    case 'b':                                             //Debug command
      SerialPort.println("Debugging in progress: Press 'a' to abort");
      mode = debugging;
      t1.reset(100);
      break;
    case 'm':                                             //Monitor command
      SerialPort.println("Monitoring in progress: Press 'a' to abort");
      mode = monitoring;
      t1.reset(100);
      break;
    case 'c':                                             //Calibrate command
      SerialPort.println("Calibration in progress: Press 'a' to abort or 's' to save");
      reset(false); //Reset the rotator, but don't load calibration from EEPROM
      mode = calibrating;
      t1.reset(50);
      break;
    case 'a':                                             //Abort command
      mode = tracking;
      t1.reset(100);
      reset(true);
      SerialPort.println("Function aborted");
      break;
    case 'e':                                             //Magnetic declination command
      param = line.substring(1);                          //Get the second parameter
      param.trim();
      if (param.length() == 0) {
        SerialPort.print("MagDecl=");
        SerialPort.println(lsm.cal.md, 1);
        break;
      }
      if (!parseSignedFloat(param, &lsm.cal.md)) {
        SerialPort.println("Usage: eNN.N (example: e4.5 or e-4.5)");
        break;
      }
      SerialPort.print("MagDecl set to ");
      SerialPort.println(lsm.cal.md, 1);
      break;
    case 's':                                             //Save command
      save();
      reset(true);
      SerialPort.println("Calibration saved");
      break;
    case 'd':                                             //Demo command
      SerialPort.println("Demo in progress: Press 'a' to abort");
      t1.reset(50);
      mode = demonstrating;
      break;
    case 'w':                                             //Factory clear calibration in EEPROM
      clearCalibration();
      reset(false);
      SerialPort.println("Calibration EEPROM cleared");
      break;
    case 'h':                                             //Help command
      SerialPort.println("Commands:");
      SerialPort.println("az el -(0..360 0..90)");
      SerialPort.println("r -Reset");
      SerialPort.println("eNN.N -MagDecl");
      SerialPort.println("c -Calibrate");
      SerialPort.println("s -Save");
      SerialPort.println("a -Abort");
      SerialPort.println("d -Demo");
      SerialPort.println("w -Clear calibration EEPROM");
      SerialPort.println("b -Debug");
      SerialPort.println("m -Monitor");
      SerialPort.println("p -Pause");
      break;
    case 'p':                                             //Pause command
      if (mode == pausing) {
        mode = tracking;
      } else {
        mode = pausing;
        SerialPort.println("Paused");
      }
      break;
    default:                                              //Process type 2 user commands
      firstSpace = line.indexOf(' ');                     //Get the index of the first space
      if (firstSpace <= 0) break;                         //Ignore invalid coordinate commands
      param = line.substring(0, firstSpace);              //Get the first parameter
      azSet = param.toFloat();                            //Get the azSet value
      param = line.substring(firstSpace + 1);             //Get the second parameter
      elSet = param.toFloat();                            //Get the elSet value
      trackingArmed = true;
      mode = tracking;
  }
}

void processEasycommCommands(const String &line) {
  processEasycommCommands(line, false);
}

void processEasycommCommands(const String &line, bool fromUserCommand) {
  //Process Easycomm II rotator commands
  //Easycomm II position command: AZnn.n ELnn.n UP000 XXX DN000 XXX\n
  //Easycomm II query command: AZ EL \n
  String cmd = line;
  cmd.trim();
  String cmdUpper = cmd;
  cmdUpper.toUpperCase();
  if (cmdUpper == "AZ") {                                 //Query command received
    lsm.readGM();
    lsm.getAzEl();
    az = lsm.az;
    el = lsm.el;
    printAz();                                            //Send the current Azimuth
  } else if (cmdUpper == "EL") {
    lsm.readGM();
    lsm.getAzEl();
    az = lsm.az;
    el = lsm.el;
    printEl();                                            //Send the current Elevation
  } else if (cmdUpper == "AZ EL") {
    // Hamlib rotctld Easycomm position poll uses combined "AZ EL" query.
    lsm.readGM();
    lsm.getAzEl();
    az = lsm.az;
    el = lsm.el;
    printAzEl();
  } else {
      //Accept: "AZnn.n ELnn.n", "AZ nn.n EL nn.n", "AZ EL nn.n nn.n", or plain "nn.n nn.n".
      String values = cmdUpper;
      values.replace("AZ", " ");
      values.replace("EL", " ");
      values.trim();

      int firstSpace = values.indexOf(' ');               //Split into azimuth and elevation tokens
      if (firstSpace <= 0) return;
      String azToken = values.substring(0, firstSpace);
      String elToken = values.substring(firstSpace + 1);
      azToken.trim();
      elToken.trim();
      if (azToken.length() == 0 || elToken.length() == 0) return;

      azSet = azToken.toFloat();                          //Set the azSet value
      if (azSet > 180) azSet = azSet - 360;               //Convert 0..360 to -180..180 degrees format
      elSet = elToken.toFloat();                          //Set the elSet value
      trackingArmed = true;
      mode = tracking;
      if (fromUserCommand) {
        SerialPort.println("Target accepted");
      }
  }
}

void processCommands(void) {
  //Process incoming data from the control computer
  //User commands are entered by the user and are terminated with a carriage return
  //Easycomm commands are generated by a tracking program and are terminated with a line feed
  static char eParam[24];
  static uint8_t eLen = 0;
  static bool collectingE = false;

  while (SerialPort.available() > 0) {
    char ch = SerialPort.read();                                //Read a single character from the serial buffer
    switch (ch) {
      case 13:                                                  //Carriage return received
        if (collectingE) {
          String eLine = "e";
          if (eLen > 0) eLine += String(eParam);
          processUserCommands(eLine);
        } else {
          processUserCommands(line);                              //Process user commands
        }
        line = "";                                              //Command processed: Clear the command line
        collectingE = false;
        eLen = 0;
        eParam[0] = '\0';
        break;
      case 10:                                                  //Line feed received
        processEasycommCommands(line);                          //Process Easycomm commands
        line = "";                                              //Command processed: Clear the command line
        collectingE = false;
        eLen = 0;
        eParam[0] = '\0';
        break;
      default:                                                  //Any other character received
        if (line.length() == 0) {
          line += ch;
          char c = ch;
          if (c >= 'A' && c <= 'Z') c = c + ('a' - 'A');
          collectingE = (c == 'e');
          eLen = 0;
          eParam[0] = '\0';
        } else {
          line += ch;                                           //Add this character to the command line
          if (collectingE && eLen < (sizeof(eParam) - 1)) {
            eParam[eLen++] = ch;
            eParam[eLen] = '\0';
          }
        }
        break;
    }
  }
}

void setup() {
  //Initialize the system
  //Set speaker pins to outputs
  pinMode(spkPin, OUTPUT);
  pinMode(gndPin, OUTPUT);
  digitalWrite(gndPin, LOW);

pinMode(azBrkPin, OUTPUT);
pinMode(elBrkPin, OUTPUT);
digitalWrite(azBrkPin, LOW);
digitalWrite(elBrkPin, LOW);
  //Initialize the serial port
  SerialPort.begin(9600);
  SerialPort.println("BOOT");
  //Reset the rotator and load configuration from EEPROM
  reset(true);
  //Initialize the sensor
  lsm.begin();
  SerialPort.println("READY");
}

void loop() {
  //Repeat continuously
  processCommands();                                              //Process commands from the control computer
  t1.execute(&processPosition);                                   //Process position only periodically
  processMotors();                                                //Process motor drive
}
