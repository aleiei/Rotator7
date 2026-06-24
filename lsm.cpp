//lsm.cpp - Library for LSM303 Accelerometer/Magnetometer.
//Copyright (c) 2015-2018 Julie VK3FOWL and Joe VK3YSP
//Released under the GNU General Public License.
//For more information please visit http://www.sarcnet.org
//Submitted for publication in Amateur Radio magazine: December 2015
//First published in Amateur Radio magazine: May 2016
//Upgraded Mk2 version published in Amateur Radio magazine: October 2016
//Reference: ST Datasheet: LSM303 Ultra-compact high-performance eCompass module: 3D accelerometer and 3D magnetometer
//There are two types of supported sensor boards containing the LSM303D or LSL303DHLC integrated circuits.
//Because the flat side of the sensor board is attached on the top of the antenna boom, with the long side of the sensor board parallel to the boom,
//the sensor axes (X', Y' and Z') are not the same as the reference axes (X, Y and Z) used in the software and our original article.
//X = -Y', Y = X' and Z = Z'. Also, the gravity field vector G is the opposite of the device acceleration vector A.
//Therefore the following transformations apply: MX = -MY', MY = MX', MZ = MZ', GX = AY', GY = -AX', GZ = -AZ'.
//Supports LSM303D or LSM303DLHC 3D Accelerometer/Magnetometer
#include "lsm.h"

static inline int16_t makeInt16(byte lo, byte hi) {
  return (int16_t)(((uint16_t)hi << 8) | (uint16_t)lo);
}

//Public methods

Lsm::Lsm(int type, float alpha): filMX(alpha), filMY(alpha), filMZ(alpha), filGX(alpha), filGY(alpha), filGZ(alpha) {
  //Constructor
  //Note: Wire.h cannot be initialized from a class constructor - see begin() method
  _type = type;
}

void Lsm::setType(int type) {
  _type = type;
  calStart();
  begin();
}

int Lsm::getType() {
  return _type;
}

void Lsm::begin() {
  //Reset the sensor
  reset();
  //Initialize the sensor filters
  for (int i = 0; i < 5; i++) readGM();
}

void Lsm::reset() {
  //Reset the sensor
  Wire.begin();                                         //Initialize the I2C bus
#if defined(TWBR)
  //Avoid permanent lock when SDA/SCL are held low or sensor misbehaves.
  Wire.setWireTimeout(3000, true);
#endif
  switch (_type) {
    case LSM303D:
      writeReg(LSM303D_ADDRESS, LSM303D_CTRL1, 0b01010111); //Acc output data rate = 50Hz all Acc axes enabled.
      writeReg(LSM303D_ADDRESS, LSM303D_CTRL2, 0b00000000); //Acc full scale = +/- 2g
      writeReg(LSM303D_ADDRESS, LSM303D_CTRL5, 0b01100100); //Mag output data rate = 6.25HzMag resolution = high;
      writeReg(LSM303D_ADDRESS, LSM303D_CTRL6, 0b00100000); //Mag full scale = +/- 4gauss
      writeReg(LSM303D_ADDRESS, LSM303D_CTRL7, 0b00000000); //Mag low power mode = Off. Mag sensor mode = Continuous-conversion
      break;
    case LSM303DLHC:
      // Some DLHC boards expose accelerometer at 0x19, others at 0x18 (SA0 low).
      writeReg(LSM303DLHC_ADDRESS_A, LSM303DLHC_CTRL_REG1_A, 0b01000111); //Acc output data rate = 50Hz all Acc axes enabled.
      writeReg(LSM303DLHC_ADDRESS_A, LSM303DLHC_CTRL_REG4_A, 0b00001000); //Acc full scale = +/- 2g, High Resolution Enable
      writeReg(LSM303DLHC_ADDRESS_A_ALT, LSM303DLHC_CTRL_REG1_A, 0b01000111);
      writeReg(LSM303DLHC_ADDRESS_A_ALT, LSM303DLHC_CTRL_REG4_A, 0b00001000);
      writeReg(LSM303DLHC_ADDRESS_M, LSM303DLHC_CRA_REG_M, 0b00011000); //Mag output data rate = 30Hz
      writeReg(LSM303DLHC_ADDRESS_M, LSM303DLHC_CRB_REG_M, 0b00101000); //Mag full scale = +/- 1.3g
      writeReg(LSM303DLHC_ADDRESS_M, LSM303DLHC_MR_REG_M, 0b00000000); //Mag Continuous Conversion Mode
      break;
    case LSM303AGR:
      // LSM303AGR accelerometer (LIS2DH12-compatible) at 0x19 or 0x18.
      writeReg(LSM303AGR_ADDRESS_A, LSM303AGR_CTRL_REG1_A, 0b01000111); //50 Hz, XYZ enabled
      writeReg(LSM303AGR_ADDRESS_A, LSM303AGR_CTRL_REG4_A, 0b00001000); //High-resolution, +/-2g
      writeReg(LSM303AGR_ADDRESS_A_ALT, LSM303AGR_CTRL_REG1_A, 0b01000111);
      writeReg(LSM303AGR_ADDRESS_A_ALT, LSM303AGR_CTRL_REG4_A, 0b00001000);

      // LSM303AGR magnetometer (LIS2MDL-compatible) at 0x1E (or 0x1C on some boards).
      // CFG_REG_A: temp compensation on, 10Hz, continuous mode.
      writeReg(LSM303AGR_ADDRESS_M, LSM303AGR_CFG_REG_A_M, 0b10000000);
      writeReg(LSM303AGR_ADDRESS_M, LSM303AGR_CFG_REG_B_M, 0b00000000);
      writeReg(LSM303AGR_ADDRESS_M, LSM303AGR_CFG_REG_C_M, 0b00010000); //BDU enabled
      writeReg(LSM303AGR_ADDRESS_M_ALT, LSM303AGR_CFG_REG_A_M, 0b10000000);
      writeReg(LSM303AGR_ADDRESS_M_ALT, LSM303AGR_CFG_REG_B_M, 0b00000000);
      writeReg(LSM303AGR_ADDRESS_M_ALT, LSM303AGR_CFG_REG_C_M, 0b00010000);
      break;
  }
}

void Lsm::calStart() {
  //Prepare to calibrate the sensor
  calMX.reset();
  calMY.reset();
  calMZ.reset();
  calGX.reset();
  calGY.reset();
  calGZ.reset();
}

void Lsm::readGM() {
  //Read the accelerometer and magnetometer
  //Do not reinitialize I2C/sensor on every sample; it can starve serial command handling.
  readG();
  readM();
}

bool Lsm::calibrate() {
  //Caculate the 3D scaling factors and errors for the magnetometer and accelerometer
  bool changed = false;
  //Update exisiting maximums and minimums based on current values
  changed = calMX.sample(mx, changed);
  changed = calMY.sample(my, changed);
  changed = calMZ.sample(mz, changed);
  changed = calGX.sample(gx, changed);
  changed = calGY.sample(gy, changed);
  changed = calGZ.sample(gz, changed);
  if (changed) {
    //Calculate the error vectors
    cal.me = Vec(calMX.offset, calMY.offset, calMZ.offset);
    cal.ge = Vec(calGX.offset, calGY.offset, calGZ.offset);
    //Caclulate the scaling vectors
    cal.ms = Vec(calMX.scale, calMY.scale, calMZ.scale);
    cal.gs = Vec(calGX.scale, calGY.scale, calGZ.scale);
  }
  return changed;
}

void Lsm::getAzEl() {
  //Get the antenna azimuth and elevation angles
  //Get the unit vectors for the earth's magnetic and gravitational fields
  //For each component subtract the error and divide by the scaling factor
  readGM();
  float msI = (abs(cal.ms.i) > 0.000001f) ? cal.ms.i : 1.0f;
  float msJ = (abs(cal.ms.j) > 0.000001f) ? cal.ms.j : 1.0f;
  float msK = (abs(cal.ms.k) > 0.000001f) ? cal.ms.k : 1.0f;
  float gsI = (abs(cal.gs.i) > 0.000001f) ? cal.gs.i : 1.0f;
  float gsJ = (abs(cal.gs.j) > 0.000001f) ? cal.gs.j : 1.0f;
  float gsK = (abs(cal.gs.k) > 0.000001f) ? cal.gs.k : 1.0f;
  Vec M = Vec((mx - cal.me.i) / msI, (my - cal.me.j) / msJ, (mz - cal.me.k) / msK).unit();
  //Fallback to raw normalized acceleration when calibration quality is poor.
  bool accelCalPoor = (abs(cal.gs.i) < 300.0f) || (abs(cal.gs.j) < 300.0f) || (abs(cal.gs.k) < 300.0f);
  Vec G;
  if (accelCalPoor) {
    G = Vec(gx, gy, gz).unit();
  } else {
    G = Vec((gx - cal.ge.i) / gsI, (gy - cal.ge.j) / gsJ, (gz - cal.ge.k) / gsK).unit();
  }
  //Define the antenna axes as the main reference axes
  const Vec X = Vec(1.0, 0.0, 0.0);         //The antenna X vector
  const Vec Y = Vec(0.0, 1.0, 0.0);         //The antenna Y (boresight) vector
  const Vec Z = Vec(0.0, 0.0, 1.0);         //The antenna Z vector
  //Compute the magnetic ground axes relative to the antenna axes
  Vec E = G.cross(M);                   //The magnetic East vector
  Vec N = E.cross(G);                   //The magnetic North vector
  Vec U = G.neg();                        //The magnetic Up vector
  //Compute the projections of the antenna axes onto the magnetic ground axes
  float Xn = X.dot(N);                     //The scalar projection of X onto N
  float Xe = X.dot(E);                     //The scalar projection of X onto E
  float Yu = Y.dot(U);                     //The scalar projection of Y onto U
  float Zu = Z.dot(U);                     //The scalar projection of Z onto U
  //Compute the true antenna pointing angles relative to the magnetic ground axes
  az = atan2(-Xn, Xe) * rad2deg + cal.md;  //The azimuth angle in degrees using the X-axis
  el = atan2(Yu, Zu) * rad2deg;            //The elevation angle in degrees using the Y-axis
  if (az > 180) az = az - 360;           //Ensure azimuth is in -180..180 format after adding D
}

//Private methods

void Lsm::readG() {
  switch (_type) {
    case LSM303D:
      //Read the 3D accelerometer to determine the gravitational field vector
      Wire.beginTransmission((byte)LSM303D_ADDRESS);
      Wire.write(LSM303D_OUT_X_L_A | 0x80);
      Wire.endTransmission();
      Wire.requestFrom((byte)LSM303D_ADDRESS, (byte)6);
      if (Wire.available() == 6) {
        //Read 8-bit values
        byte  xl = Wire.read();
        byte  xh = Wire.read();
        byte  yl = Wire.read();
        byte  yh = Wire.read();
        byte  zl = Wire.read();
        byte  zh = Wire.read();
        //Assemble 16-bit values and perform the axis transformation
        gx = makeInt16(yl, yh);
        gy = -makeInt16(xl, xh);
        gz = -makeInt16(zl, zh);
        //Low pass filter the sensor data as it improves the calibration procedure
        gx = filGX.lpf(gx);
        gy = filGY.lpf(gy);
        gz = filGZ.lpf(gz);
      }
      break;
    case LSM303DLHC:
      //Read the 3D accelerometer to determine the gravitational field vector.
      //Try both possible DLHC accelerometer I2C addresses.
      for (int pass = 0; pass < 2; pass++) {
        byte addr = (pass == 0) ? (byte)LSM303DLHC_ADDRESS_A : (byte)LSM303DLHC_ADDRESS_A_ALT;
        Wire.beginTransmission(addr);
        Wire.write(LSM303DLHC_OUT_X_L_A | 0x80);
        Wire.endTransmission();
        Wire.requestFrom(addr, (byte)6);
        if (Wire.available() == 6) {
          //Read 8-bit values
          byte  xl = Wire.read();
          byte  xh = Wire.read();
          byte  yl = Wire.read();
          byte  yh = Wire.read();
          byte  zl = Wire.read();
          byte  zh = Wire.read();
          //Assemble 16-bit values and perform the axis transformation
          gx = makeInt16(yl, yh);
          gy = -makeInt16(xl, xh);
          gz = -makeInt16(zl, zh);
          //Low pass filter the sensor data as it improves the calibration procedure
          gx = filGX.lpf(gx);
          gy = filGY.lpf(gy);
          gz = filGZ.lpf(gz);
          break;
        }
      }
      break;
    case LSM303AGR:
      //Read the 3D accelerometer to determine the gravitational field vector.
      for (int pass = 0; pass < 2; pass++) {
        byte addr = (pass == 0) ? (byte)LSM303AGR_ADDRESS_A : (byte)LSM303AGR_ADDRESS_A_ALT;
        Wire.beginTransmission(addr);
        Wire.write(LSM303AGR_OUT_X_L_A | 0x80);
        Wire.endTransmission();
        Wire.requestFrom(addr, (byte)6);
        if (Wire.available() == 6) {
          byte xl = Wire.read();
          byte xh = Wire.read();
          byte yl = Wire.read();
          byte yh = Wire.read();
          byte zl = Wire.read();
          byte zh = Wire.read();

          gx = makeInt16(yl, yh);
          gy = -makeInt16(xl, xh);
          gz = -makeInt16(zl, zh);

          gx = filGX.lpf(gx);
          gy = filGY.lpf(gy);
          gz = filGZ.lpf(gz);
          break;
        }
      }
      break;
  }
}

void Lsm::readM() {
  switch (_type) {
    case LSM303D:
      //Read the 3D magnetometer to determine the magnetic field vector
      Wire.beginTransmission((byte)LSM303D_ADDRESS);
      Wire.write(LSM303D_OUT_X_L_M | 0x80);
      Wire.endTransmission();
      Wire.requestFrom((byte)LSM303D_ADDRESS, (byte)6);
      if (Wire.available() == 6) {
        //Read 8-bit values
        byte xl = Wire.read();
        byte xh = Wire.read();
        byte yl = Wire.read();
        byte yh = Wire.read();
        byte zl = Wire.read();
        byte zh = Wire.read();
        //Assemble 16-bit values and perform the axis transformation
        mx = -makeInt16(yl, yh);
        my = makeInt16(xl, xh);
        mz = makeInt16(zl, zh);
        //Low pass filter the sensor data as it improves the calibration procedure
        mx = filMX.lpf(mx);
        my = filMY.lpf(my);
        mz = filMZ.lpf(mz);
      }
      break;
    case LSM303DLHC:
      //Read the 3D magnetometer to determine the magnetic field vector
      Wire.beginTransmission((byte)LSM303DLHC_ADDRESS_M);
      Wire.write(LSM303DLHC_OUT_X_H_M | 0x80);
      Wire.endTransmission();
      Wire.requestFrom((byte)LSM303DLHC_ADDRESS_M, (byte)6);
      if (Wire.available() == 6) {
        //Read 8-bit values
        byte xh = Wire.read();
        byte xl = Wire.read();
        byte zh = Wire.read();
        byte zl = Wire.read();
        byte yh = Wire.read();
        byte yl = Wire.read();
        //Assemble 16-bit values and perform the axis transformation
        mx = -makeInt16(yl, yh);
        my = makeInt16(xl, xh);
        mz = makeInt16(zl, zh);
        //Low pass filter the sensor data as it improves the calibration procedure
        mx = filMX.lpf(mx);
        my = filMY.lpf(my);
        mz = filMZ.lpf(mz);
      }
      break;
    case LSM303AGR:
      //Read the 3D magnetometer to determine the magnetic field vector.
      for (int pass = 0; pass < 2; pass++) {
        byte addr = (pass == 0) ? (byte)LSM303AGR_ADDRESS_M : (byte)LSM303AGR_ADDRESS_M_ALT;
        Wire.beginTransmission(addr);
        Wire.write(LSM303AGR_OUTX_L_REG_M);
        Wire.endTransmission();
        Wire.requestFrom(addr, (byte)6);
        if (Wire.available() == 6) {
          byte xl = Wire.read();
          byte xh = Wire.read();
          byte yl = Wire.read();
          byte yh = Wire.read();
          byte zl = Wire.read();
          byte zh = Wire.read();

          mx = -makeInt16(yl, yh);
          my = makeInt16(xl, xh);
          mz = makeInt16(zl, zh);

          mx = filMX.lpf(mx);
          my = filMY.lpf(my);
          mz = filMZ.lpf(mz);
          break;
        }
      }
      break;
  }
}

void Lsm::writeReg(byte address, byte reg, byte value) {
  //I2C write to register at address
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

byte Lsm::readReg(byte address, byte reg) {
  //I2C read from register at address
  byte value;
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(address, (byte)1);
  value = Wire.read();
  Wire.endTransmission();
  return value;
}
