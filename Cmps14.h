#ifndef _CMPS_H
#define _CMPS_H
// Digital Compass CMPS14
// Copyright (C) 2021 https://www.roboticboat.uk
// ccc3d672-cfb3-4df4-b0af-7b80a580dded
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
// These Terms shall be governed and construed in accordance with the laws of 
// England and Wales, without regard to its conflict of law provisions.


#include <Wire.h>

// Register Function
// 0        Command register

// 1        Compass Bearing as a byte, i.e. 0-255 for a full circle
// 2,3      Compass Bearing as a word, i.e. 0-3599 for a full circle, representing 0-359.9 degrees. Register 2 being the high byte

// 4        Pitch angle - signed byte giving angle in degrees from the horizontal plane, Kalman filtered with Gyro
// 5        Roll angle - signed byte giving angle in degrees from the horizontal plane, Kalman filtered with Gyro

// 6,7      Magnetometer X axis raw output, 16 bit signed integer with register 6 being the upper 8 bits
// 8,9      Magnetometer Y axis raw output, 16 bit signed integer with register 8 being the upper 8 bits
// 10,11    Magnetometer Z axis raw output, 16 bit signed integer with register 10 being the upper 8 bits

// 12,13    Accelerometer  X axis raw output, 16 bit signed integer with register 12 being the upper 8 bits
// 14,15    Accelerometer  Y axis raw output, 16 bit signed integer with register 14 being the upper 8 bits
// 16,17    Accelerometer  Z axis raw output, 16 bit signed integer with register 16 being the upper 8 bits

// 18,19    Gyro X axis raw output, 16 bit signed integer with register 18 being the upper 8 bits
// 20,21    Gyro Y axis raw output, 16 bit signed integer with register 20 being the upper 8 bits
// 22,23    Gyro Z axis raw output, 16 bit signed integer with register 22 being the upper 8 bits

//---------------------------------

  //Address of the CMPS14 compass on i2c
  #define CMPS14_I2C_ADDRESS 0x60

  #define CONTROL_Register 0

  #define BEARING_Register 2 
  #define PITCH_Register 4 
  #define ROLL_Register 5

  #define MAGNETX_Register  6
  #define MAGNETY_Register  8
  #define MAGNETZ_Register 10

  #define ACCELEROX_Register 12
  #define ACCELEROY_Register 14
  #define ACCELEROZ_Register 16

  #define GYROX_Register 18
  #define GYROY_Register 20
  #define GYROZ_Register 22

  #define ONE_BYTE   1
  #define TWO_BYTES  2
  #define FOUR_BYTES 4
  #define SIX_BYTES  6

//---------------------------------

  byte _byteHigh;
  byte _byteLow;

  // Please note without clear documentation in the technical documenation
  // it is notoriously difficult to get the correct measurement units.
  // I've tried my best, and may revise these numbers.

  int bearing;
  int nReceived;
  signed char pitch;
  signed char roll;

  float magnetX = 0;
  float magnetY = 0;
  float magnetZ = 0;

  float accelX = 0;
  float accelY = 0;
  float accelZ = 0;
  // The acceleration along the X-axis, presented in mg 
  // See BNO080_Datasheet_v1.3 page 21
  float accelScale = 9.80592991914f/1000.f; // 1 m/s^2
  
  float gyroX = 0;
  float gyroY = 0;
  float gyroZ = 0;
  // 16bit signed integer 32,768
  // Max 2000 degrees per second - page 6
  float gyroScale = 1.0f/16.f; // 1 Dps

int16_t getBearing()
{
  char buff[256];
  // Begin communication with CMPS14
  Wire.beginTransmission(CMPS14_I2C_ADDRESS);

  // Tell register you want some data
  Wire.write(BEARING_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return 0;}
  
  // Request 2 bytes from CMPS14
  nReceived = Wire.requestFrom(CMPS14_I2C_ADDRESS , TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate full bearing
  bearing = ((_byteHigh<<8) + _byteLow) / 10;
  return bearing;
}
 #endif
