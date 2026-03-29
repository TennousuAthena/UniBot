/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-30 10:34:30
 * @LastEditors: Changhua
 * @Description: MPU6050 Data solution
 * @FilePath: 
 */

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "MPU6050_getdata.h"
#include <stdio.h>
#include <math.h>

MPU6050 accelgyro;
MPU6050_getdata MPU6050Getdata;

// static void MsTimer2_MPU6050getdata(void)
// {
//   sei();
//   int16_t ax, ay, az, gx, gy, gz;
//   accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //Read the raw values of the six axes
//   float gyroz = -(gz - MPU6050Getdata.gzo) / 131 * 0.005f;
//   MPU6050Getdata.yaw += gyroz;
// }

bool MPU6050_getdata::MPU6050_dveInit(void)
{
  Wire.begin();
  uint8_t chip_id = 0x00;
  uint8_t cout = 0;
  do
  {
    chip_id = accelgyro.getDeviceID();
    Serial.print("MPU6050_chip_id: ");
    Serial.println(chip_id);
    delay(10);
    cout += 1;
    if (cout > 10)
    {
      return true;
    }
  } while (chip_id == 0X00 || chip_id == 0XFF); //Ensure that the slave device is online（Wait forcibly to get the ID）
  accelgyro.initialize();
  // unsigned short times = 100; //Sampling times
  // for (int i = 0; i < times; i++)
  // {
  //   gz = accelgyro.getRotationZ();
  //   gzo += gz;
  // }
  // gzo /= times; //Calculate gyroscope offset
  return false;
}
bool MPU6050_getdata::MPU6050_calibration(void)
{
  gzo = 0;
  agz = 0;
  // 200 samples with 5 ms spacing gives the ODR time to advance between reads,
  // producing a much more accurate zero-rate offset (~1 second total).
  unsigned short times = 200;
  for (int i = 0; i < times; i++)
  {
    gz = accelgyro.getRotationZ();
    gzo += gz;
    delay(5);
  }
  gzo /= times; //Calculate gyroscope offset
  lastTime = millis();
  return false;
}
bool MPU6050_getdata::MPU6050_dveGetEulerAngles(float *Yaw)
{
  unsigned long now = millis();            // Record the current time (ms)
  dt = (now - lastTime) / 1000.0;          // Derivative time (s)
  lastTime = now;
  gz = accelgyro.getRotationZ();

  // Apply deadband to the angular RATE (deg/s), not to the per-step angle delta.
  // Old code compared the integrated delta (rate * dt) against 0.05, which made
  // the effective deadband ~4 deg/s and suppressed real motion.
  float rate_degps = -(gz - gzo) / 131.0f; // Convert LSB to deg/s
  if (fabs(rate_degps) < 0.5f)             // 0.5 deg/s threshold for zero-drift rejection
  {
    rate_degps = 0.0f;
  }
  lastRate = rate_degps;                   // Store angular rate for external use
  agz += rate_degps * dt; // Integrate to get yaw angle (degrees)
  *Yaw = agz;
  return false;
}
