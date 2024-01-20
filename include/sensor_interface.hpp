#ifndef SENSOR_INTERFACE_HPP
#define SENSOR_INTERFACE_HPP
#include <Arduino.h>

#define ACCEL_ID 0x8700A
#define MAG_ID 0x8700B

#define FXOS8700_ADDRESS 0x1F
#define MS5837_ADDR  0x76
#define MPU6050_ADDRESS 0x68


 void initializeIMU();
 void initializeDepthSensor();
 void callUpdateOffset();
 void updateIMUReadings(float& ax, float& ay, float& az, float& gx, float& gy, float& gz, float& mx, float& my, float& mz);
 void updateDepthSensorReadings(float& depth);
#endif // SENSOR_INTERFACE_HPP