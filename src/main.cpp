#include "main.hpp"

#include <ros.h>

#include "Wire.h"

float ax, ay, az, gx, gy, gz, mx, my, mz, depth, roll, pitch, yaw;

Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(ACCEL_ID, MAG_ID);
MPU6050 gyro;
MS5837 depth_sensor;

long int previous_update_rate = 0, previous_publish_rate = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    delay(1);
  }
  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();

  initializeCommunication();
  initializeIMU();
  initializeDepthSensor();
  initializeSensorMath();
  initializeThrusters();
  previous_update_rate = micros();
  previous_publish_rate = millis();
}

void loop() {
  if ((micros() - previous_update_rate) >= UPDATE_RATE)
    ;
  {
    previous_update_rate = micros();
    updateIMUReadings(ax, ay, az, gx, gy, gz, mx, my, mz);
    updateDepthSensorReadings(depth);
    applyIMUCalibration(ax, ay, az, gx, gy, gz, mx, my, mz);
    updateOrientation(ax, ay, az, gx, gy, gz, mx, my, mz, roll, pitch, yaw);
  }

  if ((millis() - previous_publish_rate) >= PUBLISH_RATE) {
    previous_publish_rate = millis();
    sendIMUReadings(ax, ay, az, gx, gy, gz, mx, my, mz);
    sendOrientation(roll, pitch, yaw);
    sendDepth(depth);

    // Serial.print("roll: "); Serial.println(roll);
    // Serial.print("pitch: "); Serial.println(pitch);
    // Serial.print("yaw: "); Serial.println(yaw);
  }
  checkForCommands();
}
