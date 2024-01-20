#include "sensor_interface.hpp"
#include "sensor_math.hpp"
#include <mpu6050.hpp>
#include "Adafruit_FXOS8700.h"
#include "MS5837.h"

MPU6050 gyro;
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(ACCEL_ID , MAG_ID);
MS5837 depth_sensor;

 void initializeIMU() 
{
    sensor_t accel,mag;
    accelmag.begin();
    gyro.begin();
    gyro.setAccelerometerRange(ACCELERO_METER_RANGE_2);
    gyro.setGyroscopeRange(GYROSCOPE_RANGE_250);
    gyro.setSampleRateDivider(0);
    gyro.disableSleepMode();      
}

 void initializeDepthSensor() 
{
    bool depth_sensor_status= depth_sensor.init();
    
    depth_sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
}

 void updateIMUReadings(float& ax, float& ay, float& az, float& gx, float& gy, float& gz, float& mx, float& my, float& mz) 
{
    sensors_event_t accel_event,mag_event;
    accelmag.getEvent(&accel_event,&mag_event);
    
    gyro.getSensorsReadings(ax, ay, az, gx, gy, gz);

    mx=mag_event.magnetic.x;
    my=mag_event.magnetic.y;
    mz=mag_event.magnetic.z;
}
void callUpdateOffset()
{
    updateOffset(gyro);
}

 void updateDepthSensorReadings(float& depth) 
{
    depth_sensor.read();
    depth=depth_sensor.depth();
}

