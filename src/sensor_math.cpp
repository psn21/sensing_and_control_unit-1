#include "sensor_math.hpp"
#include "Fusion.h"
#include "config.hpp"
#include <math.h>
#include <time.h>
#include "MS5837.h"
#include "mpu6050.hpp"
MS5837 Depth_Sensor;

#define SAMPLE_RATE 100

FusionOffset offset;
FusionAhrs ahrs;
FusionEuler euler;
FusionVector gyroscope,accelerometer,magnetometer;
bool start = true;
double current_time=millis(),prev_time=current_time,deltaTime=0;

const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
const FusionMatrix softIronMatrix = {0.971, -0.048, -0.024, -0.048, 0.972, -0.011, -0.024, -0.011, 1.063};
const FusionVector hardIronOffset = {57.59, 88.22, 74.47};

void initializeSensorMath() 
{
    FusionOffsetInitialise(&offset, SAMPLE_RATE);
    FusionAhrsInitialise(&ahrs);

    const FusionAhrsSettings settings = {
            .convention = FusionConventionNwu,
            .gain = 0.5f,
            .gyroscopeRange = 250.0f, 
            .accelerationRejection = 20.0f,
            .magneticRejection = 20.0f,
            .recoveryTriggerPeriod = 3 * SAMPLE_RATE, 
    }; 
    FusionAhrsSetSettings(&ahrs, &settings);

}

void updateOffset(MPU6050 gyro)
{
    float raw_a[3],raw_g[3],raw_m[3];
    for (int  sample_no = 0; sample_no < NO_OF_SAMPLES ; sample_no++)
    {
    gyro.getSensorsReadings(raw_a[0], raw_a[1], raw_a[2], raw_g[0], raw_g[1], raw_g[2]);

    accelerometerOffset.array[0]+=raw_a[0];
    accelerometerOffset.array[1]+=raw_a[1];
    accelerometerOffset.array[2]+=raw_a[2];
    gyroscopeOffset.array[0]+=raw_g[0];
    gyroscopeOffset.array[1]+=raw_g[1];
    gyroscopeOffset.array[2]+=raw_g[2];

    accelerometerOffset.array[0]/= NO_OF_SAMPLES;
    accelerometerOffset.array[1]/= NO_OF_SAMPLES;
    accelerometerOffset.array[2]/= NO_OF_SAMPLES;
    gyroscopeOffset.array[0]/=NO_OF_SAMPLES;
    gyroscopeOffset.array[1]/=NO_OF_SAMPLES;
    gyroscopeOffset.array[2]/=NO_OF_SAMPLES;
    accelerometerOffset.array[2]-=G;
    }

}
void applyIMUCalibration(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &mx, float &my, float &mz) 
{
    gyroscope = {gx, gy, gz}; 
    accelerometer = {ax, ay, az}; 
    magnetometer = {mx, my, mz};
    
    gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
    accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
    magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

    gyroscope = FusionOffsetUpdate(&offset, gyroscope);
}

void applyDepthSensorCalibration(float& depth)
{
    depth=Depth_Sensor.depth();
}

void updateOrientation(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float &roll, float &pitch, float &yaw)
{
    float _roll, _pitch, _yaw;
    gyroscope = {gx, gy, gz}; 
    accelerometer = {ax, ay, az}; 
    magnetometer = {mx, my, mz};
    current_time=millis();
    deltaTime=current_time-prev_time;
    prev_time = current_time;
    if (start)
    {
         start = !start;
         deltaTime = 0;
    }
    FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime);
    const FusionEuler rotation = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
    const FusionVector translation = FusionAhrsGetEarthAcceleration(&ahrs);

    _roll =rotation.angle.roll;
    _pitch = rotation.angle.pitch;
    _yaw = rotation.angle.yaw;

  
    roll = 0.9 * roll + 0.1 * _roll;
    pitch = 0.9 * pitch + 0.1 * _pitch;
    yaw = 0.9 * yaw + 0.1 * _yaw;

    ax=translation.axis.x;
    ay=translation.axis.y;
    az=translation.axis.z;
}
