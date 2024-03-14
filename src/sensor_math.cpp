#include "sensor_math.hpp"
#include "Fusion.h"
#include "config.hpp"
#include <math.h>
#include <time.h>
#include "MS5837.h"
#include "mpu6050.hpp"

#define SAMPLE_RATE 100

FusionOffset offset;
FusionAhrs ahrs;
FusionEuler euler;
FusionVector gyroscope,accelerometer,magnetometer;
bool start = true;
double current_time=millis(),prev_time=current_time,deltaTime=0;

const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
FusionVector gyroscopeOffset = {-1.11f, 3.57f, -1.02f};

const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
FusionVector accelerometerOffset = {-0.10f, 1.28f, -0.06f};

const FusionMatrix softIronMatrix = {0.979, -0.024, -0.019, -0.024, 0.983, 0.007, -0.019, 0.007, 1.040};
const FusionVector hardIronOffset = {53.31, 92.47, 80.06};

void initializeSensorMath() 
{
    FusionOffsetInitialise(&offset, SAMPLE_RATE);
    FusionAhrsInitialise(&ahrs);

    const FusionAhrsSettings settings = {
            .convention = FusionConventionNed,
            .gain = 0.3f,
            .gyroscopeRange = 250.0f, 
            .accelerationRejection = 25.0f,
            .magneticRejection = 25.0f,
            .recoveryTriggerPeriod = 3 * SAMPLE_RATE, 
    }; 
    FusionAhrsSetSettings(&ahrs, &settings);

}

void updateOffset(MPU6050 gyro)
{
    float raw_a[3],raw_g[3],raw_m[3];
    for (int  sample_no = 0; sample_no < NO_OF_SAMPLES ; sample_no++)
    {
    gyro.getSensorsReadings(raw_a[1], raw_a[0], raw_a[2], raw_g[1], raw_g[0], raw_g[2]);

    raw_a[1] = -raw_a[1];
    raw_g[1] = -raw_g[1] ;
    accelerometerOffset.array[0]+=raw_a[0];
    accelerometerOffset.array[1]+=raw_a[1];
    accelerometerOffset.array[2]+=raw_a[2];
    gyroscopeOffset.array[0]+=raw_g[0];
    gyroscopeOffset.array[1]+=raw_g[1];
    gyroscopeOffset.array[2]+=raw_g[2];
    }

    accelerometerOffset.array[0]/= NO_OF_SAMPLES;
    accelerometerOffset.array[1]/= NO_OF_SAMPLES;
    accelerometerOffset.array[2]/= NO_OF_SAMPLES;
    gyroscopeOffset.array[0]/=NO_OF_SAMPLES;
    gyroscopeOffset.array[1]/=NO_OF_SAMPLES;
    gyroscopeOffset.array[2]/=NO_OF_SAMPLES;
    accelerometerOffset.array[2]-=G;
    

}
void applyIMUCalibration(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &mx, float &my, float &mz) 
{
    gyroscope = {gx, gy, gz}; 
    accelerometer = {ax, ay, az}; 
    magnetometer = {mx, my, mz};
    // FusionVector gyro_array= FusionVectorSubtract(gyroscope, gyroscopeOffset);
    // Serial.println(gyro_array.axis.x);
    // Serial.println(gyro_array.axis.y);
    // Serial.println(gyro_array.axis.z);

    // ax = ax- accelerometerOffset.axis.x;
    // ay = ay - accelerometerOffset.axis.y;
    // az = az - accelerometerOffset.axis.z;
    // gyroscope = FusionVectorSubtract(gyroscope,gyroscopeOffset);
    // accelerometer = FusionVectorSubtract(accelerometer,accelerometerOffset);
    // magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);
    // gx = gx - gyroscopeOffset.axis.x;
    // gy = gy - gyroscopeOffset.axis.y;
    // gz = gz - gyroscopeOffset.axis.z;
    gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
    accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
    magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);
    
    // Serial.print("gx: "); Serial.println(gyroscopeOffset.axis.x);
    // Serial.print("gy "); Serial.println(gyroscopeOffset.axis.y);
    // Serial.print("gz "); Serial.println(gyroscopeOffset.axis.z);

    // Serial.print("ax: "); Serial.println(accelerometerOffset.axis.x);
    // Serial.print("ay "); Serial.println(accelerometerOffset.axis.y);
    // Serial.print("az "); Serial.println(accelerometerOffset.axis.z);

    ax = accelerometer.axis.x;
    ay = accelerometer.axis.y;
    az = accelerometer.axis.z;

    gx = gyroscope.axis.x;
    gy = gyroscope.axis.y;
    gz = gyroscope.axis.z;

    mx = magnetometer.axis.x;
    my = magnetometer.axis.y;
    mz = magnetometer.axis.z;
    gyroscope = FusionOffsetUpdate(&offset, gyroscope);
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
     
     roll = _roll;
     pitch = _pitch;
     yaw= _yaw;
    
  
    // roll = 0.9 * roll + 0.1 * _roll;
    // pitch = 0.9 * pitch + 0.1 * _pitch;
    // yaw = 0.9 * yaw + 0.1 * _yaw;

    ax=translation.axis.x;
    ay=translation.axis.y;
    az=translation.axis.z;
}
