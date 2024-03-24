#ifndef COMMUNICATION_INTERFACE_HPP
#define COMMUNICATION_INTERFACE_HPP

#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>

void initializeCommunication();
void sendDepth(float depth);
void sendIMUReadings(float ax, float ay, float az, float gx, float gy, float gz,
                     float mx, float my, float mz);
void sendOrientation(float roll, float pitch, float yaw);
void throttleCb(const std_msgs::Int32MultiArray& pwm_msg);
void checkForCommands();
#endif  // COMMUNICATION_INTERFACE_HPP