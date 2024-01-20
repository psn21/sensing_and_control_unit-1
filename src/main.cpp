#include "main.hpp"
#include "communication_interface.hpp"
#include "Wire.h"
#include <ros.h>
float ax, ay, az, gx, gy, gz, mx, my, mz, depth,roll,pitch,yaw;
ros::NodeHandle nh;

std_msgs::Float64 depth_data;
geometry_msgs::Vector3 linear_acceleration;
geometry_msgs::Vector3 angular_velocity;
geometry_msgs::Vector3 magnetic_field;
geometry_msgs::Vector3 orientation_msg;
std_msgs::Int8MultiArray pwm_msg;

ros::Subscriber <std_msgs::Int8MultiArray> sub("pwm_values", &throttleCb);
ros::Publisher depth_pub("depth_data", &depth_data);
ros::Publisher pub1("linear_acceleration", &linear_acceleration);
ros::Publisher pub2("angular_velocity", &angular_velocity);
ros::Publisher pub3("magnetic_field", &magnetic_field);
ros::Publisher orientation_pub("euler_orientation", &orientation_msg);

void setup() 
{
  // Serial.begin(9600);
  // while(!Serial)
  // {
  //   delay(1);
  // }
  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  
  initializeCommunication();
  initializeIMU();
  initializeDepthSensor();
  callUpdateOffset();
  initializeSensorMath();
  //  Serial.println("Initialized");
  initializeThrusters();
  
}

void loop() 
{
  
    updateIMUReadings(ax, ay, az, gx, gy, gz, mx, my, mz);
    updateDepthSensorReadings(depth);
    applyIMUCalibration(ax, ay, az, gx, gy, gz, mx, my, mz);
    applyDepthSensorCalibration(depth);
    updateOrientation(ax, ay, az, gx, gy, gz, mx, my, mz, roll, pitch, yaw);
    sendIMUReadings(ax,ay,az,gx,gy,gz,mx,my,mz);
    sendDepth(depth);  
    sendOrientation(roll,pitch,yaw);
    checkForCommands();
}

