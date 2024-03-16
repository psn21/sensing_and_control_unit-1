#ifndef COMMUNICATION_INTERFACE_HPP
#define COMMUNICATION_INTERFACE_HPP


#include<Arduino.h>
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

  sensor_msgs::Imu acceleration_angularvelocity;
  sensor_msgs::MagneticField magnetic_field_;
  geometry_msgs::Vector3 orientation_rpy;
  std_msgs::Int32MultiArray pwm_values;
  std_msgs::Float32 depth_data;

// extern std_msgs::Float64 depth_data;
// extern geometry_msgs::Vector3 linear_acceleration;
// extern geometry_msgs::Vector3 angular_velocity;
// extern geometry_msgs::Vector3 magnetic_field;
// extern geometry_msgs::Vector3 orientation_msg;
// extern std_msgs::Int32MultiArray pwm_msg;


// extern ros::NodeHandle nh;
// extern ros::Subscriber<std_msgs::Int32MultiArray> sub;
// extern ros::Publisher depth_pub;
// extern ros::Publisher pub1;
// extern ros::Publisher pub2;
// extern ros::Publisher pub3;
// extern ros::Publisher orientation_pub;
//   ros::NodeHandle nh;
//   ros::Publisher accelerationAngularVelocityPub;
//   ros::Publisher magneticFieldPub;
//   ros::Publisher DepthDataPub;
//     ros::Publisher OrientationRPYPub;

  void initializeCommunication();
  void sendDepth(float depth);
  void sendIMUReadings(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
  void sendOrientation(float roll, float pitch, float yaw);
  void throttleCb(const std_msgs::Int32MultiArray& pwm_msg);
  void sendDepth(float depth);
  void checkForCommands();
#endif // COMMUNICATION_INTERFACE_HPP