#ifndef COMMUNICATION_INTERFACE_HPP
#define COMMUNICATION_INTERFACE_HPP


#include<Arduino.h>
#include <ros.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>



extern std_msgs::Float64 depth_data;
extern geometry_msgs::Vector3 linear_acceleration;
extern geometry_msgs::Vector3 angular_velocity;
extern geometry_msgs::Vector3 magnetic_field;
extern geometry_msgs::Vector3 orientation_msg;
extern std_msgs::Int8MultiArray pwm_msg;


extern ros::NodeHandle nh;
extern ros::Subscriber<std_msgs::Int8MultiArray> sub;
extern ros::Publisher depth_pub;
extern ros::Publisher pub1;
extern ros::Publisher pub2;
extern ros::Publisher pub3;
extern ros::Publisher orientation_pub;

void initializeCommunication();
void sendDepth(float depth);
void sendIMUReadings(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void sendOrientation(float roll, float pitch, float yaw);
void throttleCb(const std_msgs::Int8MultiArray& pwm_msg);
void checkForCommands();
#endif // COMMUNICATION_INTERFACE_HPP