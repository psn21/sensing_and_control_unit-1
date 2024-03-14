#include "communication_interface.hpp"
#include "thruster_interface.hpp"
#include "config.hpp"

void initializeCommunication() {
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(depth_pub);
    nh.advertise(pub1);
    nh.advertise(pub2);
    nh.advertise(pub3);
    nh.advertise(orientation_pub);
}


void sendDepth(float depth) {
    depth_data.data = depth;
    depth_pub.publish(&depth_data);
}

void sendIMUReadings(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
    linear_acceleration.x= ax/G;
    linear_acceleration.y= ay/G;
    linear_acceleration.z= az/G;

    angular_velocity.x= gx;
    angular_velocity.y= gy;
    angular_velocity.z= gz;

    magnetic_field.x= mx;
    magnetic_field.y= my;
    magnetic_field.z= mz;

    pub1.publish(&linear_acceleration);
    pub2.publish(&angular_velocity);
    pub3.publish(&magnetic_field);
}

void sendOrientation(float roll, float pitch, float yaw)
{
    orientation_msg.x = roll;
    orientation_msg.y = pitch;
    orientation_msg.z = yaw;
    orientation_pub.publish(&orientation_msg);
}
void throttleCb(const std_msgs::Int32MultiArray& pwm_msg){
    int32_t pwm_values[NUMBER_OF_THRUSTERS];
    for (int thruster_index = 0; thruster_index < NUMBER_OF_THRUSTERS; thruster_index ++)
    {
        pwm_values[thruster_index] = pwm_msg.data[thruster_index];
    }
    setThrusterThrottle(pwm_values);
}

void checkForCommands(){
    nh.spinOnce();
}
