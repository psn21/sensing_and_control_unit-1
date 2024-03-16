#include "communication_interface.hpp"
#include "thruster_interface.hpp"
#include "config.hpp"
ros:: NodeHandle nh;
ros::Publisher accelerationAngularVelocityPub("acceleration_angularVelocity",
                                              &acceleration_angularvelocity);
ros::Publisher magneticFieldPub("magneticField", &magnetic_field_);
ros::Publisher OrientationRPYPub("orientation_RPY", &orientation_rpy);
ros::Publisher DepthDataPub("Depth", &depth_data);
ros::Subscriber<std_msgs::Int32MultiArray> PWMsub("pwm_values", &throttleCb);

void initializeCommunication() {
    // nh.initNode();
    // nh.subscribe(sub);
    // nh.advertise(depth_pub);
    // nh.advertise(pub1);
    // nh.advertise(pub2);
    // nh.advertise(pub3);
    // nh.advertise(orientation_pub);
  nh.initNode();
  nh.subscribe(PWMsub);
  nh.advertise(accelerationAngularVelocityPub);
  nh.advertise(magneticFieldPub);
  nh.advertise(OrientationRPYPub);
  nh.advertise(DepthDataPub);
}


void sendDepth(float depth) {
    depth_data.data = depth;
    DepthDataPub.publish(&depth_data);
}

void sendIMUReadings(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
    
  acceleration_angularvelocity.linear_acceleration.x = ax;
  acceleration_angularvelocity.linear_acceleration.y = ay;
  acceleration_angularvelocity.linear_acceleration.z = az;
  acceleration_angularvelocity.angular_velocity.x = gx;
  acceleration_angularvelocity.angular_velocity.y = gy;
  acceleration_angularvelocity.angular_velocity.z = gz;
  magnetic_field_.magnetic_field.x = mx;
  magnetic_field_.magnetic_field.y = my;
  magnetic_field_.magnetic_field.z = mz;
  accelerationAngularVelocityPub.publish(&acceleration_angularvelocity);
  magneticFieldPub.publish(&magnetic_field_);

    // linear_acceleration.x= ax/G;
    // linear_acceleration.y= ay/G;
    // linear_acceleration.z= az/G;

    // angular_velocity.x= gx;
    // angular_velocity.y= gy;
    // angular_velocity.z= gz;

    // magnetic_field.x= mx;
    // magnetic_field.y= my;
    // magnetic_field.z= mz;


    // pub1.publish(&linear_acceleration);
    // pub2.publish(&angular_velocity);
    // pub3.publish(&magnetic_field);
}

void sendOrientation(float roll, float pitch, float yaw)
{
    orientation_rpy.x = roll;
    orientation_rpy.y = pitch;
    orientation_rpy.z = yaw;
    OrientationRPYPub.publish(&orientation_rpy);
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
