#include "communication_interface.hpp"

#include "config.hpp"
#include "diagnostics.hpp"
#include "thruster_interface.hpp"

geometry_msgs::Vector3 linearAcceleration;
geometry_msgs::Vector3 angularVelocity;
sensor_msgs::MagneticField magneticField;
geometry_msgs::Vector3 orientation_RPY;
std_msgs::Int32MultiArray pwm_values;
std_msgs::Float32 depth_data;
std_msgs::Bool calibration_status;
ros::NodeHandle nh;

ros::Publisher linearAccelerationPub("/sensors/linear_acceleration",
                                     &linearAcceleration);
ros::Publisher AngularVelocityPub("/sensors/angular_velocity",
                                  &angularVelocity);
ros::Publisher magneticFieldPub("/sensors/magnetic_field", &magneticField);
ros::Publisher OrientationRPYPub("/sensors/orientation", &orientation_RPY);
ros::Publisher DepthDataPub("/sensors/depth", &depth_data);
ros::Subscriber<std_msgs::Int32MultiArray> PWMsub("/control/pwm", &throttleCb);
ros::Subscriber<std_msgs::Bool> calibrationSub("/control/calibration",
                                               &calibrationCb);
ros::Subscriber<std_msgs::Int16> diagnosticSub("/control/led", &ledCb);

void initializeCommunication() {
  nh.initNode();
  nh.subscribe(PWMsub);
  nh.subscribe(calibrationSub);
  nh.advertise(linearAccelerationPub);
  nh.advertise(AngularVelocityPub);
  nh.advertise(magneticFieldPub);
  nh.advertise(OrientationRPYPub);
  nh.advertise(DepthDataPub);
}

void sendDepth(float depth) {
  depth_data.data = depth;
  DepthDataPub.publish(&depth_data);
}

void sendIMUReadings(float ax, float ay, float az, float gx, float gy, float gz,
                     float mx, float my, float mz) {
  linearAcceleration.x = ax * G;
  linearAcceleration.y = ay * G;
  linearAcceleration.z = az * G;

  angularVelocity.x = gx;
  angularVelocity.y = gy;
  angularVelocity.z = gz;

  magneticField.magnetic_field.x = mx;
  magneticField.magnetic_field.y = my;
  magneticField.magnetic_field.z = mz;

  linearAccelerationPub.publish(&linearAcceleration);
  AngularVelocityPub.publish(&angularVelocity);
  magneticFieldPub.publish(&magneticField);
}

void sendOrientation(float roll, float pitch, float yaw) {
  orientation_RPY.x = roll;
  orientation_RPY.y = pitch;
  orientation_RPY.z = yaw;
  OrientationRPYPub.publish(&orientation_RPY);
}
void throttleCb(const std_msgs::Int32MultiArray& pwm_msg) {
  int32_t pwm_values[NUMBER_OF_THRUSTERS];
  for (int thruster_index = 0; thruster_index < NUMBER_OF_THRUSTERS;
       thruster_index++) {
    pwm_values[thruster_index] = pwm_msg.data[thruster_index];
  }
  setThrusterThrottle(pwm_values);
}

void calibrationCb(const std_msgs::Bool& calibration_status) {
  bool calibration_mode = calibration_status.data;
  nh.loginfo("Calibration Mode received.");
  callUpdateOffset(calibration_mode);
}

void ledCb(const std_msgs::Int16& led_msg) {
  int16_t led_indicator = led_msg.data;
  setLED(led_indicator);
}

void checkForCommands() { nh.spinOnce(); }
