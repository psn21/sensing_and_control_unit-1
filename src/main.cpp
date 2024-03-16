#include "main.hpp"
#include "communication_interface.hpp"
#include "Wire.h"
#include <ros.h>
float ax, ay, az, gx, gy, gz, mx, my, mz, depth,roll,pitch,yaw;
// ros::NodeHandle nh;

// std_msgs::Float64 depth_data;
// geometry_msgs::Vector3 linear_acceleration;
// geometry_msgs::Vector3 angular_velocity;
// geometry_msgs::Vector3 magnetic_field;
// geometry_msgs::Vector3 orientation_msg;
// std_msgs::Int32MultiArray pwm_msg;

// ros::Subscriber <std_msgs::Int32MultiArray> sub("pwm_values", &throttleCb);
// ros::Publisher depth_pub("depth_data", &depth_data);
// ros::Publisher pub1("linear_acceleration", &linear_acceleration);
// ros::Publisher pub2("angular_velocity", &angular_velocity);
// ros::Publisher pub3("magnetic_field", &magnetic_field);
// ros::Publisher orientation_pub("euler_orientation", &orientation_msg);

Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(ACCEL_ID , MAG_ID);
MPU6050 gyro;
MS5837 depth_sensor;

long int previous_update_rate=0,previous_publish_rate=0;

void setup() 
{
  Serial.begin(9600);
  while(!Serial)
  {
    delay(1);
  }
  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  
  initializeCommunication();
  initializeIMU();
  initializeDepthSensor();
  // callUpdateOffset();
  initializeSensorMath();
  //  Serial.println("Initialized");
  initializeThrusters();
  previous_update_rate = millis();
  previous_publish_rate = millis();
  
}

void loop() 
{
    if((millis()-previous_update_rate )>= UPDATE_RATE);
    {
      previous_update_rate = millis();
    updateIMUReadings(ax, ay, az, gx, gy, gz, mx, my, mz);
    updateDepthSensorReadings(depth);
    applyIMUCalibration(ax, ay, az, gx, gy, gz, mx, my, mz);
    updateOrientation(ax, ay, az, gx, gy, gz, mx, my, mz, roll, pitch, yaw);
    }

    if((millis() - previous_publish_rate )>= PUBLISH_RATE)
    {
      previous_publish_rate = millis();
    sendIMUReadings(ax,ay,az,gx,gy,gz,mx,my,mz);
    sendDepth(depth);  
    sendOrientation(roll,pitch,yaw);

    Serial.print("roll: "); Serial.println(roll);
    Serial.print("pitch: "); Serial.println(pitch);
    Serial.print("yaw: "); Serial.println(yaw);
    }
    checkForCommands();
    // Serial.print("roll: "); Serial.println(ax);
    // Serial.print("pitch: "); Serial.println(ay);
    // Serial.print("yaw: "); Serial.println(az);
    // Serial.print("gx: "); Serial.println(mx);
    // Serial.print("gy: "); Serial.println(my);
    // Serial.print("gz: "); Serial.println(mz);

      
      // Serial.print("Depth: "); Serial.println(depth);

}

