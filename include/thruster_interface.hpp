#ifndef THRUSTER_INTERFACE_HPP
#define THRUSTER_INTERFACE_HPP
#include <Arduino.h>
#include <Servo.h> 
void initializeThrusters();
void setThrusterThrottle(const int32_t *pwm_values);
// void setThrusterThrottles(const int8_t *throttles);
#endif // THRUSTER_INTERFACE_HPP