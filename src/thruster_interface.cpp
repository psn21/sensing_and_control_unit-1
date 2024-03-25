#include "thruster_interface.hpp"

#include "config.hpp"

Servo g_thrusters[NUMBER_OF_THRUSTERS];
const uint8_t g_kPinMap[NUMBER_OF_THRUSTERS] = THRUSTER_PINS;
/**
 * @brief Initialize thrusters, set initial PWM for idle state.
 *
 * Attaches each thruster to its pin, sets PWM to INIT_THRUSTER_PWM (idle
 * state), readies thrusters for control without active propulsion.
 *
 * @note Ensure that g_thrusters[] and g_kPinMap[] are configured.
 *
 * @param None
 * @return None
 */

void initializeThrusters() {
  for (int thruster_index = 0; thruster_index < NUMBER_OF_THRUSTERS;
       thruster_index++) {
    g_thrusters[thruster_index].attach(g_kPinMap[thruster_index]);
    g_thrusters[thruster_index].writeMicroseconds(INIT_THRUSTER_PWM);
  }
}

/**
 * @brief Set thruster pwm_values based on throttle values.
 *
 * Sets pwm values for each thruster using provided throttle values.
 * Throttle values are scaled from [-100, 100] to [1000, 2000] (PWM range).
 *
 * @param throttles An array of int8_t values representing throttle for each
 * thruster. Array size should be NUMBER_OF_THRUSTERS.
 *
 * @return None
 */

void setThrusterThrottle(const int32_t *pwm_values) {
  int pwm_value;
  for (int thruster_index = 0; thruster_index < NUMBER_OF_THRUSTERS;
       thruster_index++) {
    pwm_value = pwm_values[thruster_index];
    g_thrusters[thruster_index].writeMicroseconds(pwm_value);
  }
}
