#include "sensor_interface.hpp"
const int16_t led_bitmask[9] = {1, 2, 4, 8, 16, 32, 64, 128, 256};

const uint8_t led_pinmap[9] = {PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8};

void initializeLED();
void setLED(int16_t led_indicator);