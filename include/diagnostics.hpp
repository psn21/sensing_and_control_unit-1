#include "sensor_interface.hpp"
const int16_t led_bitmask[9] = {0x0001, 0x0003, 0x0007, 0x000F, 0x001F,
                                0x003F, 0x007F, 0x00FF, 0x01FF};

const uint8_t led_pinmap[9] = {PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8};

void setLED(int16_t led_indicator);