#ifndef APP_LED_CONTROL_H
#define APP_LED_CONTROL_H
#include "OSAL.h"

void led_control(uint8 leds, uint8 numBlinks, uint8 percent, uint16 period, uint8 led_status);
#endif//APP_LED_CONTROL_H