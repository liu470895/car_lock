#ifndef APP_BATTER_H
#define APP_BATTER_H
#include "OSAL.h"

/* BATTER - The  number is the same as the bit position */
#define BATTER                0x01           
#define HAL_BATTER_MODE_OFF   0x00
#define HAL_BATTER_MODE_ON    0x01   ////////////////////////////////////////////////////////
/*
 * Set the batter en ON/OFF.
 */
extern void batter_control(uint8 status);

#endif//APP_BATTER_H