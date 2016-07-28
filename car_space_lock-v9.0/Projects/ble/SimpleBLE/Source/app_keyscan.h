#ifndef APP_keyscan_H
#define APP_keyscan_H
#include "OSAL.h"

/* KEY - The  number is the same as the bit position */
#define   KEY                   (P0&0XC0)           
#define   KEY_X2                0x80           
#define   KEY_X1                0x40           

/* carlockstate - The  number is the same as the bit position */
#define   carlock0state         0xc0           
#define   carlock0_90state      0x40           
#define   carlock90state        0x00           
#define   carlock90_180state    0x80           

extern uint8 keyscan(void);

#endif//APP_keyscan_H