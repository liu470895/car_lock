#ifndef APP_motor_H
#define APP_motor_H
#include "OSAL.h"

/* carlocktarge - The  number is the same as the bit position */
#define   carlock0degree         0xc0           
#define   carlock90degree        0x00           
#define   carlockuping           0x01           
#define   carlockdowning         0x02           
#define   carlockidle            0x03           
/* fuction  is configured by enable or disable*/
#define   Enable                 0x01           
#define   Disable                0x00           

/* motor  state*/
#define   Stoped                 0x00           
#define   Runing                 0x01          


extern uint8 motorctrol( uint8 fuction ,uint8 keystatus,uint8 motor_target);
extern void stop(void);

#endif//APP_motor_H