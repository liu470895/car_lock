#ifndef _SX1276_LPC1114_H
#define _SX1276_LPC1114_H

#include "comdef.h"
#include "ioCC2541.h"

/*********************************************************************************************************
  宏定义
*********************************************************************************************************/
#define PA_TX_L   (P0_3 = 0)	 // TX P0.3
#define PA_TX_H   (P0_3 = 1) 	 // 

#define PA_RX_L   (P0_4 = 0)	 // RX P0.4
#define PA_RX_H   (P0_4 = 1)	  // 

//#define SXRESET_L   (LPC_GPIO2->DATA&=~(1<<3))	 // RST
//#define SXRESET_H   (LPC_GPIO2->DATA|=(1<<3))	  // 


#define CSN_Low   (P1_4 = 0)	  // p1.4
#define CSN_High  (P1_4 = 1)	    // p1.4 

#define DIO0_Low   (P1_3 = 0)    //p1.3
#define DIO0_High  (P1_3 = 1)	    //p1.3

typedef enum
{
    RADIO_RESET_OFF,
    RADIO_RESET_ON,
}tRadioResetState; 

//===================================SPI函数声明===================================================
void SX1276InitIo( void );
void SX1276SPISetup(void);
void SX1276WriteBuffer( uint8 addr, uint8 *buffer, uint8 size );
void SX1276ReadBuffer( uint8 addr, uint8 *buffer, uint8 size );
void SX1276Write( uint8 addr, uint8 data );
void SX1276Read( uint8 addr, uint8 *data );
void SX1276WriteFifo( uint8 *buffer, uint8 size );
void SX1276ReadFifo( uint8 *buffer, uint8 size );
void SX1276Reset(void);
extern void SX1276WriteRxTx( uint8 txEnable );
uint8 SPI1_communication(uint8 TxData);
void SPI1_Init(void);
uint8 SPI1_communication(uint8 TxData);

#endif	
