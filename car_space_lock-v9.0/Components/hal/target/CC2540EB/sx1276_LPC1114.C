
#include "SX1276_LPC1114.h"
#include "comdef.h"
#include "hal_mcu.h"

////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF初始化IO口
// 输入参数 : 无
// 返回参数 : 无
// 说明     : 
////////////////////////////////////////////////////////////////////////////////
void SX1276InitIo( void )
{  
    P0SEL &= ~(1<<3 | 1<<4);
    P0DIR |= (1<<3 | 1<<4);
    
    P1SEL &= ~(1<<3);
    P1DIR &= ~(1<<3);
    P1INP |= (1<<3);
    P1IEN |= (1<<3);     // P1.3 设置为中断方式 1：中断使能
    PICTL &= ~0x02;      //上升沿触发   
    IEN2 |= 0x10;        //允许P1口中断; 
    P1IFG = ~(1<<3);     //清中断标志位
    P1IF = 0;            //清中断标志位
    EA = 1;              //打开总中断
   
    
//    P0IEN |= 0x2;    // P0.1 设置为中断方式 1：中断使能
//    PICTL |= 0x2;    //下降沿触发   
//    IEN1 |= 0x20;    //允许P0口中断; 
//    P0IFG = 0x00;    //初始化中断标志位
//    EA = 1;          //打开总中断
//    
    PA_RX_L;
    PA_TX_H;

//	     SXRESET_H;
}
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF  SPI设置初始化
// 输入参数 : 无
// 返回参数 : 无
// 说明     : 此初始化为430硬件SPI初始化
////////////////////////////////////////////////////////////////////////////////
void SX1276SPISetup(void)
{   
  SPI1_Init();              
}
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF   复位
// 输入参数 : 无
// 返回参数 : 无
// 说明     :
////////////////////////////////////////////////////////////////////////////////
void SX1276Reset()
{
  //HAL_SYSTEM_RESET();
}
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF  向寄存器地址连续发送数据
// 输入参数 : uint8 addr,寄存器地址 uint8 *buffer,发送数组指针 uint8 size指针长度
// 返回参数 : 无
// 说明     :
////////////////////////////////////////////////////////////////////////////////
void SX1276WriteBuffer( uint8 addr, uint8 *buffer, uint8 size )
{
  uint8 status,u8_ctr;
	CSN_Low;	    
  status = SPI1_communication(addr| 0x80);//发送寄存器值(位置),并读取状态值
  for(u8_ctr=0; u8_ctr<size; u8_ctr++)
	SPI1_communication(*buffer++); //写入数据
	CSN_High;	 
  status= status;          //返回读到的状态值
}
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF  向寄存器地址连续读数据
// 输入参数 : uint8 addr,寄存器地址 uint8 *buffer,存储数组指针 uint8 size要读的长度
// 返回参数 : 数据返回到*buffer中
// 说明     :
////////////////////////////////////////////////////////////////////////////////
void SX1276ReadBuffer( uint8 addr, uint8 *buffer, uint8 size )
{
	uint8 status,u8_ctr;
	CSN_Low;//CSN=0       
  status=SPI1_communication(addr & 0x7F);//发送寄存器地址,并读取状态值   	   
 	for(u8_ctr=0;u8_ctr<size;u8_ctr++)
	buffer[u8_ctr]=SPI1_communication(0XFF);//读出数据
	CSN_High; //CSN=1
  status= status;        //返回读到的状态值
}
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF  向寄存器地址写1字节数据
// 输入参数 : uint8 addr,寄存器地址 uint8 data数据
// 返回参数 : 
// 说明     :
////////////////////////////////////////////////////////////////////////////////
void SX1276Write( uint8 addr, uint8 data )
{
    SX1276WriteBuffer( addr, &data, 1 );
}
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF  向寄存器地址读1字节数据
// 输入参数 : uint8 addr,寄存器地址 uint8 *data读数据存储地址
// 返回参数 : 
// 说明     :
////////////////////////////////////////////////////////////////////////////////
void SX1276Read( uint8 addr, uint8 *data )
{
    SX1276ReadBuffer( addr, data, 1 );
}
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF  向FIFO写数据
// 输入参数 : uint8 *buffer,数组指针 uint8 size长度
// 返回参数 : 
// 说明     :
////////////////////////////////////////////////////////////////////////////////
void SX1276WriteFifo( uint8 *buffer, uint8 size )
{
    SX1276WriteBuffer( 0, buffer, size );
}
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF  向FIFO读数据
// 输入参数 : uint8 *buffer,数组指针 uint8 size长度
// 返回参数 : uint8 *buffer 存储读取内容
// 说明     :
////////////////////////////////////////////////////////////////////////////////
void SX1276ReadFifo( uint8 *buffer, uint8 size )
{
    SX1276ReadBuffer( 0, buffer, size );
}
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF  TX/RX的PA切换
// 输入参数 :  bool txEnable  切换逻辑
// 返回参数 : 无
// 说明     :真：作为TX。假：作为RX   为硬件两个PA控制IO口
////////////////////////////////////////////////////////////////////////////////
void SX1276WriteRxTx( uint8 txEnable )
{
    if( txEnable != 0 )       //如果为真，为TX
    {
        PA_RX_L;        //PA_RX为0
        PA_TX_H;        //PA_TX为1
    }
    else  //为假，为RX
    {
        PA_RX_H;        //PA_RX为1
        PA_TX_L;         //PA_TX为0
    }
}
////////////////////////////////////////////////////////////////////////////////
/*****************************************/
/* 函数功能：SPI1通信                    */
/* 说明：    发送一个字节，接收一个字节  */
/*****************************************/
uint8 SPI1_communication(uint8 TxData)
{		
  uint8 temp;
  
  // Write byte to USART1 buffer (transmit data).
  U1DBUF = TxData;

  // Check if byte is transmitted.
  while(!(U1CSR & 0x02));

  // Clear transmit byte status.
  U1CSR &= ~0x02;
  
  temp = U1DBUF;
  
  return temp;
}


/*****************************************/
/* 函数功能：SPI0初始化                  */
/* 说明：    没有用SSEL0                 */
/*****************************************/
void SPI1_Init(void)
{
   // SPI Master Mode
	PERCFG |= 0x02;                         // map USART1 to its alternative 2 location. P1_4: SSN, P1_5: SCK, P1_6: MOSI, P1_7: MISO
	P1SEL |= 0xE0;                          // P1_5, P1_6, and P1_7 are peripherals
	P1SEL &= ~0x10;                         // P1_4 is GPIO (SSN)
	P1DIR |= 0x10;                          // SSN is set as output
	
	U1BAUD = 0x00; U1GCR |= 0x11;           // Set baud rate to max (system clock frequency / 8)
	U1CSR &= ~0xA0;                         // SPI Master Mode
	//U1CSR &= ~0x80; U1CSR |= 0x20;        // SPI Slave Mode
	U1GCR &= ~0xC0; U1GCR |= 0x20;          // MSB
}
