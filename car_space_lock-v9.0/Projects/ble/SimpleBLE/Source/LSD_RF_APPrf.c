#include "sx1276-LoRa.h"
#include "SX1276_LPC1114.h"
#include "LSD_RF_SX1276.h"
#include "LSD_RF_APPrf.h"
#include "comdef.h"
#include "npi.h"
#include "app_process.h"

uint8 start_sleep_timerout_flag = 0;
extern uint8 start_timerout_flag = 0;

/*
  注意，本文件的函数实现了从RF的驱动与MCU的兼容

*/
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF发送数据包
// 输入参数 : uint8*data：发送数据指针
// 返回参数 : 无
// 说明     : 数据发送完成后DIO0从低电平变成高电平，每次调用此函数，会自动先将DIO0变为低，等待高电平
////////////////////////////////////////////////////////////////////////////////
void LSD_RF_SendPacket(uint8*cbuf)
{
  uint32 j=0xffff;	
  
    P1IEN &= ~(1<<3);                                   //禁止DIO0中断
    SX1276_TxPacket(cbuf);                              //发送数据
    while( P1_3 == 0 )
    {
       j--;
       if( j==0 )
       {
          break;
       }
    }
    P1IFG &= ~(1<<3);                                   //清除中断标志位
/*
    unsigned long  j=16777215;                          //超时用，用户需要根据实际情况来调整
	
    P1IEN &= ~(1<<3);                                   //禁止DIO0中断
    SX1276_TxPacket(cbuf);                              //发送数据
    while( P1_3 != 1 )
    {
		  j--;                                  //等待GDIO0电平为高
    }
    P1IFG &= ~(1<<3);                                   //清除中断标志位
*/}
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF进入接收状态
// 输入参数 : 
// 返回参数 : 无
// 说明     : 接收数据完成后DIO0从低电平变成高电平，
////////////////////////////////////////////////////////////////////////////////
void LSD_RF_RXmode(void)
{
    Rx_mode();                      //RF接收机切换到RX模式
    P1IEN |= (1<<3);                      //ENABLE DIO0中断
}
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF进入接收状态
// 输入参数 : 
// 返回参数 : 无
// 说明     : 接收数据完成后DIO0从低电平变成高电平，
////////////////////////////////////////////////////////////////////////////////
void LSD_RF_RXmode1(void)
{
    //Rx_mode1();                      //RF接收机切换到RX模式
    P1IEN &= ~(1<<3);                      //ENABLE DIO0中断
}

////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF进入Sleep状态
// 输入参数 : 无
// 返回参数 : 无
// 说明     : 无
////////////////////////////////////////////////////////////////////////////////
void LSD_RF_Sleepmode(void)
{
    SX1276Write( REG_LR_OPMODE, 0x80|RFLR_OPMODE_STANDBY );     
    SX1276Write(REG_LR_IRQFLAGS,0xff);                           
    SX1276Write( REG_LR_OPMODE, 0x80|RFLR_OPMODE_SLEEP );       
    PA_RX_L;           //PA_RX 初始化输出为0
    PA_TX_L;           //PA_TX 初始化输出为0   目的是降低待机功耗
}
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF  CAD初始化
// 输入参数 : 无
// 返回参数 : 无
// 说明     : DIO1--CADDetected    DIO3---CADDone
////////////////////////////////////////////////////////////////////////////////
void LSD_RF_CADinit(void)
{
    SX1276Write( REG_LR_OPMODE, 0x80|RFLR_OPMODE_STANDBY );  
    SX1276Write( REG_LR_PREAMBLEMSB,0xf0);
    SX1276Write( REG_LR_PREAMBLELSB,0xff);
    SX1276Write( REG_LR_IRQFLAGSMASK,\
	~(RFLR_IRQFLAGS_CADDONE|RFLR_IRQFLAGS_CADDETECTED));
    //
    SX1276Write( REG_LR_DIOMAPPING1,\
	  RFLR_DIOMAPPING1_DIO3_00 | RFLR_DIOMAPPING1_DIO1_10);
    SX1276WriteRxTx(false);     //set RF switch to RX path
    SX1276Write(REG_LR_IRQFLAGS,0xff);      
}
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF启动CAD，采样信道情况一次
// 输入参数 : 无
// 返回参数 : 无
// 说明     :   采样时间约为(2^SF+32)/BW
////////////////////////////////////////////////////////////////////////////////
void LSD_RF_CAD_Sample(void)
{
    SX1276WriteRxTx(false);     //set RF switch to RX path
    SX1276Write( REG_LR_OPMODE, 0x80|RFLR_OPMODE_STANDBY );  
    SX1276Write( REG_LR_OPMODE, 0x80|RFLR_OPMODE_CAD ); 
}
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : WOR初始化
// 输入参数 : 无
// 返回参数 : 无
// 说明     :   DIO1 :唤醒中断   DIO3：CAD超时中断（也可以认为是接收检测中断）
////////////////////////////////////////////////////////////////////////////////
void LSD_RF_WORInit(void)
{
  LSD_RF_CADinit();        //CAD功能初始化
  //CADDone使能
//  SX_DIO3_DIR=0;                       //做输入
//  DIO3_IFG = 0;                        //清除DIO0中断标志位
//  DIO3_IES = 0;                        //设置DIO0上升沿触发方式
//  DIO3_IE = 1;                         //使能DIO0中断
//  //CADDetected使能
//  SX_DIO1_DIR=0;                       //做输入
//  DIO1_IFG = 0;                        //清除DIO0中断标志位
//  DIO1_IES = 0;                        //设置DIO0上升沿触发方式
//  DIO1_IE = 1;                         //使能DIO0中断
  
  P1IEN &= ~(1<<3);                     //禁止DIO0中断
}
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : 执行WOR操作
// 输入参数 : uint8 cclen  0：进入睡眠。1：进入CAD检测模式
// 返回参数 : 无
// 说明     :   
////////////////////////////////////////////////////////////////////////////////
void LSD_RF_WOR_Execute(uint8 cclen)
{
  switch(cclen)
  {
    case 0:   //启动睡眠
      LSD_RF_Sleepmode();      //进入睡眠模式
      ON_Sleep_Timerout();     //启动睡眠超时定时器
      break;
    case 1:   //进入CAD检测模式
      OFF_Sleep_Timerout();    //关闭睡眠超时定时器
      LSD_RF_CAD_Sample();     //启动CAD一次 
      break;
    default: 
      break;
  }
}
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : WOR到RX
// 输入参数 : 无
// 返回参数 : 无
// 说明     : 退出WOR，进入RX模式，前导preamble仍然采用最大设置值。
////////////////////////////////////////////////////////////////////////////////
void LSD_RF_WOR_Exit()
{
    OFF_Sleep_Timerout();
    SX1276Write( REG_LR_OPMODE, 0x80|RFLR_OPMODE_STANDBY );      
    SX1276Write( REG_LR_PAYLOADLENGTH,G_LoRaConfig.PayloadLength);                    
    SX1276Write( REG_LR_IRQFLAGSMASK, ~(RFLR_IRQFLAGS_RXDONE)); 
    SX1276Write( REG_LR_DIOMAPPING1, RFLR_DIOMAPPING1_DIO0_00 );
    SX1276WriteRxTx(false);                                     //set RF switch to RX path
    SX1276Write( REG_LR_FIFOADDRPTR,0x00);                      
    SX1276Write(REG_LR_IRQFLAGS,0xff);                          
    SX1276Write( REG_LR_OPMODE, 0x80|RFLR_OPMODE_RECEIVER );   
    P1IEN |= (1<<3);                      //ENABLE DIO0中断
    
//    DIO1_IE = 0;                         //禁止DIO1
//    DIO3_IE = 0;                         //禁止DIO3
}



////////G_LoRaConfig.PayloadLength = 2; 
////////        uint16_t Preamble_Length = (uint16_t)(2200/G_TsXms+0.5);//唤醒前导长度4.2s
////////        LSD_RF_Awake(WakeAddr,Preamble_Length);


////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF发送唤醒包
// 输入参数 : uint8*data：发送数据指针,前导长度
// 返回参数 : 无
// 说明     : 
////////////////////////////////////////////////////////////////////////////////
void LSD_RF_Awake(uint8*cbuf,uint16 Preamble_Length)
{
	  P1IEN &= ~(1<<3);             //禁止DIO0中断
    
    SX1276Write( REG_LR_OPMODE, 0x80|RFLR_OPMODE_STANDBY );      
    SX1276Write( REG_LR_PAYLOADLENGTH,G_LoRaConfig.PayloadLength);                    
    SX1276WriteRxTx(true);                                      
    SX1276Write( REG_LR_FIFOADDRPTR,0x80);                      
    SX1276WriteBuffer(REG_LR_FIFO,cbuf,G_LoRaConfig.PayloadLength);                   
    SX1276Write(REG_LR_IRQFLAGS,0xff);                          
    SX1276Write( REG_LR_IRQFLAGSMASK, ~(RFLR_IRQFLAGS_TXDONE));  
    SX1276Write( REG_LR_DIOMAPPING1, RFLR_DIOMAPPING1_DIO0_01 );
    SX1276Write( REG_LR_PREAMBLEMSB,(uint8)(Preamble_Length>>8));//set preamble length
    SX1276Write( REG_LR_PREAMBLELSB,(uint8)Preamble_Length);//set preamble length
    SX1276Write( REG_LR_OPMODE, 0x80|RFLR_OPMODE_TRANSMITTER );  
    while(P1_3 != 1);                               //等待GDIO0电平为高
 
    P1IFG &= ~(1<<3);                                             //清除中断标志位
    //发送完唤醒数据包后，将前导时间改回正常默认值。
    SX1276Write( REG_LR_OPMODE, 0x80|RFLR_OPMODE_STANDBY );     
    SX1276Write( REG_LR_PREAMBLEMSB,0);//set preamble length
    SX1276Write( REG_LR_PREAMBLELSB,10);//set preamble length    
}

//for test
void SX1278_test(void)
{
	if(SX127x_init( frefunction1 )!=NORMAL)	
	{
		NPI_PrintString("SX1278 init failed\r\n");
	}		
  LSD_RF_RXmode();
}

void LSD_RF_Init(uint8 frefunction)
{
  if(SX127x_init( frefunction )!=NORMAL)	
  {
    NPI_PrintString("SX1278 init failed!\r\n");
  }		
  else
  {
     NPI_PrintString("SX1278 init success!\r\n");
  }
  
  LSD_RF_RXmode();  
  
//  LSD_RF_WORInit();                   //WOR初始化
//  LSD_RF_WOR_Execute(0);              //启动执行WOR
}



////////////////////////////////////////////////////////////////////////////////
// 功能描述 : 开启睡眠超时定时器
// 输入参数 : 无
// 返回参数 : 无
// 说明     : 
////////////////////////////////////////////////////////////////////////////////
void ON_Sleep_Timerout(void)
{    
  start_sleep_timerout_flag = 1;
//    TA0R =0;           //清除定时器计数器
//    TA0CTL |= TASSEL_1 | MC_1;    //开启超时定时器        
}
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : 关闭睡眠超时定时器
// 输入参数 : 无
// 返回参数 : 无
// 说明     : 
////////////////////////////////////////////////////////////////////////////////
void OFF_Sleep_Timerout(void)
{    
  start_sleep_timerout_flag = 0;
////    TA0R =0;           //清除定时器计数器
//    TA0CTL = TASSEL_1 | MC_0;    //关闭定时器       
}

////////////////////////////////////////////////////////////////////////////////
// 功能描述 : 开启超时定时器
// 输入参数 : 无
// 返回参数 : 无
// 说明     : 
////////////////////////////////////////////////////////////////////////////////
void ON_Timerout(void)
{          
  start_timerout_flag = 1;
//    TA1R =0;           //清除定时器计数器
//    TA1CTL |= TASSEL_1 | MC_1;    //开启超时定时器        
}
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : 关闭超时定时器
// 输入参数 : 无
// 返回参数 : 无
// 说明     : 
////////////////////////////////////////////////////////////////////////////////
void OFF_Timerout(void)
{   
  start_timerout_flag = 0;
////    TA0R =0;           //清除定时器计数器
//    TA1CTL = TASSEL_1 | MC_0;    //关闭定时器       
}


