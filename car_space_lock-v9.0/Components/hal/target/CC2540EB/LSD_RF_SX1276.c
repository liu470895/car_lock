////////////////////////////////////////////////////////////////////////////////
// 版权:     利尔达科技集团股份有限公司
// 文件名:   
// 版本：   
// 工作环境:
// 作者:     
// 生成日期: 
// 功能:      
// 相关文件: 
// 修改日志：
//      见更新说明
////////////////////////////////////////////////////////////////////////////////
#include "sx1276-LoRa.h"
#include "sx1276_lpc1114.h"
#include "LSD_RF_SX1276.h"
#include "comdef.h"
//===================================定义变量===================================================
float G_BandWidthKHz;//本地计算Symbol周期使用
float G_TsXms;
S_LoRaConfig G_LoRaConfig;//在main中初始化

bool LoRaConfig_Check()
{
    if((G_LoRaConfig.LoRa_Freq<410000000)||(G_LoRaConfig.LoRa_Freq>1020000000))
        return false;
    if((G_LoRaConfig.LoRa_Freq>525000000)&&(G_LoRaConfig.LoRa_Freq<862000000))
        return false;
    G_LoRaConfig.BandWidth = (t_BandWidth)(G_LoRaConfig.BandWidth&0xF0);
    if(G_LoRaConfig.BandWidth>BW500KHZ)
        return false;
    //计算BandWidth
    switch(G_LoRaConfig.BandWidth){
    case BW500KHZ:G_BandWidthKHz = 500.0;break;
    case BW250KHZ:G_BandWidthKHz = 250.0;break;
    case BW125KHZ:G_BandWidthKHz = 125.0;break;
    case BW62_50KHZ:G_BandWidthKHz = 62.5;break;
    case BW41_66KHZ:G_BandWidthKHz = 41.66;break;
    case BW31_25KHZ:G_BandWidthKHz = 31.25;break;
    case BW20_83KHZ:G_BandWidthKHz = 20.83;break;
    case BW15_62KHZ:G_BandWidthKHz = 15.62;break;
    case BW10_41KHZ:G_BandWidthKHz = 10.41;break;
    case BW7_81KHZ:G_BandWidthKHz = 7.81;break;
    }
    G_LoRaConfig.SpreadingFactor = (t_SpreadingFactor)(G_LoRaConfig.SpreadingFactor&0xF0);
    if((G_LoRaConfig.SpreadingFactor>SF12)||(G_LoRaConfig.SpreadingFactor<SF06))
        return false;
    //计算LoRa码元周期，单位ms
    G_TsXms = (2<<((G_LoRaConfig.SpreadingFactor>>4)-1))/G_BandWidthKHz;
    
    G_LoRaConfig.CodingRate = (t_CodingRate)(G_LoRaConfig.CodingRate&0x0E);
    if((G_LoRaConfig.CodingRate>CR_4_8)||(G_LoRaConfig.CodingRate<CR_4_5))
        return false;
    if(G_LoRaConfig.PowerCfig>15)
        return false;
    if(G_LoRaConfig.PayloadLength>127)
        return false;
    return true;
}
//===================================子函数===================================================
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF初始化
// 输入参数 : tSX127xInitPara initPara  输入速率Init_LoRa_0_8K, Init_LoRa_4_8K , Init_LoRa_10k,
// 返回参数 : tSX127xError              错误枚举内容
// 说明     :  初始化时，信道初始化默认为0信道
////////////////////////////////////////////////////////////////////////////////

tSX127xError SX127x_init(uint8 freq)
{    
  static uint8 test = 0;
  
  if( freq == frefunction1)
  {
     G_LoRaConfig.LoRa_Freq = 460000000;   //中心频点500MHz
     G_LoRaConfig.BandWidth = BW250KHZ;    //BW = 250KHz
  }
  else
  {
     G_LoRaConfig.LoRa_Freq = 500000000;   //中心频点500MHz
     G_LoRaConfig.BandWidth = BW500KHZ;    //BW = 500KHz
  }
  G_LoRaConfig.SpreadingFactor = SF09;  //SF = 9
  G_LoRaConfig.CodingRate = CR_4_6;     //CR = 4/6
  G_LoRaConfig.PowerCfig = 15;          //19±dBm
  G_LoRaConfig.MaxPowerOn = true;       
  G_LoRaConfig.CRCON = true;            //CRC开启
  G_LoRaConfig.ExplicitHeaderOn = true; //Header开启
  G_LoRaConfig.PayloadLength = 30;      //数据包长度
    
  if(false==LoRaConfig_Check())   //如果输入参数错误
  {
      return PARAMETER_INVALID;  //报错输入
  }
  SX1276InitIo();                // PAIO口初始化           
  SX1276Reset();                 //复位RF
  SX1276SPISetup();              //SPI初始化
  
  //切换到LoRamode，standby状态
  SX1276Write( REG_LR_OPMODE, RFLR_OPMODE_SLEEP );            
  SX1276Write( REG_LR_OPMODE, 0x80|RFLR_OPMODE_SLEEP );       
  SX1276Write( REG_LR_OPMODE, 0x80|RFLR_OPMODE_STANDBY );
  
  /*------------------------------------------------
  SPI 验证                   */  
  
  SX1276Write( REG_LR_HOPPERIOD,0x91 );//选一个用不到的寄存器来做验证
  SX1276Read( REG_LR_HOPPERIOD,&test);
  if(test!=0x91)
  {
      return SPI_READCHECK_WRONG;           
  }
  //Frequency Configuration 
  LSD_RF_FreqSet();    //设置频率
  
  //PA Configuration  
  LSD_RF_PoutSet();
  SX1276Write( REG_LR_PARAMP,RFLR_PARAMP_0100_US);  
  //  ↑PA Ramp的时间，如果用户LDO不能快速输出大电流（泵能力），适当增加PA Ramp时间
  //  ↑如果Ramp时间过短超过了LDO的能力时，会出现进入TX后，系统电流为发射电流，但是RF信号不出现的现象
  SX1276Write( REG_LR_OCP,0x20|RFLR_OCP_TRIM_240_MA);//电流过载保护 Over Current Protection
  
  //PayloadLength 初始化             
  SX1276Write( REG_LR_PAYLOADLENGTH,G_LoRaConfig.PayloadLength); 
  //注意，无头模式（Implicit Header）时，必须提前规定好收发双方的PL
  
  //BW、CR、Header有无，初始化        
  SX1276Write( REG_LR_MODEMCONFIG1,\
      (((uint8)G_LoRaConfig.BandWidth)|((uint8)G_LoRaConfig.CodingRate))|(\
          (true==G_LoRaConfig.ExplicitHeaderOn)?0x00:0x01));
  
  //SF、CRC初始化                 
  SX1276Write( REG_LR_MODEMCONFIG2,\
      ((uint8)G_LoRaConfig.SpreadingFactor)|(\
          (true==G_LoRaConfig.CRCON)?0x04:0x00));
  if(SF06==G_LoRaConfig.SpreadingFactor){   //慎用SF = 6，需要的配置很特殊
      uint8 temp = 0;
      SX1276Read( 0x31,&temp);
      SX1276Write(  0x31,(temp& 0xF8)|0x05);
      SX1276Write(  0x37,0x0C); 
  }
  
  //低速率优化功能是否开启、AutoAGC默认开启    
  SX1276Write( REG_LR_MODEMCONFIG3,((G_TsXms>16.0)?\
    RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_ON:RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_OFF\
        )|RFLR_MODEMCONFIG3_AGCAUTO_ON);
  
  return NORMAL;
}
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF发送数据包
// 输入参数 : uint8*data：发送数据指针
// 返回参数 : 无
// 说明     : 设置为发送是， preamble改回默认值
////////////////////////////////////////////////////////////////////////////////
void SX1276_TxPacket(uint8*data)
{
    
    SX1276Write( REG_LR_OPMODE, 0x80|RFLR_OPMODE_STANDBY );      
    SX1276Write( REG_LR_PREAMBLEMSB,0);
    SX1276Write( REG_LR_PREAMBLELSB,10);
    SX1276Write( REG_LR_PAYLOADLENGTH,G_LoRaConfig.PayloadLength);                    
    SX1276WriteRxTx(true);                                      
    SX1276Write( REG_LR_FIFOADDRPTR,0x80);                      
    SX1276WriteBuffer(REG_LR_FIFO,data,G_LoRaConfig.PayloadLength);                   
    SX1276Write(REG_LR_IRQFLAGS,0xff);                          
    SX1276Write( REG_LR_IRQFLAGSMASK, ~(RFLR_IRQFLAGS_TXDONE)); 
    SX1276Write( REG_LR_DIOMAPPING1, RFLR_DIOMAPPING1_DIO0_01 );
    SX1276Write( REG_LR_OPMODE, 0x80|RFLR_OPMODE_TRANSMITTER );  
}
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF进入接收状态
// 输入参数 : 
// 返回参数 : 无
// 说明     : 进入接收后preamble设置回默认值为
////////////////////////////////////////////////////////////////////////////////
void Rx_mode(void)
{
    SX1276Write( REG_LR_OPMODE, 0x80|RFLR_OPMODE_STANDBY );      
    SX1276Write( REG_LR_PREAMBLEMSB,0);
    SX1276Write( REG_LR_PREAMBLELSB,10);
    SX1276Write( REG_LR_PAYLOADLENGTH,G_LoRaConfig.PayloadLength);
    SX1276Write( REG_LR_IRQFLAGSMASK, ~(RFLR_IRQFLAGS_RXDONE)); 
    SX1276Write( REG_LR_DIOMAPPING1, RFLR_DIOMAPPING1_DIO0_00 );
    SX1276WriteRxTx(false);                                     
    SX1276Write( REG_LR_FIFOADDRPTR,0x00);                      
    SX1276Write(REG_LR_IRQFLAGS,0xff);                          
    SX1276Write( REG_LR_OPMODE, 0x80|RFLR_OPMODE_RECEIVER );       
}
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF进入接收状态
// 输入参数 : 
// 返回参数 : 无
// 说明     : 进入接收后preamble设置回默认值为
////////////////////////////////////////////////////////////////////////////////
void Rx_mode1(void)
{
    SX1276Write( REG_LR_OPMODE, 0x80|RFLR_OPMODE_STANDBY );      
    SX1276Write( REG_LR_PREAMBLEMSB,0);
    SX1276Write( REG_LR_PREAMBLELSB,10);
    SX1276Write( REG_LR_PAYLOADLENGTH,G_LoRaConfig.PayloadLength);
    SX1276Write( REG_LR_IRQFLAGSMASK, ~(RFLR_IRQFLAGS_RXDONE)); 
    SX1276Write( REG_LR_DIOMAPPING1, RFLR_DIOMAPPING1_DIO0_00 );
    SX1276WriteRxTx(false);                                     
    SX1276Write( REG_LR_FIFOADDRPTR,0x00);                      
    SX1276Write(REG_LR_IRQFLAGS,0xff);                          
    //SX1276Write( REG_LR_OPMODE, 0x80|RFLR_OPMODE_RECEIVER );       
}

////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF接收数据包
// 输入参数 : uint8*cbuf接收数组指针
// 返回参数 : 无
// 说明     : 
////////////////////////////////////////////////////////////////////////////////
void LSD_RF_RxPacket(uint8*cbuf)
{
    if(true==G_LoRaConfig.ExplicitHeaderOn)
    {
        //有头那么从寄存器中读，否则按照G_LoRaConfig设置长度读取FIFO
        SX1276Read(REG_LR_NBRXBYTES,&G_LoRaConfig.PayloadLength);
        SX1276Write( REG_LR_FIFOADDRPTR,0x00);
    }
    SX1276ReadFifo(cbuf,G_LoRaConfig.PayloadLength);              
    SX1276Write(REG_LR_IRQFLAGS,0xff);        
}
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF进入SLEEP状态
// 输入参数 : 无
// 返回参数 : 无
// 说明     : 
////////////////////////////////////////////////////////////////////////////////
void LSD_RF_SleepMode(void)
{
    SX1276Write( REG_LR_OPMODE, 0x80|RFLR_OPMODE_STANDBY );      
    SX1276Write( REG_LR_OPMODE, 0x80|RFLR_OPMODE_SLEEP );     
}
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF进入standby状态
// 输入参数 : 无
// 返回参数 : 无
// 说明     : 
////////////////////////////////////////////////////////////////////////////////
void LSD_RF_StandbyMode(void)
{
    SX1276Write( REG_LR_OPMODE, 0x80|RFLR_OPMODE_STANDBY );       
}
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF配置频率
// 输入参数 : 无
// 返回参数 : 无
// 说明     : 
////////////////////////////////////////////////////////////////////////////////
tSX127xError LSD_RF_FreqSet()
{   
    uint32 freq_reg;
    uint8 test_FRFMSB = 0,test_FRFMID=0,test_FRFLSB=0;
    
    if((G_LoRaConfig.LoRa_Freq<410000000)||(G_LoRaConfig.LoRa_Freq>1020000000))
    {
        return PARAMETER_INVALID;
    }
    if((G_LoRaConfig.LoRa_Freq>525000000)&&(G_LoRaConfig.LoRa_Freq<862000000))
    {
        return PARAMETER_INVALID;
    }
    freq_reg = (uint32)(G_LoRaConfig.LoRa_Freq/FREQ_STEP);
    
    LSD_RF_StandbyMode();
    // FREQ = 474.6MHz
    SX1276Write( REG_LR_FRFMSB, (uint8)(freq_reg>>16));//Carrier Freq 470M
    SX1276Write( REG_LR_FRFMID, (uint8)(freq_reg>>8) );
    SX1276Write( REG_LR_FRFLSB, (uint8)(freq_reg) );
    
    SX1276Read(REG_LR_FRFMSB,&test_FRFMSB);
    SX1276Read(REG_LR_FRFMID,&test_FRFMID);
    SX1276Read(REG_LR_FRFLSB,&test_FRFLSB);
    
    if(test_FRFMSB != (uint8)(freq_reg>>16))
        return SPI_READCHECK_WRONG;
    if(test_FRFMID != (uint8)(freq_reg>>8))
        return SPI_READCHECK_WRONG;
    if(test_FRFLSB != (uint8)(freq_reg))
        return SPI_READCHECK_WRONG;
    return NORMAL;
}
////////////////////////////////////////////////////////////////////////////////
// 功能描述 : RF配置功率
// 输入参数 : 无
// 返回参数 : 无
// 说明     : 
////////////////////////////////////////////////////////////////////////////////
tSX127xError LSD_RF_PoutSet()
{   
	uint8 test = 0;
    if(G_LoRaConfig.PowerCfig>15)
        return PARAMETER_INVALID;
    LSD_RF_StandbyMode();
    SX1276Write( REG_LR_PACONFIG, 0xf0|G_LoRaConfig.PowerCfig);
    
    SX1276Read(REG_LR_PACONFIG,&test);
    if((0xf0|G_LoRaConfig.PowerCfig)!=test)
        return SPI_READCHECK_WRONG;
    if(true==G_LoRaConfig.MaxPowerOn)
        SX1276Write( REG_LR_PADAC, 0x80|RFLR_PADAC_20DBM_ON );  
    else
        SX1276Write( REG_LR_PADAC, 0x80|RFLR_PADAC_20DBM_OFF );
    return NORMAL;
}
