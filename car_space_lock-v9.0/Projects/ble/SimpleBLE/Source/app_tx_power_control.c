#include "app_tx_power_control.h"
#include "hci.h"

/********************************************
* @函数名：Tx_Power_Cmd_Control
* @函数功能: 控制发射的功率
* @入口参数 :tx_power_mode ,根据手机向蓝牙发射模块发送的特征值，实现发射功率的改变
* @          共有4种模式，从0-3模式，发射的功率依次增加,默认是模式3,即最大发射功率
* @返回值:void
*********************************************/
void tx_power_cmd_control(uint8 tx_power_mode)
{
    HCI_EXT_SetTxPowerCmd( tx_power_mode);    
}