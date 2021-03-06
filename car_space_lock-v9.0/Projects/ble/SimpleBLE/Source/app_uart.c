#include "hal_uart.h"
#include "app_uart.h"
#include "OSAL.h"
#include "npi.h"


/*
1, 思路:  当串口收到数据后，就会马上调用以下回调函数，在实际测试中发现，此回调
函数调用频繁， 如果你不执行NPI_ReadTransport函数进行读取， 那么这个回调函数就会
频繁地被执行，但是，你通过串口发送一段数据， 你本意是想处理这一完整一段的数据，所以，
我们在下面引入了时间的处理方法， 也即接收的数据够多或者超时，就读取一次数据， 
然后根据当前的状态决定执行，如果没有连接上，就把所有数据当做AT命令处理， 如果连接
上了，就把数据送到对端。  ---------------amomcu   2014.08.17
*/
void NpiSerialCallback( uint8 port, uint8 events )
{
    (void)port;//加个 (void)，是未了避免编译告警，明确告诉缓冲区不用理会这个变量

    if (events & (HAL_UART_RX_TIMEOUT | HAL_UART_RX_FULL))   //串口有数据
    {
        uint8 numBytes = 0;

        numBytes = NPI_RxBufLen();           //读出串口缓冲区有多少字节
        
        if(numBytes == 0)
        {
            return;
        }
        else
        {
            //申请缓冲区buffer
            uint8 *buffer = osal_mem_alloc(numBytes);
            if(buffer)
            {
                //读取读取串口缓冲区数据，释放串口数据   
                NPI_ReadTransport(buffer,numBytes);   

                //把收到的数据发送到串口-实现回环 
                NPI_WriteTransport(buffer, numBytes);  

                //释放申请的缓冲区
                osal_mem_free(buffer);
            }
        }
    }
}