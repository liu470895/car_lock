#include "app_led_control.h"
#include "hal_led.h"

/********************************************
* @函数名：led_control
* @函数功能: 控制LED的亮灭
* @入口参数 :leds: 控制所选择的LED的亮灭 ，共5个LED 
* @          numBlinks: 控制LED闪烁的次数  
* @          percent: 控制LED亮灭占总控制周期的百分比
* @          period: 控制LED一次的时间周期
* @          led_status: 控制LED的状态，为1则亮，为0则灭
* @返回值:void
*********************************************/
void led_control(uint8 leds, uint8 numBlinks, uint8 percent, uint16 period, uint8 led_status)
{
#if (defined (BLINK_LEDS)) && (HAL_LED == TRUE)
  leds &= HAL_LED_ALL;
  if(led_status == 1)
    HalLedBlink( leds, numBlinks, percent, period);  
  else
    HalLedBlink( leds, numBlinks,0, period);
#endif
}
