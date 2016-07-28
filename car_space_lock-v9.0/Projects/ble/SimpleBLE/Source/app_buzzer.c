#include "app_buzzer.h"
#include "hal_board_cfg.h"

void buzzer_control(uint8 status)
{
  if(status == 1)
    HAL_BUZZER_ON();
  else if(status == 0)
    HAL_BUZZER_OFF();
}
