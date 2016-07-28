#include "app_batter.h"
#include "hal_board_cfg.h"

void batter_control(uint8 status)
{
  if(status == 1)
    HAL_BATTER_ON();
  else if(status == 0)
    HAL_BATTER_OFF();
}