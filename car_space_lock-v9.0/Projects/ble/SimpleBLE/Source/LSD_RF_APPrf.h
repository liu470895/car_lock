#ifndef LSD_RF_APPrf_H
#define LSD_RF_APPrf_H

#include "comdef.h"

void LSD_RF_SendPacket(uint8 *cbuf);
void LSD_RF_RXmode(void);
void LSD_RF_Sleepmode(void);
void LSD_RF_CADinit(void);
void LSD_RF_CAD_Sample(void);
void LSD_RF_WORInit(void);
void LSD_RF_WOR_Execute(uint8 cclen);
void LSD_RF_WOR_Exit(void);
void LSD_RF_Awake(uint8 *cbuf,uint16 Preamble_Length);
void LSD_RF_Init(uint8 frefunction);

void ON_Sleep_Timerout(void);
void OFF_Sleep_Timerout(void);
void ON_Timerout(void);
void OFF_Timerout(void);

#endif
