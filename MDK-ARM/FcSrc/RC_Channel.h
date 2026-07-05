#ifndef _RC_CHANNEL_H
#define _RC_CHANNEL_H

#include "SysConfig.h"

void ESC_Output(u8 unlocked);
void RC_Data_Task(float dT_s);
void Bat_Voltage_Data_Handle(void);
void Bat_Curr_Data_Handle(void);
u8 RC_MotorIsUnlocked(void);
void RC_MotorForceLock(void);

#endif
