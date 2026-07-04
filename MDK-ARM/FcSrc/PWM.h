#ifndef _PWM_H
#define _PWM_H

#include "SysConfig.h"

#define MOTOR_NUM 8

void DrvESCInit(void);
void DrvMotorPWMSet(const int16_t pwm[MOTOR_NUM]);

#endif

