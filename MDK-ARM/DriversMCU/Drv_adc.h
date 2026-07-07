#ifndef _DRV_ADC_H
#define _DRV_ADC_H

#include "SysConfig.h"

void DrvAdcInit(void);
double Drv_AdcGetBatVot(void);
double Drv_AdcGetBatCurr(void);
u16 Drv_AdcGetBatVolt100(void);
u16 Drv_AdcGetBatCurr100(void);


#endif

