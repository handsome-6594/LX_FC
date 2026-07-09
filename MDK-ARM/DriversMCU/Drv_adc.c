#include "Drv_adc.h"
#include "adc.h"

#define ADC_SAMPLE_NUM 50
#define BAT_UP_R_KOHM 10.0
#define BAT_DW_R_KOHM 1.0
#define BAT_ADC_REF_MV 3300.0
#define CURR_ADC_REF_MV 3270.0
#define BAT_VOLT100_OFFSET 0.1
#define BAT_CURR100_SCALE (125.0 * 0.01149)

static uint16_t BatAdcValue[ADC_SAMPLE_NUM] = {0};
static uint16_t CurrAdcValue[ADC_SAMPLE_NUM] = {0};

//DMA持续采样50个点，取平均值
static double AdcAverage(const uint16_t *buf)
{
    double sum = 0.0;

    for(u8 i = 0; i < ADC_SAMPLE_NUM; i++)
    {
        sum += buf[i];
    }

    return sum / ADC_SAMPLE_NUM;
}

void DrvAdcInit(void)
{
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_ADC_Stop(&hadc1);
    HAL_ADC_Stop(&hadc2);

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_Delay(10);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_Delay(10);

    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)BatAdcValue, ADC_SAMPLE_NUM);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)CurrAdcValue, ADC_SAMPLE_NUM);
}

//电池电压换算
double Drv_AdcGetBatVot(void)
{
    double adc = AdcAverage(BatAdcValue);
    return adc / 65535.0 * BAT_ADC_REF_MV * (BAT_UP_R_KOHM + BAT_DW_R_KOHM) / BAT_DW_R_KOHM;
}

//电池电流换算
double Drv_AdcGetBatCurr(void)
{
    double adc = AdcAverage(CurrAdcValue);
    return adc / 65535.0 * CURR_ADC_REF_MV;
}

u16 Drv_AdcGetBatVolt100(void)
{
    double volt_100 = Drv_AdcGetBatVot() * 100.0 * 0.001 + BAT_VOLT100_OFFSET;

    if(volt_100 < 0.0)
    {
        return 0;
    }

    if(volt_100 > 65535.0)
    {
        return 65535;
    }

    return (u16)volt_100;
}

u16 Drv_AdcGetBatCurr100(void)
{
    double curr_100 = Drv_AdcGetBatCurr() * BAT_CURR100_SCALE;

    if(curr_100 < 0.0)
    {
        return 0;
    }

    if(curr_100 > 65535.0)
    {
        return 65535;
    }

    return (u16)curr_100;
}




