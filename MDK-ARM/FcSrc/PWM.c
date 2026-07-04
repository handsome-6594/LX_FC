#include "PWM.h"
#include "gpio.h"
#include "tim.h"
#include <stddef.h>

#define DSHOT_THROTTLE_OFFSET 48
#define DSHOT_VALUE_MAX      1999
#define DSHOT_DMA_BIT_NUM    18
#define DSHOT_DMA_BUF_LEN    (DSHOT_DMA_BIT_NUM * 2)

#define ESC_BIT_0 299
#define ESC_BIT_1 599

static uint16_t PrepareDshotPacket(uint16_t value)
{
    uint16_t packet = (uint16_t)(value << 1);
    uint16_t csum_data = packet;
    uint16_t csum = 0;

    for(uint8_t i = 0; i < 3; i++)
    {
        csum ^= csum_data;
        csum_data >>= 4;
    }

    csum &= 0x0F;
    packet = (uint16_t)((packet << 4) | csum);

    return packet;
}

static void PwmWriteDigital(int16_t value, uint32_t *pwm_data)
{
    uint16_t packet;

    value += DSHOT_THROTTLE_OFFSET;

    if(value >= DSHOT_VALUE_MAX)
    {
        value = DSHOT_VALUE_MAX;
    }
    else if(value <= 0)
    {
        value = 0;
    }

    packet = PrepareDshotPacket((uint16_t)value);

    pwm_data[0] = 0;
    pwm_data[1]  = (packet & 0x8000) ? ESC_BIT_1 : ESC_BIT_0;
    pwm_data[2]  = (packet & 0x4000) ? ESC_BIT_1 : ESC_BIT_0;
    pwm_data[3]  = (packet & 0x2000) ? ESC_BIT_1 : ESC_BIT_0;
    pwm_data[4]  = (packet & 0x1000) ? ESC_BIT_1 : ESC_BIT_0;
    pwm_data[5]  = (packet & 0x0800) ? ESC_BIT_1 : ESC_BIT_0;
    pwm_data[6]  = (packet & 0x0400) ? ESC_BIT_1 : ESC_BIT_0;
    pwm_data[7]  = (packet & 0x0200) ? ESC_BIT_1 : ESC_BIT_0;
    pwm_data[8]  = (packet & 0x0100) ? ESC_BIT_1 : ESC_BIT_0;
    pwm_data[9]  = (packet & 0x0080) ? ESC_BIT_1 : ESC_BIT_0;
    pwm_data[10] = (packet & 0x0040) ? ESC_BIT_1 : ESC_BIT_0;
    pwm_data[11] = (packet & 0x0020) ? ESC_BIT_1 : ESC_BIT_0;
    pwm_data[12] = (packet & 0x0010) ? ESC_BIT_1 : ESC_BIT_0;
    pwm_data[13] = (packet & 0x0008) ? ESC_BIT_1 : ESC_BIT_0;
    pwm_data[14] = (packet & 0x0004) ? ESC_BIT_1 : ESC_BIT_0;
    pwm_data[15] = (packet & 0x0002) ? ESC_BIT_1 : ESC_BIT_0;
    pwm_data[16] = (packet & 0x0001) ? ESC_BIT_1 : ESC_BIT_0;
    pwm_data[17] = 0;
}

void DrvESCInit(void)
{
    int16_t pwm[MOTOR_NUM] = {0};

    for(uint16_t i = 0; i < 1000; i++)
    {
        HAL_Delay(1);
        DrvMotorPWMSet(pwm);
    }
}

void DrvMotorPWMSet(const int16_t pwm[MOTOR_NUM])
{
    static uint32_t m1_pwm_data[DSHOT_DMA_BUF_LEN] = {0};
    static uint32_t m2_pwm_data[DSHOT_DMA_BUF_LEN] = {0};
    static uint32_t m3_pwm_data[DSHOT_DMA_BUF_LEN] = {0};
    static uint32_t m4_pwm_data[DSHOT_DMA_BUF_LEN] = {0};

    if(pwm == NULL)
    {
        return;
    }

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

    PwmWriteDigital(pwm[0], m1_pwm_data);
    HAL_TIM_PWM_Start_DMA(&htim5, TIM_CHANNEL_3, m1_pwm_data, DSHOT_DMA_BUF_LEN);

    PwmWriteDigital(pwm[1], m2_pwm_data);
    HAL_TIM_PWM_Start_DMA(&htim5, TIM_CHANNEL_1, m2_pwm_data, DSHOT_DMA_BUF_LEN);

    PwmWriteDigital(pwm[2], m3_pwm_data);
    HAL_TIM_PWM_Start_DMA(&htim5, TIM_CHANNEL_2, m3_pwm_data, DSHOT_DMA_BUF_LEN);

    PwmWriteDigital(pwm[3], m4_pwm_data);
    HAL_TIM_PWM_Start_DMA(&htim5, TIM_CHANNEL_4, m4_pwm_data, DSHOT_DMA_BUF_LEN);
}



