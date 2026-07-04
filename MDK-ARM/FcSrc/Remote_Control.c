#include "Remote_Control.h"
#include "tim.h"
#include "usart.h"
#include <string.h>

#define SBUS_FRAME_LEN          25
#define SBUS_FRAME_HEAD         0x0F
#define SBUS_FRAME_END          0x00
#define SBUS_FRAME_LOST_FLAG    0x04
#define SBUS_FAILSAFE_FLAG      0x08

#define RC_SIGNAL_CHECK_MS      1000
#define RC_SIGNAL_MIN_FREQ      5
#define RC_SBUS_MID             1024
#define RC_SBUS_SCALE           0.644f
#define RC_PWM_MID              1500
#define RC_PWM_MIN              1000
#define RC_PWM_MAX              2000

#define RC_NO_CHECK             0

rc_channel_union Channel_of_rc;
real_time_ctrl_union ctrl_of_realtime;
_rc_input_st rc_in;

static u8 sbus_rx_byte;
static u8 sbus_frame[SBUS_FRAME_LEN];
static u8 sbus_cnt;

static void TimerInit(void);

static s16 RcLimit(s16 value, s16 min, s16 max)
{
    if(value < min)
    {
        return min;
    }
    if(value > max)
    {
        return max;
    }
    return value;
}

static s16 SbusToPwm(s16 value)
{
    s16 pwm = (s16)(RC_SBUS_SCALE * (float)(value - RC_SBUS_MID) + RC_PWM_MID);
    return RcLimit(pwm, RC_PWM_MIN, RC_PWM_MAX);
}

static u8 SbusFrameEndOk(u8 data)
{
    return (data == SBUS_FRAME_END ||
            data == 0x04 ||
            data == 0x14 ||
            data == 0x24 ||
            data == 0x34);
}

static void DrvRcSbusInit(void)
{
    HAL_UART_AbortReceive_IT(&huart8);
    HAL_UART_DeInit(&huart8);

    huart8.Instance = UART8;
    huart8.Init.BaudRate = 100000;
    huart8.Init.WordLength = UART_WORDLENGTH_9B;
    huart8.Init.StopBits = UART_STOPBITS_2;
    huart8.Init.Parity = UART_PARITY_EVEN;
    huart8.Init.Mode = UART_MODE_RX;
    huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart8.Init.OverSampling = UART_OVERSAMPLING_16;
    huart8.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart8.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT | UART_ADVFEATURE_SWAP_INIT;
    huart8.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
    huart8.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;

    if(HAL_UART_Init(&huart8) == HAL_OK)
    {
        HAL_UART_Receive_IT(&huart8, &sbus_rx_byte, 1);
    }
}

static void DrvRcPpmInit(void)
{
    TimerInit();
}

static void RcSignalCheck(float dT_s)
{
    static u8 mode_try_cnt;
    static u16 time_ms;

    time_ms += (u16)(dT_s * 1000.0f);

    if(time_ms > RC_SIGNAL_CHECK_MS)
    {
        time_ms = 0;
        rc_in.signal_fre = rc_in.signal_cnt_tmp;

        if(rc_in.signal_fre < RC_SIGNAL_MIN_FREQ)
        {
            rc_in.no_signal = 1;
        }
        else
        {
            rc_in.no_signal = 0;
            rc_in.sig_mode = rc_in.rc_in_mode_tmp;
        }

        if(rc_in.no_signal && rc_in.sig_mode == Recever_Init_Null)
        {
            mode_try_cnt++;
            if(mode_try_cnt & 0x01)
            {
                DrvRcSbusInit();
            }
            else
            {
                DrvRcPpmInit();
            }
        }

        rc_in.signal_cnt_tmp = 0;
    }
}

static void TimerInit(void)
{
    HAL_TIM_Base_Start_IT(&htim16);
    HAL_TIM_Base_Start_IT(&htim14);
}

void DrvRcInputInit(void)
{
    memset(&rc_in, 0, sizeof(rc_in));
    memset(&Channel_of_rc, 0, sizeof(Channel_of_rc));

    rc_in.sig_mode = Recever_Init_Null;
    rc_in.no_signal = 1;
    rc_in.fail_safe = 1;

    DrvRcSbusInit();
}

void DrvPpmGetOneCh(u16 data)
{
    static u8 ch_sta;

    if((data > 2500 && ch_sta > 3) || ch_sta >= RC_PPM_CH_NUM)
    {
        ch_sta = 0;
        rc_in.signal_cnt_tmp++;
        rc_in.rc_in_mode_tmp = Recever_PPM_Mode;
    }
    else if(data > 300 && data < 3000)
    {
        rc_in.ppm_ch[ch_sta] = (s16)data;
        ch_sta++;
    }
}

void Sbus_Decode(uint8_t *RX_Data, uint8_t Length)
{
    if(RX_Data == NULL || Length < SBUS_FRAME_LEN)
    {
        return;
    }

    if(RX_Data[0] != SBUS_FRAME_HEAD || !SbusFrameEndOk(RX_Data[24]))
    {
        return;
    }

    rc_in.sbus_ch[0]  = (s16)(((RX_Data[1]     | RX_Data[2] << 8)                       ) & 0x07FF);
    rc_in.sbus_ch[1]  = (s16)(((RX_Data[2] >> 3 | RX_Data[3] << 5)                       ) & 0x07FF);
    rc_in.sbus_ch[2]  = (s16)(((RX_Data[3] >> 6 | RX_Data[4] << 2 | RX_Data[5] << 10)    ) & 0x07FF);
    rc_in.sbus_ch[3]  = (s16)(((RX_Data[5] >> 1 | RX_Data[6] << 7)                       ) & 0x07FF);
    rc_in.sbus_ch[4]  = (s16)(((RX_Data[6] >> 4 | RX_Data[7] << 4)                       ) & 0x07FF);
    rc_in.sbus_ch[5]  = (s16)(((RX_Data[7] >> 7 | RX_Data[8] << 1 | RX_Data[9] << 9)     ) & 0x07FF);
    rc_in.sbus_ch[6]  = (s16)(((RX_Data[9] >> 2 | RX_Data[10] << 6)                      ) & 0x07FF);
    rc_in.sbus_ch[7]  = (s16)(((RX_Data[10] >> 5 | RX_Data[11] << 3)                     ) & 0x07FF);
    rc_in.sbus_ch[8]  = (s16)(((RX_Data[12]    | RX_Data[13] << 8)                       ) & 0x07FF);
    rc_in.sbus_ch[9]  = (s16)(((RX_Data[13] >> 3 | RX_Data[14] << 5)                     ) & 0x07FF);
    rc_in.sbus_ch[10] = (s16)(((RX_Data[14] >> 6 | RX_Data[15] << 2 | RX_Data[16] << 10) ) & 0x07FF);
    rc_in.sbus_ch[11] = (s16)(((RX_Data[16] >> 1 | RX_Data[17] << 7)                     ) & 0x07FF);
    rc_in.sbus_ch[12] = (s16)(((RX_Data[17] >> 4 | RX_Data[18] << 4)                     ) & 0x07FF);
    rc_in.sbus_ch[13] = (s16)(((RX_Data[18] >> 7 | RX_Data[19] << 1 | RX_Data[20] << 9)  ) & 0x07FF);
    rc_in.sbus_ch[14] = (s16)(((RX_Data[20] >> 2 | RX_Data[21] << 6)                     ) & 0x07FF);
    rc_in.sbus_ch[15] = (s16)(((RX_Data[21] >> 5 | RX_Data[22] << 3)                     ) & 0x07FF);
    rc_in.sbus_flag = RX_Data[23];

    if((rc_in.sbus_flag & (SBUS_FRAME_LOST_FLAG | SBUS_FAILSAFE_FLAG)) == 0)
    {
        rc_in.signal_cnt_tmp++;
        rc_in.rc_in_mode_tmp = Recever_SBUS_Mode;
    }
}

void DrvRcInputTask(float dT_s)
{
    u8 failsafe = 0;

    RcSignalCheck(dT_s);

    if(rc_in.no_signal == 0)
    {
        if(rc_in.sig_mode == Recever_PPM_Mode)
        {
            for(u8 i = 0; i < RC_CH_NUM; i++)
            {
                rc_in.rc_ch.channel_data.channel[i] = RcLimit(rc_in.ppm_ch[i], RC_PWM_MIN, RC_PWM_MAX);
            }
        }
        else if(rc_in.sig_mode == Recever_SBUS_Mode)
        {
            for(u8 i = 0; i < RC_CH_NUM; i++)
            {
                rc_in.rc_ch.channel_data.channel[i] = SbusToPwm(rc_in.sbus_ch[i]);
            }
        }

        if((rc_in.rc_ch.channel_data.channel[ch_5_aux1] > 1200 &&
            rc_in.rc_ch.channel_data.channel[ch_5_aux1] < 1400) ||
           (rc_in.rc_ch.channel_data.channel[ch_5_aux1] > 1600 &&
            rc_in.rc_ch.channel_data.channel[ch_5_aux1] < 1800))
        {
            failsafe = 1;
        }
    }
    else
    {
        failsafe = 1;
        memset(&rc_in.rc_ch, 0, sizeof(rc_in.rc_ch));
    }

#if (RC_NO_CHECK == 0)
    rc_in.fail_safe = failsafe;
#else
    if(rc_in.no_signal != 0 || failsafe != 0)
    {
        for(u8 i = 0; i < RC_CH_NUM; i++)
        {
            rc_in.rc_ch.channel_data.channel[i] = RC_PWM_MID;
        }
    }
    rc_in.fail_safe = 0;
#endif

    Channel_of_rc = rc_in.rc_ch;
}

void RemoteControl_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance != UART8)
    {
        return;
    }

    if(sbus_cnt == 0)
    {
        if(sbus_rx_byte == SBUS_FRAME_HEAD)
        {
            sbus_frame[sbus_cnt++] = sbus_rx_byte;
        }
    }
    else
    {
        sbus_frame[sbus_cnt++] = sbus_rx_byte;

        if(sbus_cnt >= SBUS_FRAME_LEN)
        {
            Sbus_Decode(sbus_frame, SBUS_FRAME_LEN);
            sbus_cnt = 0;
        }
    }

    HAL_UART_Receive_IT(&huart8, &sbus_rx_byte, 1);
}

void RemoteControl_UartErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance != UART8)
    {
        return;
    }

    sbus_cnt = 0;
    HAL_UART_AbortReceive_IT(&huart8);
    HAL_UART_Receive_IT(&huart8, &sbus_rx_byte, 1);
}
