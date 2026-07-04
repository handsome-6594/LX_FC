#ifndef _REMOTE_CONTROL_H
#define _REMOTE_CONTROL_H

#include "SysConfig.h"

#define RC_CH_NUM        10
#define RC_PPM_CH_NUM    10
#define RC_SBUS_CH_NUM   16

typedef enum
{
    Recever_Init_Null = 0,
    Recever_PPM_Mode,
    Recever_SBUS_Mode
}Recever_Signal_Mode;

typedef enum
{
    ch_1_rol = 0,
    ch_2_pit,
    ch_3_thr,
    ch_4_yaw,
    ch_5_aux1,
    ch_6_aux2,
    ch_7_aux3,
    ch_8_aux4,
    ch_9_aux5,
    ch_10_aux6
}rc_channel_index;

// 0x40 remote control data
typedef struct
{
    s16 channel[RC_CH_NUM];
}__attribute__((__packed__))rc_channel;

typedef union
{
    u8 byte[20];
    rc_channel channel_data;
}rc_channel_union;

// 0x41 realtime control data
typedef struct
{
    s16 ctrl_rol;
    s16 ctrl_pit;
    s16 ctrl_thr;
    s16 ctrl_yaw_dps;
    s16 ctrl_spd_x;
    s16 ctrl_spd_y;
    s16 ctrl_spd_z;
}__attribute__((__packed__))real_time_ctrl;

typedef union
{
    u8 byte[14];
    real_time_ctrl ctrl_data;
}real_time_ctrl_union;

typedef struct
{
    Recever_Signal_Mode sig_mode;
    s16 ppm_ch[RC_PPM_CH_NUM];
    s16 sbus_ch[RC_SBUS_CH_NUM];
    u8 sbus_flag;
    u16 signal_fre;
    u8 no_signal;
    u8 fail_safe;
    rc_channel_union rc_ch;
    u16 signal_cnt_tmp;
    Recever_Signal_Mode rc_in_mode_tmp;
}_rc_input_st;

extern rc_channel_union Channel_of_rc;
extern real_time_ctrl_union ctrl_of_realtime;
extern _rc_input_st rc_in;

void DrvRcInputInit(void);
void DrvPpmGetOneCh(u16 data);
void Sbus_Decode(uint8_t *RX_Data, uint8_t Length);
void DrvRcInputTask(float dT_s);
void RemoteControl_UartRxCpltCallback(UART_HandleTypeDef *huart);
void RemoteControl_UartErrorCallback(UART_HandleTypeDef *huart);

#endif
