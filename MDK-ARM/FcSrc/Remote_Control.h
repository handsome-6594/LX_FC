#ifndef _REMOTE_CONTROL_H
#define _REMOTE_CONTROL_H

#include "SysConfig.h"

enum
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
    ch_10_aux6,
};

typedef enum
{
    Switch_Low = 0,
    Switch_Mid = 1,
    Switch_High = 2,
}SwitchState;

typedef struct
{
    SwitchState SWA;
    SwitchState SWB;
    SwitchState SWC;
    SwitchState SWD;
    SwitchState VRA;
    SwitchState VRB;
}SwitchStateSet;

typedef struct
{
    s16 ch[10];
}__attribute__((__packed__))rc_channel_data;

typedef union
{
    u8 byte[20];
    rc_channel_data data;
}rc_channel_un;

typedef struct
{
    s16 roll;
    s16 pitch;
    s16 throttle;
    s16 yaw_dps;
    s16 vel_x;
    s16 vel_y;
    s16 vel_z;
}__attribute__((__packed__))realtime_ctrl_data;

typedef union
{
    u8 byte[14];
    realtime_ctrl_data data;
}realtime_ctrl_un;

extern rc_channel_un Channel_of_rc;
extern realtime_ctrl_un ctrl_of_realtime;
extern realtime_ctrl_un rc_ctrl_cmd;
extern realtime_ctrl_un nav_ctrl_cmd;
extern realtime_ctrl_un failsafe_ctrl_cmd;
extern SwitchStateSet Switch_sta_st;
extern volatile u32 sbus_dma_byte_cnt;
extern volatile u32 sbus_frame_cnt;

void RemoteControl_InitDefault(void);
void DrvRcInputInit(void);
void DrvRcInputTask(float dt);
void RemoteControl_UartRxCpltCallback(UART_HandleTypeDef *huart);
void RemoteControl_UartIdleCallback(UART_HandleTypeDef *huart);
void RemoteControl_UartErrorCallback(UART_HandleTypeDef *huart);
u8 RemoteControl_IsSignalLost(void);

#endif
