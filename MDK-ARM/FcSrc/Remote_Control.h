#ifndef _REMOTE_CONTROL_H
#define _REMOTE_CONTROL_H

#include "SysConfig.h"

typedef struct
{
    u16 ch[10];
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
    s16 yaw;
    s16 throttle;
    s16 pos_x;
    s16 pos_y;
    s16 pos_z;
}__attribute__((__packed__))realtime_ctrl_data;

typedef union
{
    u8 byte[14];
    realtime_ctrl_data data;
}realtime_ctrl_un;

extern rc_channel_un Channel_of_rc;
extern realtime_ctrl_un ctrl_of_realtime;

#endif
