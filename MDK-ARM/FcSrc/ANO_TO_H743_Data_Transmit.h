#ifndef ANO_TO_H743_DATA_TRANSMIT_H
#define ANO_TO_H743_DATA_TRANSMIT_H

#include "SysConfig.h"
#include "Drv_led.h"

#define FUN_NUM 256

typedef struct
{
    u8 Addr;
    u8 wait_to_send;
    u16 fre_ms;
    u16 count_mstime;
}Data_Frame;

typedef struct 
{
    Data_Frame fun[FUN_NUM];
    /* data */
}Data_Transmit;

typedef struct 
{
    s16 quat_w_10000;
    s16 quat_x_10000;
    s16 quat_y_10000;
    s16 quat_z_10000;
    u8 fusion_sta;
}attitude_quat;

typedef struct
{
    s16 speed_x;
    s16 speed_y;
    s16 speed_z;
}__attribute__((__packed__))raw_speed;

typedef union 
{
    u8 byte_data[6];
    raw_speed speed;

}raw_speed_union;

typedef struct
{
    u16 pwm_value1;
    u16 pwm_value2;
    u16 pwm_value3;
    u16 pwm_value4;
    u16 pwm_value5;
    u16 pwm_value6;
    u16 pwm_value7;
    u16 pwm_value8;
}__attribute__((__packed__))pwm_value;

typedef struct 
{
    s32 Altitude_fused;
    s32 Altitude_added;
    u8 Altitude_status;
    u8 byte_data[9];

}__attribute__((__packed__))altitude_option;

typedef union 
{
    u8 byte_data[9];
    altitude_option altitude_source;

}altitude_option_un;

extern attitude_quat LX_quat;
extern Data_Transmit Data;
extern pwm_value pwm_to_esc;
extern raw_speed_union current_speed;
extern altitude_option_un altitude_of_option;

void H743_Data_Receive(u8 data);
void Data_Init(void);


#endif
