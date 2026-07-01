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

//

//check的返回校验
typedef struct 
{
    u8 ID;
    u8 SC;
    u8 AC;
}check_ack;


typedef struct 
{
    Data_Frame fun[FUN_NUM];
    
    u8 cmd_wait_ack;    // cmd命令等待校验返回值
    
    check_ack ack_of_check;

}Data_Transmit;

//存储四元数的
typedef struct 
{
    s16 quat_w_10000;
    s16 quat_x_10000;
    s16 quat_y_10000;
    s16 quat_z_10000;
    u8 fusion_sta;
}attitude_quat;

//存储速度的
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

//存储pwm 
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

//选择速度来源
typedef struct 
{
    s32 Altitude_fused;
    s32 Altitude_added;
    u8 Altitude_status;

}__attribute__((__packed__))altitude_option;

typedef union 
{
    u8 byte_data[9];
    altitude_option altitude_source;

}altitude_option_un;

//存储陀螺仪和传感器数据
typedef struct 
{
    s16 acc_x;
    s16 acc_y;
    s16 acc_z;
    s16 gyr_x;
    s16 gyr_y;
    s16 gyr_z;
    u8 shock_sta;
}__attribute__((__packed__))ACC_GYRO_Data;

typedef union 
{
    u8 byte_data[13];
    ACC_GYRO_Data acc_gyro_data;
}Gyro_Sensor;



//////////////////////////////////////////////////////
extern attitude_quat LX_quat;
extern Data_Transmit Data;
extern pwm_value pwm_to_esc;
extern raw_speed_union current_speed;
extern altitude_option_un altitude_of_option;
extern Gyro_Sensor Gyro_acc_sense;

/////////////////////////////////////////////////////

void H743_Data_Receive(u8 data);
void Data_Init(void);


#endif
