#ifndef _JETSONNANO_DATA_TRANSMIT_H
#define _JETSONNANO_DATA_TRANSMIT_H

#include "SysConfig.h"
#include "ANO_TO_H743_Data_Transmit.h"

typedef Cmd_data _cmd_st;
typedef check_ack _ck_st;
typedef param _par_st;

typedef struct 
{
    Data_Frame fun[FUN_NUM];

    u8 cmd_wait_ack;    // CMD命令等待校验返回值
    check_ack ack_of_check;
    check_ack checksum_ok;
    param param_data;
    Cmd_data data_of_cmd;

}JetsonNano_Data;

extern JetsonNano_Data JN_DT_st;


//0X01 雷达位置数据
typedef struct 
{
    s16 x_x100;
    s16 y_x100;
    s16 z_x100;
}__attribute__((__packed__))Radar_Pos_16;

typedef union 
{
    u8 byte_data[6];
    Radar_Pos_16 pos_data;
}Radar_Pos_16_un;

//0X02  雷达传给H743的速度数据
typedef struct
{
    s16 vx_x100;
    s16 vy_x100;
    s16 vz_x100;
}__attribute__((__packed__))Radar_Speed;

typedef union
{
    u8 byte_data[6];
    Radar_Speed speed_data;
}Radar_Speed_un;

//0X05  JetsonNano发送的速度控制量
typedef struct
{
    s16 vx_x100;
    s16 vy_x100;
    s16 vz_x100;
    s16 yaw_x100;
}__attribute__((__packed__))Radar_Cmd_Vel;

typedef union
{
    u8 byte_data[8];
    Radar_Cmd_Vel cmd_vel_data;
}Radar_Cmd_Vel_un;

//0X06 雷达四元数
typedef struct
{
    s16 qx_x10000;
    s16 qy_x10000;
    s16 qz_x10000;
    s16 qw_x10000;
}__attribute__((__packed__))x10000_Radar_qua;

typedef struct 
{
    float qx;
    float qy;
    float qz;
    float qw;
}Radar_qua;

typedef union 
{
    u8 byte_data[8];
    x10000_Radar_qua data;
}x10000_Radar_qua_un;

//0X03 摄像头传过来的数据，可能内容会变化
//因此共用体byte字节数跟随结构体变化
typedef struct 
{
    s16 x_distance;
    s16 y_distance;
    u8 state;
    u8 type;
    s16 id;
}__attribute__((__packed__))Camera_data;

typedef union 
{
    u8 byte_data[sizeof(Camera_data)];
    Camera_data data;
}Camera_data_un;

//这个是为了适配凌霄IMU与H743的协议而写的 0X32 雷达位置数据
//一般没什么用  作为预留接口
typedef struct  
{
    s32 POS_X_x100;
    s32 POS_Y_x100;
    s32 POS_Z_x100;
}__attribute__ ((__packed__))Radar_Pos;

typedef union 
{
    u8 byte_data[12];
    Radar_Pos pos_data;

}Radar_Pos_un;

typedef struct
{
    u8 Radar_Pos;
    u8 Radar_Speed;
    u8 Cam_pix;
    u8 Radar_Cmd_Vel;
    u8 Radar_PID_Cmd_Vel;
    u8 Camera_PID_Cmd_Vel;
    u8 Radar_qua;
}_update_Flag_st;


extern Radar_Pos_un Pos_of_Radar;
extern volatile Radar_Pos_16_un Pos16_of_Radar;
extern volatile Radar_Speed_un Speed_of_Radar;
extern volatile Radar_Cmd_Vel_un speed_cmd_un;
extern volatile u8 radar_pos_update_cnt;
extern volatile _update_Flag_st update_Flag;
extern Camera_data_un Camera_Pos_data;

void JetsonNano_To_H743_Data_Prepare(u8 data);
void JN_CMD_Send(u8 dest_addr, _cmd_st *cmd);
void JN_CK_Back(u8 dest_addr, _ck_st *ck);
void JN_PAR_Back(u8 dest_addr, _par_st *par);
void JN_Data_Transmit_Check(void);


#endif
