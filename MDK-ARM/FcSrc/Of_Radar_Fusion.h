#ifndef _OF_RADAR_FUSION_H
#define _OF_RADAR_FUSION_H

#include "SysConfig.h"

extern volatile u32 ext_flow_send33_cnt;
extern volatile u32 ext_flow_send34_cnt;
extern volatile u8 alt_soruce;

typedef enum
{
    Radar_vel = 0, /* Radar + optical-flow Kalman fusion. */
    ano_of_vel     /* Anonymous optical-flow velocity only. */
}VelSensorSource_t;

extern volatile VelSensorSource_t vel_sen_sorce;

//0X30  GPS数据 
typedef struct 
{
    u8 FLX_STA;
    u8 S_NUM;
    s32 LNG_1e7;
    s32 LAT_1e7;
    s32 ALT_GPS;
    s16 N_SPE;
    s16 E_SPE;
    s16 D_SPE;
    u8 PDOP_001;
    u8 SACC_001;
    u8 VACC_001;
}__attribute__((__packed__))GPS_data;

typedef union
{
    u8 byte_data[23];
    GPS_data data_of_gps;
}GPS_un;

//h743发给凌霄的速度数据
typedef struct 
{
    vec3_s16 velocity_cmps;
}__attribute__((__packed__))general_vel;

typedef union 
{
    u8 byte[6];
    general_vel vel_data;

}general_vel_un;

//h743发给凌霄IMU的距离数据
typedef struct  
{
    u8 direction;
    u16 angle;
    u32 distance;
} __attribute__((__packed__))general_distance;

typedef union 
{
    u8 byte[7];
    general_distance distance_data;

}general_distance_un;


typedef struct  
{
    general_distance_un distance_general;
    general_vel_un vel_general;
    GPS_un gps;

}ex_sensor_data;

extern ex_sensor_data ex_sensor;

void ExtSensorFusion_Init(void);
void ExtSensor_UpdateFromOpticalFlow(float dT_s);

#endif

