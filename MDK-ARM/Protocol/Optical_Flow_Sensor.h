#ifndef _OPTICAL_FLOW_SENSOR_H
#define _OPTICAL_FLOW_SENSOR_H

#include "SysConfig.h"

typedef struct
{
    u8 flow_update_cnt;
    u8 alt_update_cnt;

    u8 link_sta;
    u8 work_sta;

    u8 flow_quality;

    u8 raw_flow_sta;
    s8 raw_flow_dx;
    s8 raw_flow_dy;

    u8 fusion_flow_sta;
    s16 fusion_flow_dx;
    s16 fusion_flow_dy;

    u8 inertial_flow_sta;
    s16 inertial_flow_dx;
    s16 inertial_flow_dy;
    s16 inertial_flow_dx_fix;
    s16 inertial_flow_dy_fix;
    s16 integral_x;
    s16 integral_y;

    u32 alt_cm;

    float quaternion[4];

    s16 acc_x;
    s16 acc_y;
    s16 acc_z;
    s16 gyro_x;
    s16 gyro_y;
    s16 gyro_z;
}OpticalFlowData;

extern OpticalFlowData optical_flow;

void OpticalFlow_Init(void);
void OpticalFlow_CheckState(float dT_s);
void OpticalFlow_ReceiveByte(u8 data);
u8 OpticalFlow_IsFlowUpdated(void);
u8 OpticalFlow_IsAltUpdated(void);

#endif
