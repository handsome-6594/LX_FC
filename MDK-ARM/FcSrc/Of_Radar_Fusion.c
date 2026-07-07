#include "Of_Radar_Fusion.h"
#include "Optical_Flow_Sensor.h"
#include "ANO_TO_H743_Data_Transmit.h"

#define EXT_SENSOR_INVALID_S16 ((s16)0x8000)

volatile u32 ext_flow_send33_cnt = 0;
volatile u32 ext_flow_send34_cnt = 0;

ex_sensor_data ex_sensor;

static void GeneralVelocityFromOpticalFlow(void)
{
    static u8 last_flow_update_cnt;

    if(last_flow_update_cnt == optical_flow.flow_update_cnt)
    {
        return;
    }

    last_flow_update_cnt = optical_flow.flow_update_cnt;

    ex_sensor.vel_general.vel_data.velocity_cmps[0] = optical_flow.fusion_flow_dx;
    ex_sensor.vel_general.vel_data.velocity_cmps[1] = optical_flow.fusion_flow_dy;
    ex_sensor.vel_general.vel_data.velocity_cmps[2] = EXT_SENSOR_INVALID_S16;
    Data.fun[0x33].wait_to_send = 1;
    ext_flow_send33_cnt++;
}

static void GeneralDistanceFromOpticalFlow(void)
{
    static u8 last_alt_update_cnt;

    if(last_alt_update_cnt == optical_flow.alt_update_cnt)
    {
        return;
    }

    last_alt_update_cnt = optical_flow.alt_update_cnt;

    ex_sensor.distance_general.distance_data.direction = 0;
    ex_sensor.distance_general.distance_data.angle = 270;
    ex_sensor.distance_general.distance_data.distance = optical_flow.alt_cm;

    Data.fun[0x34].wait_to_send = 1;
    ext_flow_send34_cnt++;
}

void ExtSensor_UpdateFromOpticalFlow(float dT_s)
{
    (void)dT_s;

    GeneralVelocityFromOpticalFlow();
    GeneralDistanceFromOpticalFlow();
}

