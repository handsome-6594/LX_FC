#include "Of_Radar_Fusion.h"
#include "Optical_Flow_Sensor.h"
#include "ANO_TO_H743_Data_Transmit.h"
#include "JetsonNano_Data_Transmit.h"
#include "Remote_Control.h"

#define EXT_SENSOR_INVALID_S16 ((s16)0x8000)

volatile u32 ext_flow_send33_cnt = 0;
volatile u32 ext_flow_send34_cnt = 0;
volatile u8 alt_soruce = 0;

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
    static u8 last_alt_source = 0xFF;
    static u8 last_flow_alt_update_cnt;
    static u8 last_radar_pos_update_cnt;
    u8 current_alt_source;
    u8 current_update_cnt;
    u32 distance_cm;

    if(Switch_sta_st.VRB == Switch_Low)
    {
        alt_soruce = 1;
    }
    else
    {
        alt_soruce = 0;
    }

    current_alt_source = alt_soruce;

    if(current_alt_source == 1)
    {
        current_update_cnt = optical_flow.alt_update_cnt;
        distance_cm = optical_flow.alt_cm;
    }
    else
    {
        current_update_cnt = radar_pos_update_cnt;
        distance_cm = (Pos16_of_Radar.pos_data.z_x100 > 0) ? (u32)Pos16_of_Radar.pos_data.z_x100 : 0;
    }

    if((last_alt_source == current_alt_source) &&
       (((current_alt_source == 1) && (last_flow_alt_update_cnt == current_update_cnt)) ||
        ((current_alt_source == 0) && (last_radar_pos_update_cnt == current_update_cnt))))
    {
        return;
    }

    last_alt_source = current_alt_source;
    if(current_alt_source == 1)
    {
        last_flow_alt_update_cnt = current_update_cnt;
    }
    else
    {
        last_radar_pos_update_cnt = current_update_cnt;
    }

    ex_sensor.distance_general.distance_data.direction = 0;
    ex_sensor.distance_general.distance_data.angle = 270;
    ex_sensor.distance_general.distance_data.distance = distance_cm;

    Data.fun[0x34].wait_to_send = 1;
    ext_flow_send34_cnt++;
}

void ExtSensor_UpdateFromOpticalFlow(float dT_s)
{
    (void)dT_s;

    GeneralVelocityFromOpticalFlow();
    GeneralDistanceFromOpticalFlow();
}
