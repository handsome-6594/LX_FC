#include "Of_Radar_Fusion.h"
#include "Optical_Flow_Sensor.h"
#include "ANO_TO_H743_Data_Transmit.h"
#include "JetsonNano_Data_Transmit.h"
#include "Remote_Control.h"
#include "Kalman_Filter.h"
#include "Freq_Detector.h"
#include "Point_Navigation.h"
#include <stdio.h>

#define EXT_SENSOR_INVALID_S16 ((s16)0x8000)

/* All velocity values in this module use cm/s. */
#define VELOCITY_FUSION_INITIAL_COVARIANCE  (100.0f)
#define VELOCITY_FUSION_PROCESS_NOISE       (25.0f)
#define VELOCITY_FUSION_RADAR_NOISE         (25.0f)
#define VELOCITY_FUSION_OPTICAL_FLOW_NOISE  (100.0f)
#define VELOCITY_FUSION_RADAR_LIMIT_CMPS    (300)
#define VELOCITY_FUSION_FLOW_MIN_QUALITY    (250U)

/* Set to 0 after commissioning to remove periodic serial debug output. */
#define VELOCITY_FUSION_DEBUG_ENABLE         (1U)
#define VELOCITY_FUSION_DEBUG_PERIOD_MS      (250U)

volatile u32 ext_flow_send33_cnt = 0;
volatile u32 ext_flow_send34_cnt = 0;
volatile u8 alt_soruce = 0;
volatile VelSensorSource_t vel_sen_sorce = Radar_vel;
volatile VelocityFusionDiagnostics_t velocity_fusion_diagnostics;
volatile u32 velocity_fusion_self_test_result;

ex_sensor_data ex_sensor;

static VelocityFusionKalman_t velocity_fusion_filter;
static u8 velocity_fusion_initialized;

static s16 VelocityFusion_LimitS16(float value)
{
    if(value > 32767.0f)
    {
        return 32767;
    }

    if(value < -32768.0f)
    {
        return (s16)-32768;
    }

    return (s16)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static s32 VelocityFusion_AbsS16(s16 value)
{
    return (value < 0) ? -(s32)value : (s32)value;
}

static void VelocityFusion_DebugPrint(void)
{
#if VELOCITY_FUSION_DEBUG_ENABLE
    static u32 last_print_ms;
    u32 now_ms = HAL_GetTick();

    if((now_ms - last_print_ms) < VELOCITY_FUSION_DEBUG_PERIOD_MS)
    {
        return;
    }
    last_print_ms = now_ms;

    printf("vel_fusion self=%lu swb=%u src=%u ctrl=%u "
           "r=%d,%d of=%d,%d fused=%d,%d out=%d,%d "
           "valid=%u,%u cnt=%lu,%lu,%lu\r\n",
           (unsigned long)velocity_fusion_self_test_result,
           (unsigned int)Switch_sta_st.SWB,
           (unsigned int)vel_sen_sorce,
           (unsigned int)point_navigation_enable,
           velocity_fusion_diagnostics.radar_velocity_x,
           velocity_fusion_diagnostics.radar_velocity_y,
           velocity_fusion_diagnostics.optical_flow_velocity_x,
           velocity_fusion_diagnostics.optical_flow_velocity_y,
           velocity_fusion_diagnostics.fused_velocity_x,
           velocity_fusion_diagnostics.fused_velocity_y,
           velocity_fusion_diagnostics.output_velocity_x,
           velocity_fusion_diagnostics.output_velocity_y,
           (unsigned int)velocity_fusion_diagnostics.radar_valid,
           (unsigned int)velocity_fusion_diagnostics.optical_flow_valid,
           (unsigned long)velocity_fusion_diagnostics.radar_accept_count,
           (unsigned long)velocity_fusion_diagnostics.optical_flow_accept_count,
           (unsigned long)velocity_fusion_diagnostics.publish_count);
#endif
}

static void GeneralVelocityFromRadarAndOpticalFlow(float dT_s)
{
    static u8 last_flow_update_cnt;
    u8 optical_flow_updated;
    u8 optical_flow_valid = 0;
    u8 radar_updated;
    u8 radar_valid = 0;
    s16 radar_velocity_x = 0;
    s16 radar_velocity_y = 0;

    if(velocity_fusion_initialized == 0U)
    {
        return;
    }

    /* SWB high: optical flow only; SWB middle/low: fused velocity. */
    vel_sen_sorce = (Switch_sta_st.SWB == Switch_High) ? ano_of_vel : Radar_vel;

    optical_flow_updated = (last_flow_update_cnt != optical_flow.flow_update_cnt) ? 1U : 0U;
    if(optical_flow_updated != 0U)
    {
        last_flow_update_cnt = optical_flow.flow_update_cnt;
        if(optical_flow.work_sta != 0U &&
           optical_flow.fusion_flow_sta != 0U &&
           optical_flow.flow_quality > VELOCITY_FUSION_FLOW_MIN_QUALITY)
        {
            optical_flow_valid = 1U;
        }
    }

    radar_updated = (update_Flag.Radar_Speed != 0U) ? 1U : 0U;
    if(radar_updated != 0U)
    {
        update_Flag.Radar_Speed = 0U;
        radar_velocity_x = Speed_of_Radar.speed_data.vx_x100;
        radar_velocity_y = Speed_of_Radar.speed_data.vy_x100;

        if(VelocityFusion_AbsS16(radar_velocity_x) < VELOCITY_FUSION_RADAR_LIMIT_CMPS &&
           VelocityFusion_AbsS16(radar_velocity_y) < VELOCITY_FUSION_RADAR_LIMIT_CMPS)
        {
            radar_valid = 1U;
        }
    }

    VelocityFusionKalman_Update(&velocity_fusion_filter,
                                (float)radar_velocity_x,
                                (float)radar_velocity_y,
                                radar_valid,
                                (float)optical_flow.fusion_flow_dx,
                                (float)optical_flow.fusion_flow_dy,
                                optical_flow_valid,
                                dT_s);

    velocity_fusion_diagnostics.radar_updated = radar_updated;
    velocity_fusion_diagnostics.radar_valid = radar_valid;
    velocity_fusion_diagnostics.optical_flow_updated = optical_flow_updated;
    velocity_fusion_diagnostics.optical_flow_valid = optical_flow_valid;
    velocity_fusion_diagnostics.radar_velocity_x = radar_velocity_x;
    velocity_fusion_diagnostics.radar_velocity_y = radar_velocity_y;
    velocity_fusion_diagnostics.optical_flow_velocity_x = optical_flow.fusion_flow_dx;
    velocity_fusion_diagnostics.optical_flow_velocity_y = optical_flow.fusion_flow_dy;
    velocity_fusion_diagnostics.fused_velocity_x =
        VelocityFusion_LimitS16(velocity_fusion_filter.x.velocity);
    velocity_fusion_diagnostics.fused_velocity_y =
        VelocityFusion_LimitS16(velocity_fusion_filter.y.velocity);
    if(radar_valid != 0U)
    {
        velocity_fusion_diagnostics.radar_accept_count++;
    }
    if(optical_flow_valid != 0U)
    {
        velocity_fusion_diagnostics.optical_flow_accept_count++;
    }

    if(vel_sen_sorce == ano_of_vel)
    {
        /* Radar may still maintain the filter internally, but must not affect
         * the published velocity in optical-flow-only mode. */
        if(optical_flow_valid == 0U)
        {
            return;
        }

        ex_sensor.vel_general.vel_data.velocity_cmps[0] = optical_flow.fusion_flow_dx;
        ex_sensor.vel_general.vel_data.velocity_cmps[1] = optical_flow.fusion_flow_dy;
    }
    else
    {
        /* Do not publish again when this cycle contains no new valid measurement. */
        if(radar_valid == 0U && optical_flow_valid == 0U)
        {
            return;
        }

        ex_sensor.vel_general.vel_data.velocity_cmps[0] =
            VelocityFusion_LimitS16(velocity_fusion_filter.x.velocity);
        ex_sensor.vel_general.vel_data.velocity_cmps[1] =
            VelocityFusion_LimitS16(velocity_fusion_filter.y.velocity);
    }
    ex_sensor.vel_general.vel_data.velocity_cmps[2] = EXT_SENSOR_INVALID_S16;
    velocity_fusion_diagnostics.output_velocity_x =
        ex_sensor.vel_general.vel_data.velocity_cmps[0];
    velocity_fusion_diagnostics.output_velocity_y =
        ex_sensor.vel_general.vel_data.velocity_cmps[1];
    velocity_fusion_diagnostics.publish_count++;
    Data.fun[0x33].wait_to_send = 1;
    ext_flow_send33_cnt++;
    FreqDetector_OnData(&JN_freq_detector[Data_stream_Vel_Fu]);
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

void ExtSensorFusion_Init(void)
{
    velocity_fusion_self_test_result = VelocityFusionKalman_SelfTest();

    VelocityFusionKalman_Init(&velocity_fusion_filter,
                              0.0f,
                              0.0f,
                              VELOCITY_FUSION_INITIAL_COVARIANCE,
                              VELOCITY_FUSION_PROCESS_NOISE,
                              VELOCITY_FUSION_RADAR_NOISE,
                              VELOCITY_FUSION_OPTICAL_FLOW_NOISE);
    velocity_fusion_initialized = 1U;
    vel_sen_sorce = Radar_vel;
    update_Flag.Radar_Speed = 0U;
}

void ExtSensor_UpdateFromOpticalFlow(float dT_s)
{
    GeneralVelocityFromRadarAndOpticalFlow(dT_s);
    GeneralDistanceFromOpticalFlow();
    VelocityFusion_DebugPrint();
}
