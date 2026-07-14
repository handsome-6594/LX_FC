#include "Point_Navigation.h"
#include "Adaptive_PID.h"
#include "ANO_TO_H743_Data_Transmit.h"
#include "Freq_Detector.h"
#include "FC_State.h"
#include "JetsonNano_Data_Transmit.h"
#include "Remote_Control.h"
#include <math.h>
#include <stdio.h>

#define POINT_NAV_TEST_TARGET_X_X100   100
#define POINT_NAV_TEST_TARGET_Y_X100   0
#define POINT_NAV_TEST_TARGET_Z_X100   80
#define POINT_NAV_TEST_TARGET_YAW_DEG  0
#define POINT_NAV_SENSOR_TIMEOUT_MS    300
#define POINT_NAV_DEBUG_ENABLE         (1U)
#define POINT_NAV_DEBUG_PERIOD_MS      (200U)
#define POINT_NAV_GATE_DEBUG_PERIOD_MS (500U)

volatile _cmd_vel_sorce cmd_vel_sorce = Radar_Pid_vel;
volatile u8 point_navigation_enable = 0;
FC_TaskState_t point_navigation_state = {0};
FC_Stable_t point_navigation_stable = {
    .alt_ok_time = 20,
    .alt_error_threshold = 7,
    .center_ok_time = 20,
    .center_error_threshold = 10,
    .yaw_ok_time = 20,
    .yaw_error_threshold = 7,
};
PointNavigationTarget_t point_navigation_target = {0};

#if POINT_NAV_DEBUG_ENABLE
static PID_t point_nav_debug_pid[PID_NUM];
static u8 point_nav_debug_pid_inited;

static void PointNav_DebugResetPreviewPid(void)
{
    point_nav_debug_pid[PID_X] = loc_pid[PID_X];
    point_nav_debug_pid[PID_Y] = loc_pid[PID_Y];
    point_nav_debug_pid[PID_Z] = loc_pid[PID_Z];
    point_nav_debug_pid[PID_YAW] = loc_pid[PID_YAW];
    PID_Reset(&point_nav_debug_pid[PID_X]);
    PID_Reset(&point_nav_debug_pid[PID_Y]);
    PID_Reset(&point_nav_debug_pid[PID_Z]);
    PID_Reset(&point_nav_debug_pid[PID_YAW]);
    point_nav_debug_pid_inited = 1;
}
#endif

static s16 PointNav_LimitS16(float value, s16 min_value, s16 max_value)
{
    if(value > (float)max_value)
    {
        return max_value;
    }

    if(value < (float)min_value)
    {
        return min_value;
    }

    return (s16)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static s32 PointNav_AbsS32(s32 value)
{
    return (value < 0) ? -value : value;
}

static float PointNav_AngleErrorDeg(float target_deg, float measurement_deg)
{
    float error = target_deg - measurement_deg;

    while(error > 180.0f)
    {
        error -= 360.0f;
    }

    while(error < -180.0f)
    {
        error += 360.0f;
    }

    return error;
}

static float PointNav_GetRadarYawDeg(void)
{
    float qx = real_Radar_qua.qx;
    float qy = real_Radar_qua.qy;
    float qz = real_Radar_qua.qz;
    float qw = real_Radar_qua.qw;

    return -atan2f(2.0f * qx * qy + 2.0f * qz * qw,
                   -2.0f * qy * qy - 2.0f * qz * qz + 1.0f) * 57.2957795f;
}

static void PointNav_RadarVelocityToBody(s16 radar_vel_x, s16 radar_vel_y, s16 *body_vel_x, s16 *body_vel_y)
{
    float qx = -real_Radar_qua.qx;
    float qy = -real_Radar_qua.qy;
    float qz = -real_Radar_qua.qz;
    float qw = real_Radar_qua.qw;
    float vx = (float)radar_vel_x;
    float vy = (float)radar_vel_y;
    float vz = 0.0f;
    float body_x;
    float body_y;

    float r00 = 1.0f - 2.0f * qy * qy - 2.0f * qz * qz;
    float r01 = 2.0f * qx * qy - 2.0f * qw * qz;
    float r02 = 2.0f * qx * qz + 2.0f * qw * qy;
    float r10 = 2.0f * qx * qy + 2.0f * qw * qz;
    float r11 = 1.0f - 2.0f * qx * qx - 2.0f * qz * qz;
    float r12 = 2.0f * qy * qz - 2.0f * qw * qx;

    body_x = r00 * vx + r01 * vy + r02 * vz;
    body_y = r10 * vx + r11 * vy + r12 * vz;

    *body_vel_x = PointNav_LimitS16(body_x, -100, 100);
    *body_vel_y = PointNav_LimitS16(body_y, -100, 100);
}

static void PointNav_GetFcTaskState(s16 target_x, s16 target_y, s16 target_z, s16 target_yaw,
                                    FC_TaskState_t *state, FC_Stable_t stable)
{
    static u8 alt_ok;
    static u8 center_ok;
    static u8 yaw_ok;
    static s16 last_target_x;
    static s16 last_target_y;
    static s16 last_target_z;
    static s16 last_target_yaw;
    static u8 target_inited;
    s16 z_error_threshold;
    s16 xy_error_threshold;
    s16 yaw_error_threshold;
    float error_x;
    float error_y;
    float error_length;

    if(state == 0)
    {
        return;
    }

    if(target_inited == 0 ||
       last_target_x != target_x ||
       last_target_y != target_y ||
       last_target_z != target_z ||
       last_target_yaw != target_yaw)
    {
        alt_ok = 0;
        center_ok = 0;
        yaw_ok = 0;
        state->alt_stable = 0;
        state->center_stable = 0;
        state->yaw_stable = 0;
        last_target_x = target_x;
        last_target_y = target_y;
        last_target_z = target_z;
        last_target_yaw = target_yaw;
        target_inited = 1;
    }

    if(stable.alt_ok_time > 200) stable.alt_ok_time = 200;
    if(stable.center_ok_time > 200) stable.center_ok_time = 200;
    if(stable.yaw_ok_time > 200) stable.yaw_ok_time = 200;
    if(stable.alt_ok_time == 0) stable.alt_ok_time = 1;
    if(stable.center_ok_time == 0) stable.center_ok_time = 1;
    if(stable.yaw_ok_time == 0) stable.yaw_ok_time = 1;

    z_error_threshold = (stable.alt_error_threshold == 0) ? 7 : stable.alt_error_threshold;
    xy_error_threshold = (stable.center_error_threshold == 0) ? 10 : stable.center_error_threshold;
    yaw_error_threshold = (stable.yaw_error_threshold == 0) ? 7 : stable.yaw_error_threshold;

    if(PointNav_AbsS32((s32)Pos16_of_Radar.pos_data.z_x100 - (s32)target_z) > z_error_threshold)
    {
        alt_ok = 0;
        state->alt_stable = 0;
    }
    else if(alt_ok < stable.alt_ok_time)
    {
        alt_ok++;
    }
    else
    {
        state->alt_stable = 1;
    }

    error_x = (float)(Pos16_of_Radar.pos_data.x_x100 - target_x);
    error_y = (float)(Pos16_of_Radar.pos_data.y_x100 - target_y);
    error_length = sqrtf(error_x * error_x + error_y * error_y);

    if(error_length > (float)xy_error_threshold)
    {
        center_ok = 0;
        state->center_stable = 0;
    }
    else if(center_ok < stable.center_ok_time)
    {
        center_ok++;
    }
    else
    {
        state->center_stable = 1;
    }

    if(fabsf(PointNav_AngleErrorDeg((float)target_yaw, PointNav_GetRadarYawDeg())) > (float)yaw_error_threshold)
    {
        yaw_ok = 0;
        state->yaw_stable = 0;
    }
    else if(yaw_ok < stable.yaw_ok_time)
    {
        yaw_ok++;
    }
    else
    {
        state->yaw_stable = 1;
    }
}

static void PointNav_WriteRealtimeVelocity(s16 vel_x, s16 vel_y, s16 vel_z, s16 yaw_dps)
{
    ctrl_of_realtime.data.roll = 0;
    ctrl_of_realtime.data.pitch = 0;
    ctrl_of_realtime.data.throttle = 500;
    ctrl_of_realtime.data.vel_x = vel_x;
    ctrl_of_realtime.data.vel_y = vel_y;
    ctrl_of_realtime.data.vel_z = vel_z;
    ctrl_of_realtime.data.yaw_dps = yaw_dps;
    Data.fun[0x41].wait_to_send = 1;
}

static void PointNav_ClearRealtimeVelocity(void)
{
    PointNav_WriteRealtimeVelocity(0, 0, 0, 0);
}

static u8 PointNav_RadarDataHealthy(void)
{
    static u8 last_pos_update_cnt;
    static u8 last_qua_update_cnt;
    static u32 last_pos_update_ms;
    static u32 last_qua_update_ms;
    u32 now_ms = HAL_GetTick();

    if(last_pos_update_cnt != radar_pos_update_cnt)
    {
        last_pos_update_cnt = radar_pos_update_cnt;
        last_pos_update_ms = now_ms;
    }

    if(last_qua_update_cnt != radar_qua_update_cnt)
    {
        last_qua_update_cnt = radar_qua_update_cnt;
        last_qua_update_ms = now_ms;
    }

    if(radar_pos_update_cnt == 0 || radar_qua_update_cnt == 0)
    {
        return 0;
    }

    if((now_ms - last_pos_update_ms) > POINT_NAV_SENSOR_TIMEOUT_MS)
    {
        return 0;
    }

    if((now_ms - last_qua_update_ms) > POINT_NAV_SENSOR_TIMEOUT_MS)
    {
        return 0;
    }

    return 1;
}

static u8 PointNav_UpdateFromRadarPid(void)
{
    static u8 last_radar_pos_update_cnt;
#if POINT_NAV_DEBUG_ENABLE
    static u32 last_debug_print_ms;
#endif
    s16 vel_x;
    s16 vel_y;
    s16 vel_z;
    s16 yaw_dps;
    s16 body_vel_x;
    s16 body_vel_y;
    float yaw_deg;

    if(last_radar_pos_update_cnt == radar_pos_update_cnt)
    {
        return 0;
    }

    last_radar_pos_update_cnt = radar_pos_update_cnt;

    yaw_deg = PointNav_GetRadarYawDeg();
    vel_x = PointNav_LimitS16(PID_Update(&loc_pid[PID_X],
                                         (float)point_navigation_target.target_x,
                                         (float)Pos16_of_Radar.pos_data.x_x100),
                              -100, 100);
    vel_y = PointNav_LimitS16(PID_Update(&loc_pid[PID_Y],
                                         (float)point_navigation_target.target_y,
                                         (float)Pos16_of_Radar.pos_data.y_x100),
                              -100, 100);
    vel_z = PointNav_LimitS16(PID_Update(&loc_pid[PID_Z],
                                         (float)point_navigation_target.target_z,
                                         (float)Pos16_of_Radar.pos_data.z_x100),
                              -100, 100);
    yaw_dps = PointNav_LimitS16(PID_UpdateYaw(&loc_pid[PID_YAW],
                                              (float)point_navigation_target.target_yaw,
                                              yaw_deg),
                                -200, 200);

    PointNav_RadarVelocityToBody(vel_x, vel_y, &body_vel_x, &body_vel_y);

#if POINT_NAV_DEBUG_ENABLE
    if(HAL_GetTick() - last_debug_print_ms >= POINT_NAV_DEBUG_PERIOD_MS)
    {
        s16 yaw_x10 = PointNav_LimitS16(yaw_deg * 10.0f, -32768, 32767);
        s16 yaw_err_x10 = PointNav_LimitS16(PointNav_AngleErrorDeg((float)point_navigation_target.target_yaw,
                                                                   yaw_deg) * 10.0f,
                                            -32768,
                                            32767);

        last_debug_print_ms = HAL_GetTick();
        printf("pnav_run pos=%d,%d,%d tar=%d,%d,%d err=%d,%d,%d "
               "yaw_x10=%d yaw_err_x10=%d vr=%d,%d,%d body=%d,%d,%d "
               "stable=%u,%u,%u\r\n",
               Pos16_of_Radar.pos_data.x_x100,
               Pos16_of_Radar.pos_data.y_x100,
               Pos16_of_Radar.pos_data.z_x100,
               point_navigation_target.target_x,
               point_navigation_target.target_y,
               point_navigation_target.target_z,
               point_navigation_target.target_x - Pos16_of_Radar.pos_data.x_x100,
               point_navigation_target.target_y - Pos16_of_Radar.pos_data.y_x100,
               point_navigation_target.target_z - Pos16_of_Radar.pos_data.z_x100,
               yaw_x10,
               yaw_err_x10,
               vel_x,
               vel_y,
               vel_z,
               body_vel_x,
               body_vel_y,
               vel_z,
               point_navigation_state.alt_stable,
               point_navigation_state.center_stable,
               point_navigation_state.yaw_stable);
    }
#endif

    PointNav_WriteRealtimeVelocity(body_vel_x, body_vel_y, vel_z, yaw_dps);
    update_Flag.Radar_PID_Cmd_Vel = 1;
    FreqDetector_OnData(&JN_freq_detector[Data_stream_Radar_cmd_vel]);

    return 1;
}

static u8 PointNav_UpdateFromJnCmdVel(void)
{
    s16 yaw_dps;

    yaw_dps = (s16)(speed_cmd_un.cmd_vel_data.yaw_x100 / 100);
    PointNav_WriteRealtimeVelocity(speed_cmd_un.cmd_vel_data.vx_x100,
                                   speed_cmd_un.cmd_vel_data.vy_x100,
                                   speed_cmd_un.cmd_vel_data.vz_x100,
                                   yaw_dps);

    return 1;
}

void PointNavigation_Init(void)
{
    PID_Init();
#if POINT_NAV_DEBUG_ENABLE
    PointNav_DebugResetPreviewPid();
#endif
    point_navigation_enable = 0;
    cmd_vel_sorce = Radar_Pid_vel;
}

void PointNavigation_Start(void)
{
    PID_Reset(&loc_pid[PID_X]);
    PID_Reset(&loc_pid[PID_Y]);
    PID_Reset(&loc_pid[PID_Z]);
    PID_Reset(&loc_pid[PID_YAW]);
    point_navigation_enable = 1;
#if POINT_NAV_DEBUG_ENABLE
    PointNav_DebugResetPreviewPid();
#endif
}

void PointNavigation_Stop(void)
{
    point_navigation_enable = 0;
    update_Flag.Radar_PID_Cmd_Vel = 0;
    update_Flag.Camera_PID_Cmd_Vel = 0;
    PointNav_ClearRealtimeVelocity();
}

void PointNavigation_SetCmdVelSource(_cmd_vel_sorce source)
{
    cmd_vel_sorce = source;
}

void PointNavigation_SetStableCondition(FC_Stable_t stable)
{
    point_navigation_stable = stable;
}

void PointNavigation_SetTarget(s16 target_x, s16 target_y, s16 target_z, s16 target_yaw)
{
    point_navigation_target.target_x = target_x;
    point_navigation_target.target_y = target_y;
    point_navigation_target.target_z = target_z;
    point_navigation_target.target_yaw = target_yaw;
}

void PointNavigation_TestPointTask(void)
{
#if POINT_NAV_DEBUG_ENABLE
    static u32 last_gate_print_ms;
    static u32 last_preview_print_ms;
    static u8 last_preview_pos_update_cnt;
    static s16 preview_vel_x;
    static s16 preview_vel_y;
    static s16 preview_vel_z;
    static s16 preview_yaw_dps;
    static s16 preview_body_vel_x;
    static s16 preview_body_vel_y;
    static s16 preview_yaw_x10;
    static s16 preview_yaw_err_x10;
#endif
    u8 should_run = 0;
    u32 now_ms = HAL_GetTick();
    u8 radar_healthy = PointNav_RadarDataHealthy();

#if POINT_NAV_DEBUG_ENABLE
    if(now_ms - last_gate_print_ms >= POINT_NAV_GATE_DEBUG_PERIOD_MS)
    {
        last_gate_print_ms = now_ms;
        printf("pnav_gate unlock=%u swb=%u swc=%u swd=%u src=%u "
               "pos_cnt=%u qua_cnt=%u healthy=%u en=%u\r\n",
               state.is_unlocked,
               Switch_sta_st.SWB,
               Switch_sta_st.SWC,
               Switch_sta_st.SWD,
               cmd_vel_sorce,
               radar_pos_update_cnt,
               radar_qua_update_cnt,
               radar_healthy,
               point_navigation_enable);
    }

    if(point_navigation_enable == 0U && radar_healthy != 0U)
    {
        if(point_nav_debug_pid_inited == 0U)
        {
            PointNav_DebugResetPreviewPid();
        }

        if(last_preview_pos_update_cnt != radar_pos_update_cnt)
        {
            float yaw_deg;

            last_preview_pos_update_cnt = radar_pos_update_cnt;
            yaw_deg = PointNav_GetRadarYawDeg();
            preview_vel_x = PointNav_LimitS16(PID_Update(&point_nav_debug_pid[PID_X],
                                                         (float)POINT_NAV_TEST_TARGET_X_X100,
                                                         (float)Pos16_of_Radar.pos_data.x_x100),
                                              -100,
                                              100);
            preview_vel_y = PointNav_LimitS16(PID_Update(&point_nav_debug_pid[PID_Y],
                                                         (float)POINT_NAV_TEST_TARGET_Y_X100,
                                                         (float)Pos16_of_Radar.pos_data.y_x100),
                                              -100,
                                              100);
            preview_vel_z = PointNav_LimitS16(PID_Update(&point_nav_debug_pid[PID_Z],
                                                         (float)POINT_NAV_TEST_TARGET_Z_X100,
                                                         (float)Pos16_of_Radar.pos_data.z_x100),
                                              -100,
                                              100);
            preview_yaw_dps = PointNav_LimitS16(PID_UpdateYaw(&point_nav_debug_pid[PID_YAW],
                                                              (float)POINT_NAV_TEST_TARGET_YAW_DEG,
                                                              yaw_deg),
                                                -200,
                                                200);
            PointNav_RadarVelocityToBody(preview_vel_x,
                                         preview_vel_y,
                                         &preview_body_vel_x,
                                         &preview_body_vel_y);
            preview_yaw_x10 = PointNav_LimitS16(yaw_deg * 10.0f, -32768, 32767);
            preview_yaw_err_x10 = PointNav_LimitS16(PointNav_AngleErrorDeg((float)POINT_NAV_TEST_TARGET_YAW_DEG,
                                                                           yaw_deg) * 10.0f,
                                                    -32768,
                                                    32767);
        }

        if(now_ms - last_preview_print_ms >= POINT_NAV_DEBUG_PERIOD_MS)
        {
            last_preview_print_ms = now_ms;
            printf("pnav_preview pos=%d,%d,%d tar=%d,%d,%d err=%d,%d,%d "
                   "yaw_x10=%d yaw_err_x10=%d vr=%d,%d,%d body=%d,%d,%d yaw_dps=%d "
                   "note=no_send_0x41\r\n",
                   Pos16_of_Radar.pos_data.x_x100,
                   Pos16_of_Radar.pos_data.y_x100,
                   Pos16_of_Radar.pos_data.z_x100,
                   POINT_NAV_TEST_TARGET_X_X100,
                   POINT_NAV_TEST_TARGET_Y_X100,
                   POINT_NAV_TEST_TARGET_Z_X100,
                   POINT_NAV_TEST_TARGET_X_X100 - Pos16_of_Radar.pos_data.x_x100,
                   POINT_NAV_TEST_TARGET_Y_X100 - Pos16_of_Radar.pos_data.y_x100,
                   POINT_NAV_TEST_TARGET_Z_X100 - Pos16_of_Radar.pos_data.z_x100,
                   preview_yaw_x10,
                   preview_yaw_err_x10,
                   preview_vel_x,
                   preview_vel_y,
                   preview_vel_z,
                   preview_body_vel_x,
                   preview_body_vel_y,
                   preview_vel_z,
                   preview_yaw_dps);
        }
    }
#endif

    if(state.is_unlocked &&
       Switch_sta_st.SWC == Switch_Mid &&
       Switch_sta_st.SWD == Switch_High &&
       Switch_sta_st.SWB == Switch_Low)
    {
        cmd_vel_sorce = Radar_Pid_vel;

        if(cmd_vel_sorce == Radar_Pid_vel && radar_healthy)
        {
            should_run = 1;
        }
    }

    if(should_run)
    {
        PointNavigation_SetTarget(POINT_NAV_TEST_TARGET_X_X100,
                                  POINT_NAV_TEST_TARGET_Y_X100,
                                  POINT_NAV_TEST_TARGET_Z_X100,
                                  POINT_NAV_TEST_TARGET_YAW_DEG);

        if(point_navigation_enable == 0)
        {
            PointNavigation_Start();
        }
    }
    else if(point_navigation_enable)
    {
        PointNavigation_Stop();
    }
}

void PointNavigation_Update(void)
{
    if(point_navigation_enable == 0)
    {
        return;
    }

    PointNav_GetFcTaskState(point_navigation_target.target_x,
                            point_navigation_target.target_y,
                            point_navigation_target.target_z,
                            point_navigation_target.target_yaw,
                            &point_navigation_state,
                            point_navigation_stable);

    if(cmd_vel_sorce == Radar_Pid_vel)
    {
        (void)PointNav_UpdateFromRadarPid();
    }
    else if(cmd_vel_sorce == JN_Cmd_vel)
    {
        (void)PointNav_UpdateFromJnCmdVel();
    }
    else if(cmd_vel_sorce == Camera_Pid_vel)
    {
        update_Flag.Camera_PID_Cmd_Vel = 0;
    }
}
