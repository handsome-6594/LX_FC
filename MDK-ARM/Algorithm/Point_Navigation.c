#include "Point_Navigation.h"
#include "Adaptive_PID.h"
#include "ANO_TO_H743_Data_Transmit.h"
#include "Freq_Detector.h"
#include "FC_State.h"
#include "JetsonNano_Data_Transmit.h"
#include "Remote_Control.h"
#include "RC_Channel.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stdio.h>

#define POINT_NAV_TEST_TARGET_X_X100   100
#define POINT_NAV_TEST_TARGET_Y_X100   0
#define POINT_NAV_TEST_TARGET_Z_X100   80
#define POINT_NAV_TEST_TARGET_YAW_DEG  0
#define POINT_NAV_SENSOR_TIMEOUT_MS    300
#define POINT_NAV_ALLOW_FLIGHT_TEST    (0U)
#define POINT_NAV_ENABLE_YAW_CONTROL   (0U)
#define POINT_NAV_YAW_DIR_TEST_ENABLE  (0U)
#define POINT_NAV_YAW_DIR_TEST_DPS     (10)
#define POINT_NAV_YAW_DIR_TEST_PRINT_MS (200U)
#define POINT_NAV_YAW_PID_DEBUG_ENABLE (1U)
#define POINT_NAV_YAW_PID_DEBUG_PRINT_MS (200U)

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

typedef struct
{
    Radar_Pos_16 pos;
    Radar_qua qua;
    u8 pos_update_cnt;
    u8 qua_update_cnt;
} PointNav_RadarSnapshot_t;

static u8 PointNav_RadarDataHealthy(void);

//一次性读取雷达位置，雷达四元数，更新计数
static void PointNav_GetRadarSnapshot(PointNav_RadarSnapshot_t *snapshot)
{
    if(snapshot == 0)
    {
        return;
    }

    taskENTER_CRITICAL();
    snapshot->pos = Pos16_of_Radar.pos_data;
    snapshot->qua = real_Radar_qua;
    snapshot->pos_update_cnt = radar_pos_update_cnt;
    snapshot->qua_update_cnt = radar_qua_update_cnt;
    taskEXIT_CRITICAL();
}

//限幅函数
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

//取绝对值的函数
static s32 PointNav_AbsS32(s32 value)
{
    return (value < 0) ? -value : value;
}

//计算yaw角误差，并限制在-180-180度
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

//用雷达四元数算当前yaw角，单位是度
static float PointNav_GetRadarYawDeg(const Radar_qua *qua)
{
    float qx;
    float qy;
    float qz;
    float qw;

    if(qua == 0)
    {
        return 0.0f;
    }

    qx = qua->qx;
    qy = qua->qy;
    qz = qua->qz;
    qw = qua->qw;

    return -atan2f(2.0f * qx * qy + 2.0f * qz * qw,
                   -2.0f * qy * qy - 2.0f * qz * qz + 1.0f) * 57.2957795f;
}

#if POINT_NAV_YAW_PID_DEBUG_ENABLE
static s32 PointNav_FloatToX100(float value)
{
    value *= 100.0f;
    return (s32)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static void PointNav_PrintYawPidDebug(float target_yaw_deg,
                                      float yaw_deg,
                                      float yaw_error_deg,
                                      float yaw_pid_out,
                                      s16 cmd_yaw_dps)
{
    static u32 last_print_ms;
    u32 now_ms = HAL_GetTick();

    if((now_ms - last_print_ms) < POINT_NAV_YAW_PID_DEBUG_PRINT_MS)
    {
        return;
    }

    last_print_ms = now_ms;
    printf("yaw_pid target=%d cur=%d err=%d out=%d cmd=%d\r\n",
           (int)PointNav_FloatToX100(target_yaw_deg),
           (int)PointNav_FloatToX100(yaw_deg),
           (int)PointNav_FloatToX100(yaw_error_deg),
           (int)PointNav_FloatToX100(yaw_pid_out),
           (int)cmd_yaw_dps);
}
#endif


//把雷达坐标系的速度转换成机体坐标系的速度
static void PointNav_RadarVelocityToBody(const Radar_qua *qua,
                                         s16 radar_vel_x,
                                         s16 radar_vel_y,
                                         s16 *body_vel_x,
                                         s16 *body_vel_y)
{
    float qx;
    float qy;
    float qz;
    float qw;
    float vx = (float)radar_vel_x;
    float vy = (float)radar_vel_y;
    float vz = 0.0f;
    float body_x;
    float body_y;
    float r00;
    float r01;
    float r02;
    float r10;
    float r11;
    float r12;

    if(qua == 0 || body_vel_x == 0 || body_vel_y == 0)
    {
        return;
    }

    qx = -qua->qx;
    qy = -qua->qy;
    qz = -qua->qz;
    qw = qua->qw;

    r00 = 1.0f - 2.0f * qy * qy - 2.0f * qz * qz;
    r01 = 2.0f * qx * qy - 2.0f * qw * qz;
    r02 = 2.0f * qx * qz + 2.0f * qw * qy;
    r10 = 2.0f * qx * qy + 2.0f * qw * qz;
    r11 = 1.0f - 2.0f * qx * qx - 2.0f * qz * qz;
    r12 = 2.0f * qy * qz - 2.0f * qw * qx;

    body_x = r00 * vx + r01 * vy + r02 * vz;
    body_y = r10 * vx + r11 * vy + r12 * vz;

    *body_vel_x = PointNav_LimitS16(body_x, -100, 100);
    *body_vel_y = PointNav_LimitS16(body_y, -100, 100);
}


//判断当前是否已经到目标点附近   分别判断高度，水平方向的距离，和yaw轴角度是否稳定
static void PointNav_GetFcTaskState(s16 target_x, s16 target_y, s16 target_z, s16 target_yaw,
                                    FC_TaskState_t *state, FC_Stable_t stable,
                                    const PointNav_RadarSnapshot_t *snapshot)
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

    if(state == 0 || snapshot == 0)
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

    if(PointNav_AbsS32((s32)snapshot->pos.z_x100 - (s32)target_z) > z_error_threshold)
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

    error_x = (float)(snapshot->pos.x_x100 - target_x);
    error_y = (float)(snapshot->pos.y_x100 - target_y);
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

    if(fabsf(PointNav_AngleErrorDeg((float)target_yaw, PointNav_GetRadarYawDeg(&snapshot->qua))) > (float)yaw_error_threshold)
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

//把导航速度写进候选控制量，最终 0x41 由仲裁器统一选择
static void PointNav_WriteRealtimeVelocity(s16 vel_x, s16 vel_y, s16 vel_z, s16 yaw_dps)
{
    nav_ctrl_cmd.data.roll = 0;
    nav_ctrl_cmd.data.pitch = 0;
    nav_ctrl_cmd.data.throttle = rc_ctrl_cmd.data.throttle;
    nav_ctrl_cmd.data.vel_x = vel_x;
    nav_ctrl_cmd.data.vel_y = vel_y;
    nav_ctrl_cmd.data.vel_z = vel_z;
    nav_ctrl_cmd.data.yaw_dps = yaw_dps;
}

//将速度清零
static void PointNav_ClearRealtimeVelocity(void)
{
    PointNav_WriteRealtimeVelocity(0, 0, 0, 0);
}

//速度控制量选择逻辑
static u8 PointNav_CanUseNavigationCmd(void)
{
    if(point_navigation_enable == 0)
    {
        return 0;
    }

    if(RemoteControl_IsSignalLost())
    {
        return 0;
    }

    if(state.is_unlocked == 0)
    {
        return 0;
    }

    if(RC_MotorIsUnlocked() == 0)
    {
        return 0;
    }

    if(cmd_vel_sorce == Radar_Pid_vel)
    {
        return PointNav_RadarDataHealthy();
    }

    if(cmd_vel_sorce == JN_Cmd_vel)
    {
        return 1;
    }

    return 0;
}

//给实时控制量选择速度来源
static void PointNav_RealtimeControlMuxUpdate(void)
{
    if(RemoteControl_IsSignalLost())
    {
        ctrl_of_realtime = failsafe_ctrl_cmd;
    }
    else if(PointNav_CanUseNavigationCmd())
    {
        ctrl_of_realtime = nav_ctrl_cmd;
    }
    else
    {
        ctrl_of_realtime = rc_ctrl_cmd;
    }

    Data.fun[0x41].wait_to_send = 1;
}

//判断雷达位置和四元数有没有正常更新
//超过 POINT_NAV_SENSOR_TIMEOUT_MS，也就是 300ms 没更新，就认为雷达不健康
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

//启动导航时，用当前雷达位置/yaw 初始化 PID 的上一次测量值
//这个函数是为了让点导航启动时 PID 从当前状态平滑开始
//而不是从 0 开始，防止第一次控制输出突然很大
static void PointNav_ResetControlPidFromMeasurement(void)
{
    PointNav_RadarSnapshot_t snapshot;
    float yaw_deg;

    PointNav_GetRadarSnapshot(&snapshot);
    yaw_deg = PointNav_GetRadarYawDeg(&snapshot.qua);

    PID_Reset(&loc_pid[PID_X]);
    PID_Reset(&loc_pid[PID_Y]);
    PID_Reset(&loc_pid[PID_Z]);
    PID_Reset(&loc_pid[PID_YAW]);

    loc_pid[PID_X].prev_measurement = (float)snapshot.pos.x_x100;
    loc_pid[PID_Y].prev_measurement = (float)snapshot.pos.y_x100;
    loc_pid[PID_Z].prev_measurement = (float)snapshot.pos.z_x100;
    loc_pid[PID_YAW].prev_measurement = yaw_deg;
    loc_pid[PID_YAW].prev_error = PointNav_AngleErrorDeg((float)point_navigation_target.target_yaw, yaw_deg);
}

//雷达定点导航的核心
static u8 PointNav_UpdateFromRadarPid(const PointNav_RadarSnapshot_t *snapshot)
{
    static u8 last_radar_pos_update_cnt;
    s16 vel_x;
    s16 vel_y;
    s16 vel_z;
    s16 yaw_dps;
    s16 body_vel_x;
    s16 body_vel_y;
    float yaw_deg;
    float yaw_error_deg;
    float yaw_pid_out;

    if(snapshot == 0)
    {
        return 0;
    }

    if(last_radar_pos_update_cnt == snapshot->pos_update_cnt)
    {
        return 0;
    }

    last_radar_pos_update_cnt = snapshot->pos_update_cnt;

    yaw_deg = PointNav_GetRadarYawDeg(&snapshot->qua);
    yaw_error_deg = PointNav_AngleErrorDeg((float)point_navigation_target.target_yaw, yaw_deg);
    vel_x = PointNav_LimitS16(PID_Update(&loc_pid[PID_X],
                                         (float)point_navigation_target.target_x,
                                         (float)snapshot->pos.x_x100),
                              -100, 100);
    vel_y = PointNav_LimitS16(PID_Update(&loc_pid[PID_Y],
                                         (float)point_navigation_target.target_y,
                                         (float)snapshot->pos.y_x100),
                              -100, 100);
    vel_z = PointNav_LimitS16(PID_Update(&loc_pid[PID_Z],
                                         (float)point_navigation_target.target_z,
                                         (float)snapshot->pos.z_x100),
                              -100, 100);
    yaw_pid_out = PID_UpdateYaw(&loc_pid[PID_YAW],
                                (float)point_navigation_target.target_yaw,
                                yaw_deg);
    yaw_dps = PointNav_LimitS16(yaw_pid_out, -200, 200);
#if POINT_NAV_ENABLE_YAW_CONTROL == 0U
    yaw_dps = 0;
#endif
#if POINT_NAV_YAW_PID_DEBUG_ENABLE
    PointNav_PrintYawPidDebug((float)point_navigation_target.target_yaw,
                              yaw_deg,
                              yaw_error_deg,
                              yaw_pid_out,
                              yaw_dps);
#endif

    PointNav_RadarVelocityToBody(&snapshot->qua, vel_x, vel_y, &body_vel_x, &body_vel_y);

    PointNav_WriteRealtimeVelocity(body_vel_x, body_vel_y, vel_z, yaw_dps);
    update_Flag.Radar_PID_Cmd_Vel = 1;
    FreqDetector_OnData(&JN_freq_detector[Data_stream_Radar_cmd_vel]);

    return 1;
}

//不用pid，直接使用JetsonNano发来的速度指令
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

//初始化导航模块
void PointNavigation_Init(void)
{
    PID_Init();
    point_navigation_enable = 0;
    cmd_vel_sorce = Radar_Pid_vel;
}

//启动点导航
void PointNavigation_Start(void)
{
    PointNav_RadarSnapshot_t snapshot;

    PointNav_GetRadarSnapshot(&snapshot);

    point_navigation_target.target_x = snapshot.pos.x_x100;
    point_navigation_target.target_y = snapshot.pos.y_x100;
    point_navigation_target.target_z = snapshot.pos.z_x100;
    point_navigation_target.target_yaw = (s16)PointNav_GetRadarYawDeg(&snapshot.qua);

    PointNav_ResetControlPidFromMeasurement();
    PointNav_ClearRealtimeVelocity();
    point_navigation_enable = 1;
}

//停止点导航
void PointNavigation_Stop(void)
{
    point_navigation_enable = 0;
    update_Flag.Radar_PID_Cmd_Vel = 0;
    update_Flag.Camera_PID_Cmd_Vel = 0;
    PointNav_ClearRealtimeVelocity();
}

//设置速度来源的函数：来源于雷达 PID、相机 PID、JetsonNano 速度
void PointNavigation_SetCmdVelSource(_cmd_vel_sorce source)
{
    cmd_vel_sorce = source;
}

//设置稳定条件，比如选择误差阈值还是稳定时间
void PointNavigation_SetStableCondition(FC_Stable_t stable)
{
    point_navigation_stable = stable;
}

//设置目标点和目标角
void PointNavigation_SetTarget(s16 target_x, s16 target_y, s16 target_z, s16 target_yaw)
{
    point_navigation_target.target_x = target_x;
    point_navigation_target.target_y = target_y;
    point_navigation_target.target_z = target_z;
    point_navigation_target.target_yaw = target_yaw;
}

//测试用，可以删
void PointNavigation_TestPointTask(void)
{
#if POINT_NAV_ALLOW_FLIGHT_TEST
    u8 should_run = 0;
    u8 radar_healthy;

    radar_healthy = PointNav_RadarDataHealthy();

    if(state.is_unlocked &&
       Switch_sta_st.SWC == Switch_Mid &&
       Switch_sta_st.SWD == Switch_High &&
       Switch_sta_st.SWB == Switch_High)
    {
        cmd_vel_sorce = Radar_Pid_vel;

        if(cmd_vel_sorce == Radar_Pid_vel && radar_healthy)
        {
            should_run = 1;
        }
    }

    if(should_run)
    {
        if(point_navigation_enable == 0)
        {
            PointNavigation_Start();
        }

        PointNavigation_SetTarget(POINT_NAV_TEST_TARGET_X_X100,
                                  POINT_NAV_TEST_TARGET_Y_X100,
                                  POINT_NAV_TEST_TARGET_Z_X100,
                                  POINT_NAV_TEST_TARGET_YAW_DEG);
    }
    else if(point_navigation_enable)
    {
        PointNavigation_Stop();
    }
#endif
}

#if POINT_NAV_YAW_DIR_TEST_ENABLE
static u8 PointNav_YawDirectionTestUpdate(void)
{
    static u8 was_active;
    static u32 last_print_ms;
    u32 now_ms = HAL_GetTick();
    u8 active;

    active = (RemoteControl_IsSignalLost() == 0U) &&
             (state.is_unlocked != 0U) &&
             (RC_MotorIsUnlocked() != 0U) &&
             (Switch_sta_st.SWC == Switch_Mid) &&
             (Switch_sta_st.SWD == Switch_High) &&
             (Switch_sta_st.SWB == Switch_High);

    if(active == 0U)
    {
        if(was_active != 0U)
        {
            point_navigation_enable = 0;
            PointNav_ClearRealtimeVelocity();
            PointNav_RealtimeControlMuxUpdate();
            printf("yaw_dir_test stop\r\n");
        }

        was_active = 0;
        return 0;
    }

    if(was_active == 0U)
    {
        was_active = 1;
        last_print_ms = 0;
        printf("yaw_dir_test start\r\n");
    }

    point_navigation_enable = 1;
    cmd_vel_sorce = JN_Cmd_vel;
    PointNav_WriteRealtimeVelocity(0, 0, 0, POINT_NAV_YAW_DIR_TEST_DPS);
    PointNav_RealtimeControlMuxUpdate();

    if((now_ms - last_print_ms) >= POINT_NAV_YAW_DIR_TEST_PRINT_MS)
    {
        last_print_ms = now_ms;
        printf("yaw_dir_test cmd=%d tx=%d bytes=%02X,%02X rc=%d pwm=%u,%u,%u,%u\r\n",
               POINT_NAV_YAW_DIR_TEST_DPS,
               ctrl_of_realtime.data.yaw_dps,
               (unsigned int)ctrl_of_realtime.byte[6],
               (unsigned int)ctrl_of_realtime.byte[7],
               rc_ctrl_cmd.data.yaw_dps,
               (unsigned int)pwm_to_esc.pwm_value1,
               (unsigned int)pwm_to_esc.pwm_value2,
               (unsigned int)pwm_to_esc.pwm_value3,
               (unsigned int)pwm_to_esc.pwm_value4);
    }

    return 1;
}
#endif

//导航主循环函数
void PointNavigation_Update(void)
{
    PointNav_RadarSnapshot_t snapshot;

#if POINT_NAV_YAW_DIR_TEST_ENABLE
    if(PointNav_YawDirectionTestUpdate() != 0U)
    {
        return;
    }
#endif

    if(point_navigation_enable == 0)
    {
        PointNav_RealtimeControlMuxUpdate();
        return;
    }

    if(RemoteControl_IsSignalLost() || state.is_unlocked == 0 || RC_MotorIsUnlocked() == 0)
    {
        PointNavigation_Stop();
        PointNav_RealtimeControlMuxUpdate();
        return;
    }

    if(cmd_vel_sorce == Radar_Pid_vel && PointNav_RadarDataHealthy() == 0)
    {
        PointNavigation_Stop();
        PointNav_RealtimeControlMuxUpdate();
        return;
    }

    PointNav_GetRadarSnapshot(&snapshot);

    PointNav_GetFcTaskState(point_navigation_target.target_x,
                            point_navigation_target.target_y,
                            point_navigation_target.target_z,
                            point_navigation_target.target_yaw,
                            &point_navigation_state,
                            point_navigation_stable,
                            &snapshot);

    if(cmd_vel_sorce == Radar_Pid_vel)
    {
        (void)PointNav_UpdateFromRadarPid(&snapshot);
    }
    else if(cmd_vel_sorce == JN_Cmd_vel)
    {
        (void)PointNav_UpdateFromJnCmdVel();
    }
    else if(cmd_vel_sorce == Camera_Pid_vel)
    {
        update_Flag.Camera_PID_Cmd_Vel = 0;
    }

    PointNav_RealtimeControlMuxUpdate();
}
