#include "WayPoint_Mission.h"
#include "Adaptive_PID.h"
#include "FreeRTOS.h"
#include "JetsonNano_Data_Transmit.h"
#include "task.h"
#include <math.h>
#include <string.h>

#define WAYPOINT_LINE_MIN_LENGTH       (1.0f)
#define WAYPOINT_DEFAULT_MAX_XY_SPEED  (100)
#define WAYPOINT_DEFAULT_MIN_XY_SPEED  (8)
#define WAYPOINT_DEFAULT_MAX_Z_SPEED   (100)
#define WAYPOINT_DEFAULT_MAX_YAW_DPS   (200)
#define WAYPOINT_DEFAULT_SLOW_DISTANCE (80)

static WayPointMissionConfig_t waypoint_config = {
    .max_xy_speed = WAYPOINT_DEFAULT_MAX_XY_SPEED,
    .min_xy_speed = WAYPOINT_DEFAULT_MIN_XY_SPEED,
    .max_z_speed = WAYPOINT_DEFAULT_MAX_Z_SPEED,
    .max_yaw_dps = WAYPOINT_DEFAULT_MAX_YAW_DPS,
    .slow_down_distance = WAYPOINT_DEFAULT_SLOW_DISTANCE,
};

static WayPointMissionState_t waypoint_state;
static u8 waypoint_last_pos_update_cnt;
static u8 waypoint_has_last_target;
static u8 waypoint_last_segment_id;
static s16 waypoint_last_target_x;
static s16 waypoint_last_target_y;
static s16 waypoint_last_target_z;
static s16 waypoint_last_target_yaw;
static u8 waypoint_alt_ok;
static u8 waypoint_center_ok;
static u8 waypoint_yaw_ok;

//限幅函数
static s16 WayPoint_LimitS16(float value, s16 min_value, s16 max_value)
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

//取绝对值
static s32 WayPoint_AbsS32(s32 value)
{
    return (value < 0) ? -value : value;
}

//浮点数取绝对值
static float WayPoint_AbsF(float value)
{
    return (value < 0.0f) ? -value : value;
}

//角度限制在-180-180度
static float WayPoint_AngleErrorDeg(float target_deg, float measurement_deg)
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

//把当前雷达发来的 yaw 角，从整数放大格式转换成真实角度单位
static float WayPoint_GetRadarYawDeg(void)
{
    return (float)Radar_YAW_tar_un.st_data.yaw_x100 * 0.01f;
}

//一次性读取当前雷达位置、四元数、位置更新计数
static void WayPoint_GetRadarSnapshot(Radar_Pos_16 *pos,
                                      Radar_qua *qua,
                                      u8 *pos_update_cnt)
{
    if(pos == 0 || qua == 0 || pos_update_cnt == 0)
    {
        return;
    }

    taskENTER_CRITICAL();
    *pos = Pos16_of_Radar.pos_data;
    *qua = real_Radar_qua;
    *pos_update_cnt = radar_pos_update_cnt;
    taskEXIT_CRITICAL();
}

//有点重复，可以改
//将雷达坐标系速度转换为机体坐标系速度
static void WayPoint_RadarVelocityToBody(const Radar_qua *qua,
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
    float body_x;
    float body_y;
    float r00;
    float r01;
    float r10;
    float r11;

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
    r10 = 2.0f * qx * qy + 2.0f * qw * qz;
    r11 = 1.0f - 2.0f * qx * qx - 2.0f * qz * qz;

    body_x = r00 * vx + r01 * vy;
    body_y = r10 * vx + r11 * vy;

    *body_vel_x = WayPoint_LimitS16(body_x,
                                    (s16)-waypoint_config.max_xy_speed,
                                    waypoint_config.max_xy_speed);
    *body_vel_y = WayPoint_LimitS16(body_y,
                                    (s16)-waypoint_config.max_xy_speed,
                                    waypoint_config.max_xy_speed);
}

//限制xy轴速度的合速度大小
static void WayPoint_LimitVector(float *x, float *y, float max_length)
{
    float length;
    float scale;

    if(x == 0 || y == 0 || max_length <= 0.0f)
    {
        return;
    }

    length = sqrtf((*x) * (*x) + (*y) * (*y));
    if(length <= max_length || length <= 0.001f)
    {
        return;
    }

    scale = max_length / length;
    *x *= scale;
    *y *= scale;
}

//末端减速函数
//如果距离终点还很远，就允许最大速度；如果进入 slow_down_distance，就按剩余距离逐渐降低最大沿线速度，减少冲过头
static float WayPoint_GetSlowDownSpeedLimit(float remaining)
{
    float max_speed = (float)waypoint_config.max_xy_speed;
    float min_speed = (float)waypoint_config.min_xy_speed;
    float slow_distance = (float)waypoint_config.slow_down_distance;
    float limit;

    if(max_speed <= 0.0f)
    {
        return 0.0f;
    }

    if(slow_distance <= 0.0f || remaining >= slow_distance)
    {
        return max_speed;
    }

    if(remaining < 0.0f)
    {
        remaining = 0.0f;
    }

    limit = max_speed * (remaining / slow_distance);
    if(limit < min_speed)
    {
        limit = min_speed;
    }

    if(limit > max_speed)
    {
        limit = max_speed;
    }

    return limit;
}

//清零到点稳定状态
static void WayPoint_ResetStableState(FC_TaskState_t *task_state)
{
    waypoint_alt_ok = 0;
    waypoint_center_ok = 0;
    waypoint_yaw_ok = 0;
    waypoint_state.target_reached = 0;

    if(task_state != 0)
    {
        task_state->alt_stable = 0;
        task_state->center_stable = 0;
        task_state->yaw_stable = 0;
    }
}

//切换新航段时，对齐 X/Y PID 的历史量
static void WayPoint_AlignPidHistory(float along, float cross)
{
    loc_pid[PID_X].prev_measurement = along;
    loc_pid[PID_X].prev_error = waypoint_state.line_length - along;
    loc_pid[PID_X].d_filtered = 0.0f;

    loc_pid[PID_Y].prev_measurement = cross;
    loc_pid[PID_Y].prev_error = -cross;
    loc_pid[PID_Y].d_filtered = 0.0f;
}

//判断当前是否到点稳定
static void WayPoint_UpdateStableState(const Radar_Pos_16 *pos,
                                       s16 target_z,
                                       s16 target_yaw,
                                       FC_TaskState_t *task_state,
                                       FC_Stable_t stable)
{
    s16 z_error_threshold;
    s16 xy_error_threshold;
    s16 yaw_error_threshold;
    float yaw_error;
    float target_distance;

    if(pos == 0 || task_state == 0)
    {
        return;
    }

    if(stable.alt_ok_time > 200U) stable.alt_ok_time = 200U;
    if(stable.center_ok_time > 200U) stable.center_ok_time = 200U;
    if(stable.yaw_ok_time > 200U) stable.yaw_ok_time = 200U;
    if(stable.alt_ok_time == 0U) stable.alt_ok_time = 1U;
    if(stable.center_ok_time == 0U) stable.center_ok_time = 1U;
    if(stable.yaw_ok_time == 0U) stable.yaw_ok_time = 1U;

    z_error_threshold = (stable.alt_error_threshold == 0U) ? 7 : stable.alt_error_threshold;
    xy_error_threshold = (stable.center_error_threshold == 0U) ? 10 : stable.center_error_threshold;
    yaw_error_threshold = (stable.yaw_error_threshold == 0U) ? 7 : stable.yaw_error_threshold;

    if(WayPoint_AbsS32((s32)pos->z_x100 - (s32)target_z) > z_error_threshold)
    {
        waypoint_alt_ok = 0;
        task_state->alt_stable = 0;
    }
    else if(waypoint_alt_ok < stable.alt_ok_time)
    {
        waypoint_alt_ok++;
    }
    else
    {
        task_state->alt_stable = 1;
    }

    target_distance = sqrtf(((float)waypoint_state.target_x - (float)pos->x_x100) *
                            ((float)waypoint_state.target_x - (float)pos->x_x100) +
                            ((float)waypoint_state.target_y - (float)pos->y_x100) *
                            ((float)waypoint_state.target_y - (float)pos->y_x100));

    if(target_distance > (float)xy_error_threshold ||
       WayPoint_AbsF(waypoint_state.cross) > (float)xy_error_threshold)
    {
        waypoint_center_ok = 0;
        task_state->center_stable = 0;
    }
    else if(waypoint_center_ok < stable.center_ok_time)
    {
        waypoint_center_ok++;
    }
    else
    {
        task_state->center_stable = 1;
    }

    yaw_error = WayPoint_AngleErrorDeg((float)target_yaw, WayPoint_GetRadarYawDeg());
    if(WayPoint_AbsF(yaw_error) > (float)yaw_error_threshold)
    {
        waypoint_yaw_ok = 0;
        task_state->yaw_stable = 0;
    }
    else if(waypoint_yaw_ok < stable.yaw_ok_time)
    {
        waypoint_yaw_ok++;
    }
    else
    {
        task_state->yaw_stable = 1;
    }

    waypoint_state.target_reached =
        (task_state->alt_stable != 0U &&
         task_state->center_stable != 0U &&
         task_state->yaw_stable != 0U) ? 1U : 0U;
}

//判断目标点或航段编号是否变化
static u8 WayPoint_TargetChanged(s16 target_x,
                                 s16 target_y,
                                 s16 target_z,
                                 s16 target_yaw,
                                 u8 segment_id)
{
    if(waypoint_has_last_target == 0U)
    {
        return 1U;
    }

    if(waypoint_last_segment_id != segment_id)
    {
        return 1U;
    }

    return (waypoint_last_target_x != target_x ||
            waypoint_last_target_y != target_y ||
            waypoint_last_target_z != target_z ||
            waypoint_last_target_yaw != target_yaw) ? 1U : 0U;
}

//保存当前目标点和航段编号，供下次判断是否变化
static void WayPoint_SetLastTarget(s16 target_x,
                                   s16 target_y,
                                   s16 target_z,
                                   s16 target_yaw,
                                   u8 segment_id)
{
    waypoint_has_last_target = 1U;
    waypoint_last_segment_id = segment_id;
    waypoint_last_target_x = target_x;
    waypoint_last_target_y = target_y;
    waypoint_last_target_z = target_z;
    waypoint_last_target_yaw = target_yaw;
}

//Reset的衣服
void WayPointMission_Init(void)
{
    WayPointMission_Reset();
}

//清空航线函数
void WayPointMission_Reset(void)
{
    memset(&waypoint_state, 0, sizeof(waypoint_state));
    waypoint_last_pos_update_cnt = radar_pos_update_cnt;
    waypoint_has_last_target = 0U;
    waypoint_last_segment_id = 0U;
    waypoint_last_target_x = 0;
    waypoint_last_target_y = 0;
    waypoint_last_target_z = 0;
    waypoint_last_target_yaw = 0;
    waypoint_alt_ok = 0U;
    waypoint_center_ok = 0U;
    waypoint_yaw_ok = 0U;
}

//设置航线控制函数
//会做一些保护，比如最大速度不能小于等于 0，min_xy_speed 不能大于 max_xy_speed
void WayPointMission_SetConfig(WayPointMissionConfig_t config)
{
    if(config.max_xy_speed <= 0)
    {
        config.max_xy_speed = WAYPOINT_DEFAULT_MAX_XY_SPEED;
    }

    if(config.min_xy_speed < 0)
    {
        config.min_xy_speed = 0;
    }

    if(config.min_xy_speed > config.max_xy_speed)
    {
        config.min_xy_speed = config.max_xy_speed;
    }

    if(config.max_z_speed <= 0)
    {
        config.max_z_speed = WAYPOINT_DEFAULT_MAX_Z_SPEED;
    }

    if(config.max_yaw_dps <= 0)
    {
        config.max_yaw_dps = WAYPOINT_DEFAULT_MAX_YAW_DPS;
    }

    if(config.max_yaw_dps > 327)
    {
        config.max_yaw_dps = 327;
    }

    if(config.slow_down_distance < 0)
    {
        config.slow_down_distance = 0;
    }

    waypoint_config = config;
}

//返回当前航线控制参数。主要用于调试、OLED 显示或者确认当前参数
WayPointMissionConfig_t WayPointMission_GetConfig(void)
{
    return waypoint_config;
}

//返回当前航线状态指针
//这个函数很适合后面接 OLED 菜单调试
const WayPointMissionState_t *WayPointMission_GetState(void)
{
    return &waypoint_state;
}


//核心函数：根据当前雷达位置和目标点，计算一帧新的航线速度指令
u8 WayPointMission_UpdateLineTarget(s16 target_x,
                                    s16 target_y,
                                    s16 target_z,
                                    s16 target_yaw,
                                    u8 segment_id,
                                    FC_TaskState_t *task_state,
                                    FC_Stable_t stable,
                                    Radar_Cmd_Vel *body_cmd_vel)
{
    Radar_Pos_16 pos = {0};
    Radar_qua qua = {0};
    u8 pos_update_cnt = 0U;
    u8 changed;
    float dx_line;
    float dy_line;
    float ux = 0.0f;
    float uy = 0.0f;
    float dx_current;
    float dy_current;
    float along_cmd;
    float cross_cmd;
    float max_along_speed;
    float radar_cmd_x;
    float radar_cmd_y;
    float yaw_cmd;
    s16 radar_vel_x;
    s16 radar_vel_y;

    if(body_cmd_vel == 0)
    {
        return 0U;
    }

    memset(body_cmd_vel, 0, sizeof(*body_cmd_vel));

    WayPoint_GetRadarSnapshot(&pos, &qua, &pos_update_cnt);
    if(pos_update_cnt == waypoint_last_pos_update_cnt)
    {
        return 0U;
    }

    waypoint_last_pos_update_cnt = pos_update_cnt;
    changed = WayPoint_TargetChanged(target_x, target_y, target_z, target_yaw, segment_id);

    if(changed != 0U)
    {
        waypoint_state.start_x = pos.x_x100;
        waypoint_state.start_y = pos.y_x100;
        WayPoint_SetLastTarget(target_x, target_y, target_z, target_yaw, segment_id);
        WayPoint_ResetStableState(task_state);
    }

    waypoint_state.active = 1U;
    waypoint_state.segment_id = segment_id;
    waypoint_state.target_x = target_x;
    waypoint_state.target_y = target_y;
    waypoint_state.target_z = target_z;
    waypoint_state.target_yaw = target_yaw;
    waypoint_state.current_x = pos.x_x100;
    waypoint_state.current_y = pos.y_x100;
    waypoint_state.current_z = pos.z_x100;

    dx_line = (float)target_x - (float)waypoint_state.start_x;
    dy_line = (float)target_y - (float)waypoint_state.start_y;
    waypoint_state.line_length = sqrtf(dx_line * dx_line + dy_line * dy_line);

    if(waypoint_state.line_length > WAYPOINT_LINE_MIN_LENGTH)
    {
        waypoint_state.line_valid = 1U;
        ux = dx_line / waypoint_state.line_length;
        uy = dy_line / waypoint_state.line_length;
    }
    else
    {
        waypoint_state.line_valid = 0U;
    }

    dx_current = (float)pos.x_x100 - (float)waypoint_state.start_x;
    dy_current = (float)pos.y_x100 - (float)waypoint_state.start_y;

    if(waypoint_state.line_valid != 0U)
    {
        waypoint_state.along = dx_current * ux + dy_current * uy;
        waypoint_state.cross = -dx_current * uy + dy_current * ux;
        waypoint_state.remaining = waypoint_state.line_length - waypoint_state.along;

        if(changed != 0U)
        {
            WayPoint_AlignPidHistory(waypoint_state.along, waypoint_state.cross);
        }

        along_cmd = PID_Update(&loc_pid[PID_X],
                               waypoint_state.line_length,
                               waypoint_state.along);
        cross_cmd = PID_Update(&loc_pid[PID_Y], 0.0f, waypoint_state.cross);

        max_along_speed = WayPoint_GetSlowDownSpeedLimit(waypoint_state.remaining);
        along_cmd = (float)WayPoint_LimitS16(along_cmd,
                                             (s16)-waypoint_config.max_xy_speed,
                                             (s16)max_along_speed);
        cross_cmd = (float)WayPoint_LimitS16(cross_cmd,
                                             (s16)-waypoint_config.max_xy_speed,
                                             waypoint_config.max_xy_speed);

        radar_cmd_x = along_cmd * ux - cross_cmd * uy;
        radar_cmd_y = along_cmd * uy + cross_cmd * ux;
    }
    else
    {
        waypoint_state.along = 0.0f;
        waypoint_state.cross = 0.0f;
        waypoint_state.remaining = 0.0f;

        if(changed != 0U)
        {
            loc_pid[PID_X].prev_measurement = (float)pos.x_x100;
            loc_pid[PID_X].prev_error = (float)target_x - (float)pos.x_x100;
            loc_pid[PID_X].d_filtered = 0.0f;
            loc_pid[PID_Y].prev_measurement = (float)pos.y_x100;
            loc_pid[PID_Y].prev_error = (float)target_y - (float)pos.y_x100;
            loc_pid[PID_Y].d_filtered = 0.0f;
        }

        radar_cmd_x = PID_Update(&loc_pid[PID_X], (float)target_x, (float)pos.x_x100);
        radar_cmd_y = PID_Update(&loc_pid[PID_Y], (float)target_y, (float)pos.y_x100);
    }

    WayPoint_LimitVector(&radar_cmd_x, &radar_cmd_y, (float)waypoint_config.max_xy_speed);
    radar_vel_x = WayPoint_LimitS16(radar_cmd_x,
                                    (s16)-waypoint_config.max_xy_speed,
                                    waypoint_config.max_xy_speed);
    radar_vel_y = WayPoint_LimitS16(radar_cmd_y,
                                    (s16)-waypoint_config.max_xy_speed,
                                    waypoint_config.max_xy_speed);

    WayPoint_RadarVelocityToBody(&qua,
                                 radar_vel_x,
                                 radar_vel_y,
                                 &body_cmd_vel->vx_x100,
                                 &body_cmd_vel->vy_x100);

    body_cmd_vel->vz_x100 = WayPoint_LimitS16(PID_Update(&loc_pid[PID_Z],
                                                         (float)target_z,
                                                         (float)pos.z_x100),
                                              (s16)-waypoint_config.max_z_speed,
                                              waypoint_config.max_z_speed);

    yaw_cmd = PID_UpdateYaw(&loc_pid[PID_YAW],
                            (float)target_yaw,
                            WayPoint_GetRadarYawDeg());
    waypoint_state.yaw_dps = WayPoint_LimitS16(yaw_cmd,
                                               (s16)-waypoint_config.max_yaw_dps,
                                               waypoint_config.max_yaw_dps);
    body_cmd_vel->yaw_x100 = (s16)(waypoint_state.yaw_dps * 100);

    waypoint_state.radar_vel_x = radar_vel_x;
    waypoint_state.radar_vel_y = radar_vel_y;
    waypoint_state.body_vel_x = body_cmd_vel->vx_x100;
    waypoint_state.body_vel_y = body_cmd_vel->vy_x100;
    waypoint_state.vel_z = body_cmd_vel->vz_x100;

    WayPoint_UpdateStableState(&pos, target_z, target_yaw, task_state, stable);
    if(waypoint_state.target_reached != 0U)
    {
        memset(body_cmd_vel, 0, sizeof(*body_cmd_vel));
        waypoint_state.radar_vel_x = 0;
        waypoint_state.radar_vel_y = 0;
        waypoint_state.body_vel_x = 0;
        waypoint_state.body_vel_y = 0;
        waypoint_state.vel_z = 0;
        waypoint_state.yaw_dps = 0;
    }

    return 1U;
}
