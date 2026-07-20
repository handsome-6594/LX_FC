#include "User_Task.h"
#include "FC_State.h"
#include "JetsonNano_Data_Transmit.h"
#include "Map_Update.h"
#include "Point_Navigation.h"
#include "RC_Channel.h"
#include "Remote_Control.h"
#include "WayPoint_Mission.h"
#include <stdio.h>

#define USER_TASK_SENSOR_TIMEOUT_MS      300U
#define USER_TASK_TAKEOFF_Z_X100         75
#define USER_TASK_TEST_POINT_COUNT       2U
#define USER_TASK_DEBUG_ENABLE           (0U)
#define USER_TASK_DEBUG_PERIOD_MS        200U

typedef enum
{
    USER_TASK_STATE_IDLE = 0,
    USER_TASK_STATE_TAKEOFF,
    USER_TASK_STATE_LOAD_POINT,
    USER_TASK_STATE_FLY_LINE,
    USER_TASK_STATE_HOLD_LAST,
} UserTaskState_e;

static UserTaskState_e user_task_state = USER_TASK_STATE_IDLE;
static FC_TaskState_t user_task_fc_state;
static FC_Stable_t user_task_stable = {
    .alt_ok_time = 20,
    .alt_error_threshold = 7,
    .center_ok_time = 20,
    .center_error_threshold = 10,
    .yaw_ok_time = 10,
    .yaw_error_threshold = 10,
};
static Point_t user_task_target_point;
static s16 user_task_takeoff_x;
static s16 user_task_takeoff_y;
static s16 user_task_target_yaw;
static u8 user_task_segment_id;
static u8 user_task_active;
static u8 user_task_initialized;

static u8 UserTask_SwitchRequested(void)
{
    return (Switch_sta_st.SWC == Switch_Mid &&
            Switch_sta_st.SWD == Switch_High &&
            Switch_sta_st.SWB == Switch_Low) ? 1U : 0U;
}

static u8 UserTask_SystemReady(void)
{
    return (RemoteControl_IsSignalLost() == 0U &&
            state.is_unlocked != 0U &&
            RC_MotorIsUnlocked() != 0U) ? 1U : 0U;
}

static u8 UserTask_RadarDataHealthy(void)
{
    static u8 last_pos_update_cnt;
    static u8 last_qua_update_cnt;
    static u8 last_yaw_update_cnt;
    static u32 last_pos_ms;
    static u32 last_qua_ms;
    static u32 last_yaw_ms;
    static u8 pos_seen;
    static u8 qua_seen;
    static u8 yaw_seen;
    static u8 inited;
    u32 now_ms = HAL_GetTick();

    if(inited == 0U)
    {
        inited = 1U;
        last_pos_update_cnt = radar_pos_update_cnt;
        last_qua_update_cnt = radar_qua_update_cnt;
        last_yaw_update_cnt = radar_yaw_update_cnt;
        pos_seen = (radar_pos_update_cnt != 0U) ? 1U : 0U;
        qua_seen = (radar_qua_update_cnt != 0U) ? 1U : 0U;
        yaw_seen = (radar_yaw_update_cnt != 0U) ? 1U : 0U;
        last_pos_ms = now_ms;
        last_qua_ms = now_ms;
        last_yaw_ms = now_ms;
    }

    if(last_pos_update_cnt != radar_pos_update_cnt)
    {
        last_pos_update_cnt = radar_pos_update_cnt;
        pos_seen = 1U;
        last_pos_ms = now_ms;
    }

    if(last_qua_update_cnt != radar_qua_update_cnt)
    {
        last_qua_update_cnt = radar_qua_update_cnt;
        qua_seen = 1U;
        last_qua_ms = now_ms;
    }

    if(last_yaw_update_cnt != radar_yaw_update_cnt)
    {
        last_yaw_update_cnt = radar_yaw_update_cnt;
        yaw_seen = 1U;
        last_yaw_ms = now_ms;
    }

    if(pos_seen == 0U || qua_seen == 0U || yaw_seen == 0U)
    {
        return 0U;
    }

    return (((now_ms - last_pos_ms) < USER_TASK_SENSOR_TIMEOUT_MS) &&
            ((now_ms - last_qua_ms) < USER_TASK_SENSOR_TIMEOUT_MS) &&
            ((now_ms - last_yaw_ms) < USER_TASK_SENSOR_TIMEOUT_MS)) ? 1U : 0U;
}

static s16 UserTask_GetCurrentYawDeg(void)
{
    s16 yaw_x100 = Radar_YAW_tar_un.st_data.yaw_x100;

    if(yaw_x100 >= 0)
    {
        return (s16)((yaw_x100 + 50) / 100);
    }

    return (s16)((yaw_x100 - 50) / 100);
}

static void UserTask_PublishVelocity(const Radar_Cmd_Vel *cmd)
{
#if USER_TASK_DEBUG_ENABLE
    static u32 last_print_ms;
    const WayPointMissionState_t *line_state;
    u32 now_ms;
#endif

    if(cmd == 0)
    {
        return;
    }

    speed_cmd_un.cmd_vel_data = *cmd;
    update_Flag.Radar_Cmd_Vel = 1U;

#if USER_TASK_DEBUG_ENABLE
    now_ms = HAL_GetTick();
    if((now_ms - last_print_ms) >= USER_TASK_DEBUG_PERIOD_MS)
    {
        last_print_ms = now_ms;
        line_state = WayPointMission_GetState();
        if(line_state != 0)
        {
            printf("wp st=%u seg=%u cur[%d,%d,%d] tar[%d,%d,%d] len=%d along=%d cross=%d rem=%d rv[%d,%d] bv[%d,%d,%d] yaw=%d stable[%u,%u,%u]\r\n",
                   user_task_state,
                   user_task_segment_id,
                   line_state->current_x,
                   line_state->current_y,
                   line_state->current_z,
                   line_state->target_x,
                   line_state->target_y,
                   line_state->target_z,
                   (s16)line_state->line_length,
                   (s16)line_state->along,
                   (s16)line_state->cross,
                   (s16)line_state->remaining,
                   line_state->radar_vel_x,
                   line_state->radar_vel_y,
                   cmd->vx_x100,
                   cmd->vy_x100,
                   cmd->vz_x100,
                   (s16)(cmd->yaw_x100 / 100),
                   user_task_fc_state.alt_stable,
                   user_task_fc_state.center_stable,
                   user_task_fc_state.yaw_stable);
        }
    }
#endif
}

static void UserTask_PublishZeroVelocity(void)
{
    Radar_Cmd_Vel cmd = {0};

    UserTask_PublishVelocity(&cmd);
}

static u8 UserTask_LoadTestWayPoints(void)
{
    Point_t test_points[USER_TASK_TEST_POINT_COUNT];
    u8 count = 0U;
    u8 i;

    WayPointClear();
    for(i = 0U; i < USER_TASK_TEST_POINT_COUNT; i++)
    {
        if(MapPoint_IsValid(All_Point[i]) != 0U)
        {
            test_points[count] = All_Point[i];
            count++;
        }
    }

    if(count == 0U)
    {
        return 0U;
    }

    return Add_WayPoint(test_points, count) ? 1U : 0U;
}

static void UserTask_ResetMission(void)
{
    user_task_state = USER_TASK_STATE_IDLE;
    user_task_active = 0U;
    user_task_segment_id = 0U;
    user_task_target_point.x = 0;
    user_task_target_point.y = 0;
    user_task_fc_state.alt_stable = 0U;
    user_task_fc_state.center_stable = 0U;
    user_task_fc_state.yaw_stable = 0U;
    WayPointMission_Reset();
}

static void UserTask_StartMission(void)
{
    WayPointMissionConfig_t config = {
        .max_xy_speed = 30,
        .min_xy_speed = 5,
        .max_z_speed = 20,
        .max_yaw_dps = 20,
        .slow_down_distance = 80,
    };

    user_task_takeoff_x = Pos16_of_Radar.pos_data.x_x100;
    user_task_takeoff_y = Pos16_of_Radar.pos_data.y_x100;
    user_task_target_yaw = UserTask_GetCurrentYawDeg();

    WayPointMission_SetConfig(config);
    WayPointMission_Reset();
    if(UserTask_LoadTestWayPoints() == 0U)
    {
        UserTask_ResetMission();
        return;
    }

    PointNavigation_Start();
    PointNavigation_SetCmdVelSource(JN_Cmd_vel);
    PointNavigation_SetStableCondition(user_task_stable);
    PointNavigation_SetTarget(user_task_takeoff_x,
                              user_task_takeoff_y,
                              USER_TASK_TAKEOFF_Z_X100,
                              user_task_target_yaw);

    user_task_active = 1U;
    user_task_segment_id = 1U;
    user_task_state = USER_TASK_STATE_TAKEOFF;
    user_task_fc_state.alt_stable = 0U;
    user_task_fc_state.center_stable = 0U;
    user_task_fc_state.yaw_stable = 0U;
    UserTask_PublishZeroVelocity();
}

static void UserTask_StopMission(void)
{
    if(user_task_active == 0U)
    {
        return;
    }

    UserTask_PublishZeroVelocity();
    PointNavigation_Stop();
    UserTask_ResetMission();
}

static void UserTask_RunTakeoff(void)
{
    Radar_Cmd_Vel cmd;

    PointNavigation_SetTarget(user_task_takeoff_x,
                              user_task_takeoff_y,
                              USER_TASK_TAKEOFF_Z_X100,
                              user_task_target_yaw);

    if(WayPointMission_UpdateLineTarget(user_task_takeoff_x,
                                        user_task_takeoff_y,
                                        USER_TASK_TAKEOFF_Z_X100,
                                        user_task_target_yaw,
                                        user_task_segment_id,
                                        &user_task_fc_state,
                                        user_task_stable,
                                        &cmd) != 0U)
    {
        UserTask_PublishVelocity(&cmd);
    }

    if(user_task_fc_state.alt_stable != 0U)
    {
        user_task_state = USER_TASK_STATE_LOAD_POINT;
    }
}

static void UserTask_LoadNextPoint(void)
{
    if(WayPointEmpty() != 0U)
    {
        user_task_state = USER_TASK_STATE_HOLD_LAST;
        return;
    }

    user_task_target_point = WayPointTake();
    user_task_segment_id++;
    user_task_fc_state.alt_stable = 0U;
    user_task_fc_state.center_stable = 0U;
    user_task_fc_state.yaw_stable = 0U;
    user_task_state = USER_TASK_STATE_FLY_LINE;
}

static void UserTask_RunLine(void)
{
    Radar_Cmd_Vel cmd;

    PointNavigation_SetTarget(user_task_target_point.x,
                              user_task_target_point.y,
                              USER_TASK_TAKEOFF_Z_X100,
                              user_task_target_yaw);

    if(WayPointMission_UpdateLineTarget(user_task_target_point.x,
                                        user_task_target_point.y,
                                        USER_TASK_TAKEOFF_Z_X100,
                                        user_task_target_yaw,
                                        user_task_segment_id,
                                        &user_task_fc_state,
                                        user_task_stable,
                                        &cmd) != 0U)
    {
        UserTask_PublishVelocity(&cmd);
    }

    if(user_task_fc_state.alt_stable != 0U &&
       user_task_fc_state.center_stable != 0U &&
       user_task_fc_state.yaw_stable != 0U)
    {
        user_task_state = USER_TASK_STATE_LOAD_POINT;
    }
}

static void UserTask_HoldLastPoint(void)
{
    Radar_Cmd_Vel cmd;

    PointNavigation_SetTarget(user_task_target_point.x,
                              user_task_target_point.y,
                              USER_TASK_TAKEOFF_Z_X100,
                              user_task_target_yaw);

    if(WayPointMission_UpdateLineTarget(user_task_target_point.x,
                                        user_task_target_point.y,
                                        USER_TASK_TAKEOFF_Z_X100,
                                        user_task_target_yaw,
                                        user_task_segment_id,
                                        &user_task_fc_state,
                                        user_task_stable,
                                        &cmd) != 0U)
    {
        UserTask_PublishVelocity(&cmd);
    }
}

void UserTask_Init(void)
{
    WayPointFifoInit();
    WayPointMission_Init();
    UserTask_ResetMission();
    user_task_initialized = 1U;
}

void UserTask_Update(void)
{
    if(user_task_initialized == 0U)
    {
        UserTask_Init();
    }

    if(UserTask_SwitchRequested() == 0U)
    {
        UserTask_StopMission();
        return;
    }

    if(UserTask_SystemReady() == 0U || UserTask_RadarDataHealthy() == 0U)
    {
        UserTask_StopMission();
        return;
    }

    if(user_task_active == 0U)
    {
        UserTask_StartMission();
    }

    PointNavigation_SetCmdVelSource(JN_Cmd_vel);

    switch(user_task_state)
    {
        case USER_TASK_STATE_TAKEOFF:
            UserTask_RunTakeoff();
            break;

        case USER_TASK_STATE_LOAD_POINT:
            UserTask_LoadNextPoint();
            break;

        case USER_TASK_STATE_FLY_LINE:
            UserTask_RunLine();
            break;

        case USER_TASK_STATE_HOLD_LAST:
            UserTask_HoldLastPoint();
            break;

        case USER_TASK_STATE_IDLE:
        default:
            break;
    }
}
