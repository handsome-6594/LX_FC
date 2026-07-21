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
#define USER_TASK_TAKEOFF_Z_X100         50
#define USER_TASK_LAND_APPROACH_Z_X100   25
#define USER_TASK_LAND_DESCEND_SPEED     (-6)
#define USER_TASK_LAND_COMPLETE_Z_X100   8
#define USER_TASK_LAND_COMPLETE_COUNT    20U
#define USER_TASK_TEST_POINT_COUNT       1U
#define USER_TASK_GROUND_TEST_X_X100     50
#define USER_TASK_DEBUG_ENABLE           (0U)
#define USER_TASK_DEBUG_PERIOD_MS        200U
#define USER_TASK_SWITCH_STOP_COUNT      10U
#define USER_TASK_RADAR_STOP_COUNT       20U

typedef enum
{
    USER_TASK_STATE_IDLE = 0,
    USER_TASK_STATE_TAKEOFF,
    USER_TASK_STATE_LOAD_POINT,
    USER_TASK_STATE_FLY_LINE,
    USER_TASK_STATE_RETURN_HOME,
    USER_TASK_STATE_LAND_APPROACH,
    USER_TASK_STATE_LAND_DESCEND,
    USER_TASK_STATE_LANDED,
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
static u8 user_task_land_low_count;
static u8 user_task_last_land_pos_update_cnt;
static u8 user_task_switch_bad_count;
static u8 user_task_radar_bad_count;

static const char *UserTask_StateName(UserTaskState_e state)
{
    switch(state)
    {
        case USER_TASK_STATE_IDLE:
            return "IDLE";

        case USER_TASK_STATE_TAKEOFF:
            return "TAKEOFF";

        case USER_TASK_STATE_LOAD_POINT:
            return "LOAD_POINT";

        case USER_TASK_STATE_FLY_LINE:
            return "FLY_LINE";

        case USER_TASK_STATE_RETURN_HOME:
            return "RETURN_HOME";

        case USER_TASK_STATE_LAND_APPROACH:
            return "LAND_APPROACH";

        case USER_TASK_STATE_LAND_DESCEND:
            return "LAND_DESCEND";

        case USER_TASK_STATE_LANDED:
            return "LANDED";

        case USER_TASK_STATE_HOLD_LAST:
            return "HOLD_LAST";

        default:
            return "UNKNOWN";
    }
}

static void UserTask_SetState(UserTaskState_e next_state, const char *reason)
{
#if USER_TASK_DEBUG_ENABLE
    if(user_task_state != next_state)
    {
        printf("wp_evt %s->%s seg=%u p[%d,%d,%d] home[%d,%d] sw[%u,%u,%u] sys[%u,%u,%u] bad[%u,%u] land=%u reason=%s\r\n",
               UserTask_StateName(user_task_state),
               UserTask_StateName(next_state),
               user_task_segment_id,
               Pos16_of_Radar.pos_data.x_x100,
               Pos16_of_Radar.pos_data.y_x100,
               Pos16_of_Radar.pos_data.z_x100,
               user_task_takeoff_x,
               user_task_takeoff_y,
               Switch_sta_st.SWC,
               Switch_sta_st.SWD,
               Switch_sta_st.SWB,
               RemoteControl_IsSignalLost(),
               state.is_unlocked,
               RC_MotorIsUnlocked(),
               user_task_switch_bad_count,
               user_task_radar_bad_count,
               user_task_land_low_count,
               (reason != 0) ? reason : "");
    }
#else
    (void)reason;
#endif

    user_task_state = next_state;
}

static u8 UserTask_SwitchRequested(void)
{
    return (Switch_sta_st.SWC == Switch_Mid &&
            Switch_sta_st.SWD == Switch_High &&
            Switch_sta_st.SWB == Switch_Low) ? 1U : 0U;
}

static const char *UserTask_NotReadyReason(void)
{
#if USER_TASK_GROUND_TEST_ENABLE
    if(RemoteControl_IsSignalLost() != 0U)
    {
        return "rc_lost";
    }
#else
    if(RemoteControl_IsSignalLost() != 0U)
    {
        return "rc_lost";
    }

    if(state.is_unlocked == 0U)
    {
        return "fc_locked";
    }

    if(RC_MotorIsUnlocked() == 0U)
    {
        return "motor_locked";
    }
#endif

    return 0;
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
            printf("wp_test st=%s(%u) seg=%u p[%d,%d,%d] t[%d,%d,%d] line[%d,%d,%d] pid_r[%d,%d] body[%d,%d] vz=%d yaw=%d ok[%u,%u,%u] land=%u\r\n",
                   UserTask_StateName(user_task_state),
                   user_task_state,
                   user_task_segment_id,
                   line_state->current_x,
                   line_state->current_y,
                   line_state->current_z,
                   line_state->target_x,
                   line_state->target_y,
                   line_state->target_z,
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
                   user_task_fc_state.yaw_stable,
                   user_task_land_low_count);
        }
    }
#endif
}

static void UserTask_PublishZeroVelocity(void)
{
    Radar_Cmd_Vel cmd = {0};

    UserTask_PublishVelocity(&cmd);
}

static void UserTask_ClearStableState(void)
{
    user_task_fc_state.alt_stable = 0U;
    user_task_fc_state.center_stable = 0U;
    user_task_fc_state.yaw_stable = 0U;
}

static void UserTask_ResetLandDetect(void)
{
    user_task_land_low_count = 0U;
    user_task_last_land_pos_update_cnt = radar_pos_update_cnt;
}

static void UserTask_ClearStopDetect(void)
{
    user_task_switch_bad_count = 0U;
    user_task_radar_bad_count = 0U;
}

static u8 UserTask_LandHeightReached(void)
{
    s16 z_x100;

    if(user_task_last_land_pos_update_cnt == radar_pos_update_cnt)
    {
        return 0U;
    }

    user_task_last_land_pos_update_cnt = radar_pos_update_cnt;
    z_x100 = Pos16_of_Radar.pos_data.z_x100;

    if(z_x100 <= USER_TASK_LAND_COMPLETE_Z_X100)
    {
        if(user_task_land_low_count < USER_TASK_LAND_COMPLETE_COUNT)
        {
            user_task_land_low_count++;
        }
    }
    else
    {
        user_task_land_low_count = 0U;
    }

    return (user_task_land_low_count >= USER_TASK_LAND_COMPLETE_COUNT) ? 1U : 0U;
}

static u8 UserTask_LoadTestWayPoints(void)
{
    Point_t test_points[USER_TASK_TEST_POINT_COUNT];
    u8 count = 0U;
#if USER_TASK_GROUND_TEST_ENABLE == 0U
    u8 i;
#endif

    WayPointClear();
#if USER_TASK_GROUND_TEST_ENABLE
    test_points[0].x = (s16)(user_task_takeoff_x + USER_TASK_GROUND_TEST_X_X100);
    test_points[0].y = user_task_takeoff_y;
    test_points[1].x = (s16)(user_task_takeoff_x - USER_TASK_GROUND_TEST_X_X100);
    test_points[1].y = user_task_takeoff_y;
    count = USER_TASK_TEST_POINT_COUNT;
#else
    for(i = 0U; i < USER_TASK_TEST_POINT_COUNT; i++)
    {
        if(MapPoint_IsValid(All_Point[i]) != 0U)
        {
            test_points[count] = All_Point[i];
            count++;
        }
    }
#endif

    if(count == 0U)
    {
        return 0U;
    }

    return Add_WayPoint(test_points, count) ? 1U : 0U;
}

static void UserTask_ResetMission(const char *reason)
{
    UserTask_SetState(USER_TASK_STATE_IDLE, reason);
    user_task_active = 0U;
    user_task_segment_id = 0U;
    user_task_target_point.x = 0;
    user_task_target_point.y = 0;
    UserTask_ClearStableState();
    UserTask_ResetLandDetect();
    UserTask_ClearStopDetect();
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
        UserTask_ResetMission("no_waypoint");
        return;
    }

#if USER_TASK_GROUND_TEST_ENABLE
    PointNavigation_Stop();
#else
    PointNavigation_Start();
#endif
    PointNavigation_SetCmdVelSource(JN_Cmd_vel);
    PointNavigation_SetStableCondition(user_task_stable);
    PointNavigation_SetTarget(user_task_takeoff_x,
                              user_task_takeoff_y,
                              USER_TASK_TAKEOFF_Z_X100,
                              user_task_target_yaw);

    user_task_active = 1U;
    user_task_segment_id = 1U;
    UserTask_SetState(USER_TASK_STATE_TAKEOFF, "start");
    UserTask_ClearStableState();
    UserTask_ResetLandDetect();
    UserTask_ClearStopDetect();
    UserTask_PublishZeroVelocity();

#if USER_TASK_DEBUG_ENABLE
    printf("wp_evt start home[%d,%d] z=%d points=%u land_approach=%d land_done_z=%d descend=%d\r\n",
           user_task_takeoff_x,
           user_task_takeoff_y,
           USER_TASK_TAKEOFF_Z_X100,
           USER_TASK_TEST_POINT_COUNT,
           USER_TASK_LAND_APPROACH_Z_X100,
           USER_TASK_LAND_COMPLETE_Z_X100,
           USER_TASK_LAND_DESCEND_SPEED);
#endif
}

static void UserTask_StopMission(const char *reason)
{
    if(user_task_active == 0U)
    {
        return;
    }

    UserTask_PublishZeroVelocity();
    PointNavigation_Stop();
    UserTask_ResetMission(reason);
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
        UserTask_SetState(USER_TASK_STATE_LOAD_POINT, "takeoff_ok");
    }
}

static void UserTask_LoadNextPoint(void)
{
    if(WayPointEmpty() != 0U)
    {
        user_task_segment_id++;
        UserTask_ClearStableState();
        UserTask_SetState(USER_TASK_STATE_RETURN_HOME, "waypoint_empty");
        return;
    }

    user_task_target_point = WayPointTake();
    user_task_segment_id++;
    UserTask_ClearStableState();
    UserTask_SetState(USER_TASK_STATE_FLY_LINE, "load_point");
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
        UserTask_SetState(USER_TASK_STATE_LOAD_POINT, "line_ok");
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

static void UserTask_RunReturnHome(void)
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

    if(user_task_fc_state.alt_stable != 0U &&
       user_task_fc_state.center_stable != 0U &&
       user_task_fc_state.yaw_stable != 0U)
    {
        user_task_segment_id++;
        UserTask_ClearStableState();
        UserTask_SetState(USER_TASK_STATE_LAND_APPROACH, "home_ok");
    }
}

static void UserTask_RunLandApproach(void)
{
    Radar_Cmd_Vel cmd;

    PointNavigation_SetTarget(user_task_takeoff_x,
                              user_task_takeoff_y,
                              USER_TASK_LAND_APPROACH_Z_X100,
                              user_task_target_yaw);

    if(WayPointMission_UpdateLineTarget(user_task_takeoff_x,
                                        user_task_takeoff_y,
                                        USER_TASK_LAND_APPROACH_Z_X100,
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
        user_task_segment_id++;
        UserTask_ClearStableState();
        UserTask_ResetLandDetect();
        UserTask_SetState(USER_TASK_STATE_LAND_DESCEND, "land_approach_ok");
    }
}

static void UserTask_RunLandDescend(void)
{
    Radar_Cmd_Vel cmd;

    PointNavigation_SetTarget(user_task_takeoff_x,
                              user_task_takeoff_y,
                              USER_TASK_LAND_APPROACH_Z_X100,
                              user_task_target_yaw);

    if(WayPointMission_UpdateLineTarget(user_task_takeoff_x,
                                        user_task_takeoff_y,
                                        USER_TASK_LAND_APPROACH_Z_X100,
                                        user_task_target_yaw,
                                        user_task_segment_id,
                                        &user_task_fc_state,
                                        user_task_stable,
                                        &cmd) != 0U)
    {
        cmd.vz_x100 = USER_TASK_LAND_DESCEND_SPEED;
        UserTask_PublishVelocity(&cmd);
    }

    if(UserTask_LandHeightReached() != 0U)
    {
        UserTask_PublishZeroVelocity();
        PointNavigation_Stop();
        RC_MotorForceLock();
        UserTask_SetState(USER_TASK_STATE_LANDED, "land_height_ok");
    }
}

static void UserTask_RunLanded(void)
{
    UserTask_PublishZeroVelocity();
    PointNavigation_Stop();
    RC_MotorForceLock();
}

void UserTask_Init(void)
{
    WayPointFifoInit();
    WayPointMission_Init();
    UserTask_ResetMission("init");
    user_task_initialized = 1U;
}

void UserTask_Update(void)
{
    const char *not_ready_reason;

    if(user_task_initialized == 0U)
    {
        UserTask_Init();
    }

    if(UserTask_SwitchRequested() == 0U)
    {
        if(user_task_active == 0U)
        {
            return;
        }

        if(user_task_switch_bad_count < USER_TASK_SWITCH_STOP_COUNT)
        {
            user_task_switch_bad_count++;
        }

        if(user_task_switch_bad_count >= USER_TASK_SWITCH_STOP_COUNT)
        {
            UserTask_StopMission("switch_off");
            return;
        }
    }
    else
    {
        user_task_switch_bad_count = 0U;
    }

#if USER_TASK_GROUND_TEST_ENABLE
    RC_MotorForceLock();
#endif

    not_ready_reason = UserTask_NotReadyReason();
    if(not_ready_reason != 0)
    {
        UserTask_StopMission(not_ready_reason);
        return;
    }

    if(UserTask_RadarDataHealthy() == 0U)
    {
        if(user_task_active == 0U)
        {
            return;
        }

        if(user_task_radar_bad_count < USER_TASK_RADAR_STOP_COUNT)
        {
            user_task_radar_bad_count++;
        }

        if(user_task_radar_bad_count >= USER_TASK_RADAR_STOP_COUNT)
        {
            UserTask_StopMission("radar_unhealthy");
            return;
        }
    }
    else
    {
        user_task_radar_bad_count = 0U;
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

        case USER_TASK_STATE_RETURN_HOME:
            UserTask_RunReturnHome();
            break;

        case USER_TASK_STATE_LAND_APPROACH:
            UserTask_RunLandApproach();
            break;

        case USER_TASK_STATE_LAND_DESCEND:
            UserTask_RunLandDescend();
            break;

        case USER_TASK_STATE_LANDED:
            UserTask_RunLanded();
            break;

        case USER_TASK_STATE_HOLD_LAST:
            UserTask_HoldLastPoint();
            break;

        case USER_TASK_STATE_IDLE:
        default:
            break;
    }
}
