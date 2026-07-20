#ifndef WAYPOINT_MISSION_H
#define WAYPOINT_MISSION_H

#include "JetsonNano_Data_Transmit.h"
#include "Point_Navigation.h"
#include "SysConfig.h"

typedef struct
{
    s16 max_xy_speed;
    s16 min_xy_speed;
    s16 max_z_speed;
    s16 max_yaw_dps;
    s16 slow_down_distance;
} WayPointMissionConfig_t;

typedef struct
{
    u8 active;
    u8 line_valid;
    u8 target_reached;
    u8 segment_id;

    s16 start_x;
    s16 start_y;
    s16 target_x;
    s16 target_y;
    s16 target_z;
    s16 target_yaw;

    s16 current_x;
    s16 current_y;
    s16 current_z;

    float line_length;
    float along;
    float cross;
    float remaining;

    s16 radar_vel_x;
    s16 radar_vel_y;
    s16 body_vel_x;
    s16 body_vel_y;
    s16 vel_z;
    s16 yaw_dps;
} WayPointMissionState_t;

void WayPointMission_Init(void);
void WayPointMission_Reset(void);
void WayPointMission_SetConfig(WayPointMissionConfig_t config);
WayPointMissionConfig_t WayPointMission_GetConfig(void);
const WayPointMissionState_t *WayPointMission_GetState(void);

u8 WayPointMission_UpdateLineTarget(s16 target_x,
                                    s16 target_y,
                                    s16 target_z,
                                    s16 target_yaw,
                                    u8 segment_id,
                                    FC_TaskState_t *task_state,
                                    FC_Stable_t stable,
                                    Radar_Cmd_Vel *body_cmd_vel);

#endif
