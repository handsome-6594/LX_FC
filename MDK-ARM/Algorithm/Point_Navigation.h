#ifndef _POINT_NAVIGATION_H
#define _POINT_NAVIGATION_H

#include "SysConfig.h"

typedef enum
{
    Radar_Pid_vel = 0,
    Camera_Pid_vel,
    JN_Cmd_vel
}_cmd_vel_sorce;

typedef struct
{
    u8 alt_stable;
    u8 center_stable;
    u8 yaw_stable;
}FC_TaskState_t;

typedef struct
{
    u8 alt_ok_time;
    u8 alt_error_threshold;
    u8 center_ok_time;
    u8 center_error_threshold;
    u8 yaw_ok_time;
    u8 yaw_error_threshold;
}FC_Stable_t;

typedef struct
{
    s16 target_x;
    s16 target_y;
    s16 target_z;
    s16 target_yaw;
}PointNavigationTarget_t;

extern volatile _cmd_vel_sorce cmd_vel_sorce;
extern volatile u8 point_navigation_enable;
extern FC_TaskState_t point_navigation_state;
extern FC_Stable_t point_navigation_stable;
extern PointNavigationTarget_t point_navigation_target;

void PointNavigation_Init(void);
void PointNavigation_Start(void);
void PointNavigation_Stop(void);
void PointNavigation_SetCmdVelSource(_cmd_vel_sorce source);
void PointNavigation_SetStableCondition(FC_Stable_t stable);
void PointNavigation_SetTarget(s16 target_x, s16 target_y, s16 target_z, s16 target_yaw);
void PointNavigation_TestPointTask(void);
void PointNavigation_Update(void);


#endif
