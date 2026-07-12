#ifndef _ADAPTIVE_PID_H
#define _ADAPTIVE_PID_H

#include "main.h"

typedef enum
{
    PID_X = 0,
    PID_Y,
    PID_Z,
    PID_YAW,
    PID_NUM
} PID_Axis_e;

typedef struct
{
    float kp;
    float ki;
    float kd;

    float integral;
    float prev_measurement;
    float prev_error;

    float output_min;
    float output_max;
    float integral_min;
    float integral_max;

    float deadband;
    float d_filter_alpha;
    float d_filtered;

    uint32_t last_time_ms;
    float error;
} PID_t;

extern PID_t loc_pid[PID_NUM];

void PID_Init(void);
void PID_Reset(PID_t *pid);
float PID_Update(PID_t *pid, float target, float measurement);
float PID_UpdateYaw(PID_t *pid, float target_deg, float measurement_deg);




#endif

