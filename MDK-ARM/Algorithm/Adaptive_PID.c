#include "Adaptive_PID.h"
#include <math.h>


PID_t loc_pid[PID_NUM];

//限幅
static float clampf(float value, float min_value, float max_value)
{
    if(value > max_value) return max_value;
    if(value < min_value) return min_value;
    return value;
}

//计算yaw轴偏差角度
static float angle_error_deg(float target, float measurement)
{
    float error = target - measurement;

    while(error > 180.0f) {
        error -= 360.0f;
    }

    while(error < -180.0f) {
        error += 360.0f;
    }

    return error;
}

//复位pid的积分，测量值，误差什么的
void PID_Reset(PID_t *pid)
{
    if(pid == NULL) return;

    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->prev_error = 0.0f;
    pid->d_filtered = 0.0f;
    pid->last_time_ms = HAL_GetTick();
    pid->error = 0.0f;
}

//设置pid 参数
void PID_Init(void)
{
    loc_pid[PID_X].kp = 0.30f;
    loc_pid[PID_X].ki = 0.00f;
    loc_pid[PID_X].kd = 0.00f;
    loc_pid[PID_X].output_min = -20.0f;
    loc_pid[PID_X].output_max = 20.0f;
    loc_pid[PID_X].integral_min = -0.0f;
    loc_pid[PID_X].integral_max = 0.0f;
    loc_pid[PID_X].deadband = 3.0f;
    loc_pid[PID_X].d_filter_alpha = 0.7f;
    PID_Reset(&loc_pid[PID_X]);

    loc_pid[PID_Y] = loc_pid[PID_X];
    PID_Reset(&loc_pid[PID_Y]);

    loc_pid[PID_Z].kp = 0.30f;
    loc_pid[PID_Z].ki = 0.00f;
    loc_pid[PID_Z].kd = 0.00f;
    loc_pid[PID_Z].output_min = -18.0f;
    loc_pid[PID_Z].output_max = 18.0f;
    loc_pid[PID_Z].integral_min = -0.0f;
    loc_pid[PID_Z].integral_max = 0.0f;
    loc_pid[PID_Z].deadband = 3.0f;
    loc_pid[PID_Z].d_filter_alpha = 0.7f;
    PID_Reset(&loc_pid[PID_Z]);

    loc_pid[PID_YAW].kp = 0.35f;
    loc_pid[PID_YAW].ki = 0.00f;
    loc_pid[PID_YAW].kd = 0.00f;
    loc_pid[PID_YAW].output_min = -5.0f;
    loc_pid[PID_YAW].output_max = 5.0f;
    loc_pid[PID_YAW].integral_min = -5.0f;
    loc_pid[PID_YAW].integral_max = 5.0f;
    loc_pid[PID_YAW].deadband = 1.0f;
    loc_pid[PID_YAW].d_filter_alpha = 0.7f;
    PID_Reset(&loc_pid[PID_YAW]);
}

//核心：pid计算
float PID_Update(PID_t *pid, float target, float measurement)
{
    if(pid == NULL) return 0.0f;

    uint32_t now_ms = HAL_GetTick();
    uint32_t dt_ms = now_ms - pid->last_time_ms;
    pid->last_time_ms = now_ms;

    if(dt_ms == 0) {
        dt_ms = 1;
    }

    float dt = dt_ms * 0.001f;

    float error = target - measurement;

    if(fabsf(error) < pid->deadband) {
        error = 0.0f;
    }

    pid->error = error;

    pid->integral += error * dt;
    pid->integral = clampf(pid->integral,
                           pid->integral_min,
                           pid->integral_max);

    float measurement_rate = (measurement - pid->prev_measurement) / dt;

    pid->d_filtered =
        pid->d_filter_alpha * measurement_rate +
        (1.0f - pid->d_filter_alpha) * pid->d_filtered;

    float output =
        pid->kp * error +
        pid->ki * pid->integral -
        pid->kd * pid->d_filtered;

    output = clampf(output, pid->output_min, pid->output_max);

    pid->prev_measurement = measurement;
    pid->prev_error = error;

    return output;
}

//yaw轴专门弄了个pid
float PID_UpdateYaw(PID_t *pid, float target_deg, float measurement_deg)
{
    if(pid == NULL) return 0.0f;

    uint32_t now_ms = HAL_GetTick();
    uint32_t dt_ms = now_ms - pid->last_time_ms;
    pid->last_time_ms = now_ms;

    if(dt_ms == 0) {
        dt_ms = 1;
    }

    float dt = dt_ms * 0.001f;

    float error = angle_error_deg(target_deg, measurement_deg);

    if(fabsf(error) < pid->deadband) {
        error = 0.0f;
    }

    pid->error = error;

    pid->integral += error * dt;
    pid->integral = clampf(pid->integral,
                           pid->integral_min,
                           pid->integral_max);

    float error_rate = (error - pid->prev_error) / dt;

    pid->d_filtered =
        pid->d_filter_alpha * error_rate +
        (1.0f - pid->d_filter_alpha) * pid->d_filtered;

    float output =
        pid->kp * error +
        pid->ki * pid->integral +
        pid->kd * pid->d_filtered;

    output = clampf(output, pid->output_min, pid->output_max);

    pid->prev_error = error;
    pid->prev_measurement = measurement_deg;

    return output;
}


