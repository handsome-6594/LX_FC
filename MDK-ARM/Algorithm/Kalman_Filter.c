#include "Kalman_Filter.h"

#define VELOCITY_KALMAN_MIN_VARIANCE  (1.0e-9f)

static float VelocityKalman_PositiveVariance(float value)
{
    return (value > VELOCITY_KALMAN_MIN_VARIANCE) ?
           value : VELOCITY_KALMAN_MIN_VARIANCE;
}

//更新残差协方差，计算卡尔曼增益，进行后验估计，然后更新误差协方差
static void VelocityKalman_UpdateAxis(VelocityKalmanAxis_t *axis,
                                      float measurement,
                                      float measurement_noise)
{
    float innovation_covariance;
    float kalman_gain;

    innovation_covariance = axis->covariance + measurement_noise;
    kalman_gain = axis->covariance / innovation_covariance;

    axis->velocity += kalman_gain * (measurement - axis->velocity);
    axis->covariance *= (1.0f - kalman_gain);
    axis->covariance = VelocityKalman_PositiveVariance(axis->covariance);
}

//初始化各个参数
void VelocityFusionKalman_Init(VelocityFusionKalman_t *filter,
                               float initial_velocity_x,
                               float initial_velocity_y,
                               float initial_covariance,
                               float process_noise,
                               float radar_noise,
                               float optical_flow_noise)
{
    if(filter == 0)
    {
        return;
    }

    filter->x.velocity = initial_velocity_x;
    filter->y.velocity = initial_velocity_y;
    filter->x.covariance = VelocityKalman_PositiveVariance(initial_covariance);
    filter->y.covariance = filter->x.covariance;

    VelocityFusionKalman_SetNoise(filter,
                                  process_noise,
                                  radar_noise,
                                  optical_flow_noise);
}

void VelocityFusionKalman_SetNoise(VelocityFusionKalman_t *filter,
                                   float process_noise,
                                   float radar_noise,
                                   float optical_flow_noise)
{
    if(filter == 0)
    {
        return;
    }

    filter->process_noise = VelocityKalman_PositiveVariance(process_noise);
    filter->radar_noise = VelocityKalman_PositiveVariance(radar_noise);
    filter->optical_flow_noise = VelocityKalman_PositiveVariance(optical_flow_noise);
}

void VelocityFusionKalman_Update(VelocityFusionKalman_t *filter,
                                 float radar_velocity_x,
                                 float radar_velocity_y,
                                 uint8_t radar_valid,
                                 float optical_flow_velocity_x,
                                 float optical_flow_velocity_y,
                                 uint8_t optical_flow_valid,
                                 float dt_s)
{
    float predicted_variance;

    if(filter == 0)
    {
        return;
    }

    if(dt_s < 0.0f)
    {
        dt_s = 0.0f;
    }

    predicted_variance = filter->process_noise * dt_s;
    filter->x.covariance += predicted_variance;
    filter->y.covariance += predicted_variance;

    if(radar_valid != 0U)
    {
        VelocityKalman_UpdateAxis(&filter->x, radar_velocity_x, filter->radar_noise);
        VelocityKalman_UpdateAxis(&filter->y, radar_velocity_y, filter->radar_noise);
    }

    if(optical_flow_valid != 0U)
    {
        VelocityKalman_UpdateAxis(&filter->x,
                                  optical_flow_velocity_x,
                                  filter->optical_flow_noise);
        VelocityKalman_UpdateAxis(&filter->y,
                                  optical_flow_velocity_y,
                                  filter->optical_flow_noise);
    }
}

uint32_t VelocityFusionKalman_SelfTest(void)
{
    VelocityFusionKalman_t test_filter;
    float previous_velocity;
    float previous_covariance;
    uint32_t result = 0U;

    VelocityFusionKalman_Init(&test_filter,
                              0.0f, 0.0f,
                              100.0f,
                              25.0f,
                              25.0f,
                              100.0f);

    if(test_filter.x.velocity != 0.0f ||
       test_filter.y.velocity != 0.0f ||
       test_filter.x.covariance != 100.0f)
    {
        result |= (1UL << 0);
    }

    /* A reliable radar measurement should move both estimates toward it. */
    VelocityFusionKalman_Update(&test_filter,
                                10.0f, -10.0f, 1U,
                                0.0f, 0.0f, 0U,
                                0.01f);
    if(test_filter.x.velocity < 7.9f || test_filter.x.velocity > 8.1f ||
       test_filter.y.velocity < -8.1f || test_filter.y.velocity > -7.9f)
    {
        result |= (1UL << 1);
    }

    /* With no valid measurement, velocity stays fixed while P increases. */
    previous_velocity = test_filter.x.velocity;
    previous_covariance = test_filter.x.covariance;
    VelocityFusionKalman_Update(&test_filter,
                                0.0f, 0.0f, 0U,
                                0.0f, 0.0f, 0U,
                                0.02f);
    if(test_filter.x.velocity != previous_velocity ||
       test_filter.x.covariance <= previous_covariance)
    {
        result |= (1UL << 2);
    }

    /* Optical flow must also be usable as the only measurement source. */
    VelocityFusionKalman_Init(&test_filter,
                              0.0f, 0.0f,
                              100.0f,
                              25.0f,
                              25.0f,
                              100.0f);
    VelocityFusionKalman_Update(&test_filter,
                                0.0f, 0.0f, 0U,
                                10.0f, -10.0f, 1U,
                                0.01f);
    if(test_filter.x.velocity < 4.9f || test_filter.x.velocity > 5.1f ||
       test_filter.y.velocity < -5.1f || test_filter.y.velocity > -4.9f)
    {
        result |= (1UL << 3);
    }

    return result;
}



