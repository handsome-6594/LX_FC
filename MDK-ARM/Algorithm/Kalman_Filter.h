#ifndef _KALMAN_FILTER_H
#define _KALMAN_FILTER_H

#include <stdint.h>

/* One-dimensional state used independently for the body X and Y velocities. */
typedef struct
{
    float velocity;
    float covariance;
} VelocityKalmanAxis_t;

typedef struct
{
    VelocityKalmanAxis_t x;
    VelocityKalmanAxis_t y;

    /* All noise values are variances, not standard deviations. */
    float process_noise;       /* Velocity variance added per second. */
    float radar_noise;         /* Radar measurement variance. */
    float optical_flow_noise;  /* Optical-flow measurement variance. */
} VelocityFusionKalman_t;

/*
 * Initialize a horizontal velocity fusion filter.
 * initial_covariance should normally be larger when the initial velocity is
 * poorly known. Noise arguments must be greater than zero.
 */
void VelocityFusionKalman_Init(VelocityFusionKalman_t *filter,
                               float initial_velocity_x,
                               float initial_velocity_y,
                               float initial_covariance,
                               float process_noise,
                               float radar_noise,
                               float optical_flow_noise);

/* Change sensor tuning without discarding the current velocity estimate. */
void VelocityFusionKalman_SetNoise(VelocityFusionKalman_t *filter,
                                   float process_noise,
                                   float radar_noise,
                                   float optical_flow_noise);

/*
 * Perform one predict/update cycle. All velocities must use the same unit and
 * coordinate frame. A validity flag of zero skips that sensor for this cycle.
 */
void VelocityFusionKalman_Update(VelocityFusionKalman_t *filter,
                                 float radar_velocity_x,
                                 float radar_velocity_y,
                                 uint8_t radar_valid,
                                 float optical_flow_velocity_x,
                                 float optical_flow_velocity_y,
                                 uint8_t optical_flow_valid,
                                 float dt_s);

/* Returns zero when all deterministic filter checks pass; each set bit
 * identifies a failed check. This function has no hardware side effects. */
uint32_t VelocityFusionKalman_SelfTest(void);




#endif


