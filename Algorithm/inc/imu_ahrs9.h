/**
 * @file    imu_ahrs9.h
 * @brief   Lightweight 9-axis Mahony AHRS attitude solver.
 */
#ifndef IMU_AHRS9_H
#define IMU_AHRS9_H

#include <stdbool.h>

typedef struct {
    float kpAcc;
    float kpMag;
    float ki;
} Ahrs9Config;

void ahrs9_init(const Ahrs9Config *cfg);
void ahrs9_update(float gx, float gy, float gz,
                  float ax, float ay, float az,
                  float mx, float my, float mz,
                  float dt);
void ahrs9_get_euler(float *roll, float *pitch, float *yaw);
void ahrs9_reset_yaw(void);

#endif /* IMU_AHRS9_H */
