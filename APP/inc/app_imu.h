/**
 * @file    app_imu.h
 * @brief   IMU 应用层接口（ICM-20608 + LIS3MDLRT I2C）
 */
#ifndef _APP_IMU_H_
#define _APP_IMU_H_

#include "FreeRTOS.h"
#include "semphr.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float gyro_z_dps;
    bool gyro_z_valid;
    char yawSource;
} IMU_Data_t;

typedef struct {
    IMU_Data_t data;
    SemaphoreHandle_t lock;
} IMU_DataGlobal_t;

/* IMU startup lifecycle; READY is set after gyro/magnetometer calibration and AHRS setup. */
typedef enum {
    IMU_INIT_PENDING = 0,
    IMU_INIT_READY,
    IMU_INIT_FAILED
} IMU_InitStatus_t;

extern IMU_DataGlobal_t g_imuDataGlobal;

bool IMU_Data_Read(IMU_Data_t *out);
void IMU_Data_Write(const IMU_Data_t *in);
IMU_InitStatus_t IMU_InitStatus_Get(void);

void app_imu_start(void);

#ifdef __cplusplus
}
#endif

#endif
