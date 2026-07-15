/**
 * @file    app_imu.h
 * @brief   IMU 应用层接口（ICM-20608 + LIS3MDLRT I2C）
 */
#ifndef _APP_IMU_H_
#define _APP_IMU_H_

#include "FreeRTOS.h"
#include "queue.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_DATA_QUEUE_LEN  1U

typedef struct {
    float roll;
    float pitch;
    float yaw;
    char yawSource;
} IMU_Data_t;

extern QueueHandle_t g_imuDataQueue;

void app_imu_start(void);

#ifdef __cplusplus
}
#endif

#endif