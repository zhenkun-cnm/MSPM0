/**
 * @file    app_imu.h
 * @brief   IMU 应用层接口（ICM-20608 + LIS3MDLRT I2C）
 */
#ifndef _APP_IMU_H_
#define _APP_IMU_H_

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"

/**
 * @brief IMU 姿态数据快照（imu_task → tft_task 队列传输）
 */
typedef struct {
    float roll, pitch, yaw;   /* 欧拉角 (°) */
    char  yawSource;          /* 'M'=磁力计, 'B'=偏置锁定, 'G'=仅陀螺 */
} IMU_Data_t;

/* IMU 数据队列（imu_task → tft_task 单向推送，xQueueOverwrite 保证最新） */
extern QueueHandle_t g_imuDataQueue;

/* 队列深度 */
#define IMU_DATA_QUEUE_LEN      1U

void app_imu_start(void);

#endif
