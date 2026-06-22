/**
 * @file    app_imu.h
 * @brief   App 层 IMU 数据采集任务声明
 * @note    四层解耦架构 - App 层
 *          通过 Device 句柄读取 ICM-20948 6 轴原始数据
 *          1 秒打印一次
 */

#ifndef APP_IMU_H
#define APP_IMU_H

/**
 * @brief IMU 数据采集任务
 * @param pvParameters  传入参数
 * @note  FreeRTOS 任务函数，由 start_task 创建
 *        初始化验证 WHO_AM_I，每 1s 打印 6 轴原始值
 */
void imu_task(void *pvParameters);

#endif /* APP_IMU_H */