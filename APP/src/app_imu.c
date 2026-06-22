/**
 * @file    app_imu.c
 * @brief   App 层 ICM-20948 数据采集任务实现
 * @note    四层解耦架构 - App 层
 *          通过 Device 句柄读取 6 轴数据，每 1 秒打印一次原始值
 */

#include "app_imu.h"
#include "dev_imu.h"
#include "port_log.h"
#include "FreeRTOS.h"
#include "task.h"

void imu_task(void *pvParameters)
{
    (void)pvParameters;

    /* 获取 IMU 设备句柄 */
    DevIMU *imu = GetIMU();
    if (imu == NULL) {
        LOG_ERROR("[IMU] Device handle is NULL!\r\n");
        vTaskDelete(NULL);
        return;
    }

    /* 初始化 IMU 硬件（GPIO + 传感器配置） */
    imu->init(imu);

    /* 验证 WHO_AM_I */
    uint8_t whoami = imu->whoAmI(imu);
    LOG_INFO("[IMU] WHO_AM_I = 0x%02X (expected 0xEA)\r\n", whoami);

    if (whoami != 0xEA) {
        LOG_ERROR("[IMU] WHO_AM_I mismatch! Check wiring.\r\n");
        vTaskDelete(NULL);
        return;
    }
    LOG_INFO("[IMU] ICM-20948 detected OK\r\n");

    /* 主循环：每 1 秒采集并打印 6 轴原始值 */
    while (1)
    {
        DevIMU_SensorData_t data;

        if (imu->readSensorData(imu, &data)) {
            LOG_INFO("ACCEL: %6d %6d %6d | GYRO: %6d %6d %6d\r\n",
                     data.accelX, data.accelY, data.accelZ,
                     data.gyroX,  data.gyroY,  data.gyroZ);
        } else {
            LOG_ERROR("[IMU] Read sensor data failed!\r\n");
        }

        /* 延时 1 秒 */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}