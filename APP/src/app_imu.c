/**
 * @file    app_imu.c
 * @brief   App 层 ICM-20608 数据采集任务实现
 * @note    四层解耦架构 - App 层
 *          通过 Device 句柄读取 6 轴数据，每 1 秒打印一次原始值
 *          SPI1 互斥锁保证 IMU 与 W25Q64 Flash 不冲突
 */

#include "app_imu.h"
#include "dev_imu.h"
#include "port_imu.h"       /* g_spi1_mutex */
#include "port_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

void imu_task(void *pvParameters)
{
    (void)pvParameters;

    /* 创建 SPI1 互斥锁（IMU 与 Flash 共享总线） */
    g_spi1_mutex = xSemaphoreCreateMutex();
    if (g_spi1_mutex == NULL) {
        LOG_ERROR("[IMU] Failed to create SPI1 mutex!\r\n");
    }

    /* 获取 IMU 设备句柄 */
    DevIMU *imu = GetIMU();
    if (imu == NULL) {
        LOG_ERROR("[IMU] Device handle is NULL!\r\n");
        vTaskDelete(NULL);
        return;
    }

    /* 初始化 IMU 硬件（GPIO + 传感器配置，含串口日志） */
    imu->init(imu);

    /* 验证 WHO_AM_I */
    uint8_t whoami = imu->whoAmI(imu);
    LOG_INFO("[IMU] WHO_AM_I = 0x%02X (expected 0xAF)\r\n", whoami);

    if (whoami != 0xAF) {
        LOG_ERROR("[IMU] WHO_AM_I mismatch! Check wiring.\r\n");
        vTaskDelete(NULL);
        return;
    }
    LOG_INFO("[IMU] ICM-20608 detected OK\r\n");

    /* 主循环：每 1 秒采集并打印 6 轴原始值 */
    while (1)
    {
        DevIMU_SensorData_t data;

        /* 加锁 SPI1，避免与 Flash 冲突 */
        if (g_spi1_mutex && xSemaphoreTake(g_spi1_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            bool ok = imu->readSensorData(imu, &data);
            xSemaphoreGive(g_spi1_mutex);

            if (ok) {
                LOG_INFO("ACCEL: %6d %6d %6d | GYRO: %6d %6d %6d\r\n",
                         data.accelX, data.accelY, data.accelZ,
                         data.gyroX,  data.gyroY,  data.gyroZ);
            } else {
                LOG_ERROR("[IMU] Read sensor data failed!\r\n");
            }
        }

        /* 延时 1 秒 */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}