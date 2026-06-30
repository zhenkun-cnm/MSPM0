/**
 * @file    app_imu.c
 * @brief   App 层 ICM-20608 数据采集 + 7 状态 EKF 姿态解算
 * @note    100Hz 预测+更新循环，每秒输出欧拉角 + 零偏
 *
 *          EKF 状态: [qw, qx, qy, qz, gbx, gby, gbz]
 *          预测: 陀螺积分（每次读数据后）
 *          更新: 加速度计重力向量修正
 *
 *          SPI1 互斥锁保证与 W25Q64 Flash 不冲突
 */

#include "app_imu.h"
#include "dev_imu.h"
#include "port_imu.h"       /* g_spi1_mutex */
#include "port_log.h"
#include "imu_ekf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* ================================================================
 *  EKF 参数
 * ================================================================ */

#define EKF_GYRO_FREQ   1000.0   /* 陀螺采样频率 (Hz) — ICM-20608 ODR */
#define EKF_ACCEL_FREQ  100.0    /* 加速度计更新频率 (Hz) */
#define EKF_DT          (1.0 / EKF_GYRO_FREQ)  /* 时间步长 0.001s */

/* ================================================================
 *  刻度因子（ICM-20608 手册 Table 1 p.7, Table 2 p.8）
 * ================================================================ */

#define ACCEL_SENSITIVITY  16384.0   /* ±2g: 16384 LSB/g */
#define GYRO_SENSITIVITY   131.0     /* ±250°/s: 131 LSB/(°/s) */

/* ================================================================
 *  任务实现
 * ================================================================ */

void imu_task(void *pvParameters)
{
    (void)pvParameters;
    int printCounter = 0;

    /* 创建 SPI1 互斥锁 */
    g_spi1_mutex = xSemaphoreCreateMutex();

    /* 获取 IMU 设备句柄 */
    DevIMU *imu = GetIMU();
    if (imu == NULL) {
        LOG_ERROR("[IMU] Device handle is NULL!\r\n");
        vTaskDelete(NULL);
        return;
    }

    /* 初始化 IMU 硬件 */
    imu->init(imu);

    /* 验证 WHO_AM_I */
    uint8_t whoami = imu->whoAmI(imu);
    LOG_INFO("[IMU] WHO_AM_I = 0x%02X (expected 0xAF)\r\n", whoami);
    if (whoami != 0xAF) {
        LOG_ERROR("[IMU] WHO_AM_I mismatch!\r\n");
        vTaskDelete(NULL);
        return;
    }
    LOG_INFO("[IMU] ICM-20608 OK, starting EKF...\r\n");

    /* 初始化 EKF */
    imu_ekf_init(EKF_GYRO_FREQ, EKF_ACCEL_FREQ);

    /* 主循环: ~100Hz 采集 + EKF 解算 */
    while (1)
    {
        DevIMU_SensorData_t raw;

        /* 加锁 SPI1，避免与 Flash 冲突 */
        if (g_spi1_mutex == NULL ||
            xSemaphoreTake(g_spi1_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        bool ok = imu->readSensorData(imu, &raw);
        xSemaphoreGive(g_spi1_mutex);

        if (!ok) {
            LOG_ERROR("[IMU] Read failed\r\n");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* 转换为物理单位 */
        double gx = (double)raw.gyroX  / GYRO_SENSITIVITY;   /* °/s */
        double gy = (double)raw.gyroY  / GYRO_SENSITIVITY;
        double gz = (double)raw.gyroZ  / GYRO_SENSITIVITY;

        double ax = (double)raw.accelX / ACCEL_SENSITIVITY;  /* g */
        double ay = (double)raw.accelY / ACCEL_SENSITIVITY;
        double az = (double)raw.accelZ / ACCEL_SENSITIVITY;

        /* EKF 预测（每次） + 更新（每 10 次 = 100Hz） */
        imu_ekf_predict(gx, gy, gz, EKF_DT);

        static int accelCounter = 0;
        accelCounter++;
        if (accelCounter >= ((int)(EKF_GYRO_FREQ / EKF_ACCEL_FREQ))) {
            accelCounter = 0;
            imu_ekf_update(ax, ay, az);
        }

        /* 每约 1 秒打印一次欧拉角 + 零偏 */
        printCounter++;
        if (printCounter >= (int)EKF_GYRO_FREQ) {
            printCounter = 0;

            double roll, pitch, yaw;
            double bx, by, bz;
            imu_ekf_get_euler(&roll, &pitch, &yaw);
            imu_ekf_get_bias(&bx, &by, &bz);

            LOG_INFO("ROLL:%+7.2f PITCH:%+7.2f YAW:%+7.2f"
                     " | bias:%+6.3f %+6.3f %+6.3f °/s\r\n",
                     roll, pitch, yaw, bx, by, bz);
        }

        /* 延时约 1ms，保持 ~1kHz 循环速率 */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}