/**
 * @file    app_imu.c
 * @brief   App 层 ICM-20608 采集 + 4 状态 EKF 姿态解算
 *          gz 死区: |gz| < 0.5°/s 强制归零，抑制 Yaw 零偏漂移
 *          陀螺量程: ±250°/s, 灵敏度: 131 LSB/(°/s)
 */

#include "app_imu.h"
#include "dev_imu.h"
#include "port_log.h"
#include "imu_ekf_light.h"
#include "FreeRTOS.h"
#include "task.h"

#define ACCEL_SENSITIVITY  16384.0f
#define GYRO_SENSITIVITY    131.0f
#define SAMPLE_DT           (1.0f / 100.0f)
#define EKF_BETA            0.15f
#define GYRO_Z_DEADBAND     0.5f

void imu_task(void *pvParameters)
{
    (void)pvParameters;

    DevIMU *imu = GetIMU();
    if (!imu) { LOG_ERROR("[IMU] NULL handle\r\n"); vTaskDelete(NULL); return; }
    imu->init(imu);

    uint8_t whoami = imu->whoAmI(imu);
    LOG_INFO("[IMU] WHO_AM_I=0x%02X\r\n", whoami);
    if (whoami != 0xAF) { LOG_ERROR("[IMU] WHO_AM_I fail\r\n"); vTaskDelete(NULL); return; }

    /* 10 秒静止标定 */
    LOG_INFO("[IMU] Calib 10s...\r\n");
    float gxb=0, gyb=0, gzb=0; int cnt=0;
    for (int i=0; i<1000; i++) {
        DevIMU_SensorData_t r;
        if (imu->readSensorData(imu, &r)) { gxb+=r.gyroX; gyb+=r.gyroY; gzb+=r.gyroZ; cnt++; }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    float gx_b=cnt?gxb/cnt/GYRO_SENSITIVITY:0;
    float gy_b=cnt?gyb/cnt/GYRO_SENSITIVITY:0;
    float gz_b=cnt?gzb/cnt/GYRO_SENSITIVITY:0;
    LOG_INFO("[IMU] Bias: %+.3f %+.3f %+.3f\r\n", gx_b, gy_b, gz_b);

    /* 加速度计初始化姿态 */
    DevIMU_SensorData_t init;
    float ax0=0,ay0=0,az0=1;
    for (int i=0;i<100;i++) { if (imu->readSensorData(imu,&init)) {
        ax0=init.accelX/ACCEL_SENSITIVITY; ay0=init.accelY/ACCEL_SENSITIVITY; az0=init.accelZ/ACCEL_SENSITIVITY; break; } vTaskDelay(1); }
    ekf_light_init_from_accel(EKF_BETA, ax0, ay0, az0);

    float offR, offP, offY;
    ekf_light_get_euler(&offR, &offP, &offY);
    LOG_INFO("[IMU] Zero: R=%+.2f P=%+.2f\r\n", offR, offP);

    TickType_t lastPrint = xTaskGetTickCount();

    while (1)
    {
        DevIMU_SensorData_t raw;
        if (!imu->readSensorData(imu, &raw)) continue;

        float gx = raw.gyroX/GYRO_SENSITIVITY - gx_b;
        float gy = raw.gyroY/GYRO_SENSITIVITY - gy_b;
        float gz = raw.gyroZ/GYRO_SENSITIVITY - gz_b;
        float ax = raw.accelX/ACCEL_SENSITIVITY;
        float ay = raw.accelY/ACCEL_SENSITIVITY;
        float az = raw.accelZ/ACCEL_SENSITIVITY;

        /* gz 死区 */
        float gz_f = (fabsf(gz) < GYRO_Z_DEADBAND) ? 0.0f : gz;

        ekf_light_update(gx, gy, gz_f, ax, ay, az, SAMPLE_DT);

        if ((xTaskGetTickCount() - lastPrint) >= pdMS_TO_TICKS(1000)) {
            lastPrint = xTaskGetTickCount();
            float roll, pitch, yaw;
            ekf_light_get_euler(&roll, &pitch, &yaw);
            roll -= offR; pitch -= offP; yaw -= offY;
            LOG_INFO("ROLL:%+7.2f PITCH:%+7.2f YAW:%+7.2f\r\n", roll, pitch, yaw);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}