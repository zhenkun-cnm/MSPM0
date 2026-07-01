/**
 * @file    app_imu.c
 * @brief   App 层 ICM-20608 采集 + 数据预处理 + 互补滤波姿态解算
 *          角速度: 一阶低通 (α=0.3, com_filter 基准)
 *          加速度: 1D 卡尔曼 (Q=0.001 R=0.543, com_filter 基准)
 *          姿态: 互补滤波 PI + 四元数龙格库塔 (com_imu 基准)
 *          陀螺量程: ±2000°/s, 灵敏度: 16.4 LSB/(°/s)
 */

#include "app_imu.h"
#include "dev_imu.h"
#include "port_log.h"
#include "imu_filter.h"
#include "imu_complementary.h"
#include "FreeRTOS.h"
#include "task.h"

#define ACCEL_SENSITIVITY  16384.0f              /* ±2g → 16384 LSB/g */
#define GYRO_SENSITIVITY    16.4f                /* ±2000°/s → 65536/4000 */
#define SAMPLE_DT           (1.0f / 100.0f)
#define LP_ALPHA            0.3f                 /* 低通系数 */
#define KALMAN_Q            0.001f               /* 卡尔曼过程噪声 */
#define KALMAN_R            0.543f               /* 卡尔曼测量噪声 */
#define KALMAN_P_INIT       0.02f                /* 卡尔曼初始协方差 */

void imu_task(void *pvParameters)
{
    (void)pvParameters;

    DevIMU *imu = GetIMU();
    if (!imu) { LOG_ERROR("[IMU] NULL handle\r\n"); vTaskDelete(NULL); return; }
    imu->init(imu);

    uint8_t whoami = imu->whoAmI(imu);
    LOG_INFO("[IMU] WHO_AM_I=0x%02X\r\n", whoami);
    if (whoami != 0xAF) { LOG_ERROR("[IMU] WHO_AM_I fail\r\n"); vTaskDelete(NULL); return; }

    /* 5 秒静止标定 — 陀螺零偏 */
    LOG_INFO("[IMU] Calib 5s...\r\n");
    float gxb=0, gyb=0, gzb=0; int cnt=0;
    for (int i=0; i<500; i++) {
        DevIMU_SensorData_t r;
        if (imu->readSensorData(imu, &r)) { gxb+=r.gyroX; gyb+=r.gyroY; gzb+=r.gyroZ; cnt++; }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    float gx_b=cnt?gxb/cnt/GYRO_SENSITIVITY:0;
    float gy_b=cnt?gyb/cnt/GYRO_SENSITIVITY:0;
    float gz_b=cnt?gzb/cnt/GYRO_SENSITIVITY:0;
    LOG_INFO("[IMU] Bias: %+.3f %+.3f %+.3f\r\n", gx_b, gy_b, gz_b);

    /* 加速度计初始读数 */
    DevIMU_SensorData_t init;
    float ax0=0,ay0=0,az0=1;
    for (int i=0;i<100;i++) { if (imu->readSensorData(imu,&init)) {
        ax0=init.accelX/ACCEL_SENSITIVITY; ay0=init.accelY/ACCEL_SENSITIVITY; az0=init.accelZ/ACCEL_SENSITIVITY; break; } vTaskDelay(1); }

    /* 初始化滤波器 */
    complementary_init();
    float gx_lp=0, gy_lp=0, gz_lp=0;
    ImuKalmanFilter kf_ax, kf_ay, kf_az;
    imu_kalman_init(&kf_ax, KALMAN_Q, KALMAN_R, KALMAN_P_INIT, ax0);
    imu_kalman_init(&kf_ay, KALMAN_Q, KALMAN_R, KALMAN_P_INIT, ay0);
    imu_kalman_init(&kf_az, KALMAN_Q, KALMAN_R, KALMAN_P_INIT, az0);
    LOG_INFO("[IMU] Filter+Complementary ready\r\n");

    TickType_t lastPrint = xTaskGetTickCount();

    while (1)
    {
        DevIMU_SensorData_t raw;
        if (!imu->readSensorData(imu, &raw)) continue;

        /* 原始数据 */
        float gx_r = raw.gyroX/GYRO_SENSITIVITY - gx_b;
        float gy_r = raw.gyroY/GYRO_SENSITIVITY - gy_b;
        float gz_r = raw.gyroZ/GYRO_SENSITIVITY - gz_b;
        float ax_r = raw.accelX/ACCEL_SENSITIVITY;
        float ay_r = raw.accelY/ACCEL_SENSITIVITY;
        float az_r = raw.accelZ/ACCEL_SENSITIVITY;

        /* 一阶低通: 角速度 */
        gx_lp = imu_lowpass(gx_r, gx_lp, LP_ALPHA);
        gy_lp = imu_lowpass(gy_r, gy_lp, LP_ALPHA);
        gz_lp = imu_lowpass(gz_r, gz_lp, LP_ALPHA);

        /* 1D 卡尔曼: 加速度 */
        float ax_kf = imu_kalman_update(&kf_ax, ax_r);
        float ay_kf = imu_kalman_update(&kf_ay, ay_r);
        float az_kf = imu_kalman_update(&kf_az, az_r);

        /* 互补滤波姿态解算 */
        complementary_update(gx_lp, gy_lp, gz_lp,
                             ax_kf, ay_kf, az_kf, SAMPLE_DT);

        if ((xTaskGetTickCount() - lastPrint) >= pdMS_TO_TICKS(1000)) {
            lastPrint = xTaskGetTickCount();
            float roll, pitch, yaw;
            complementary_get_euler(&roll, &pitch, &yaw);
            LOG_INFO("ROLL:%+7.2f PITCH:%+7.2f YAW:%+7.2f\r\n", roll, pitch, yaw);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}