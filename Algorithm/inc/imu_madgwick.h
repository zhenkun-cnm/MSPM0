/**
 * @file    imu_madgwick.h
 * @brief   Madgwick AHRS 6 轴姿态解算（无磁力计）
 * @note    Cortex-M0+ 优化版，全 float 运算
 *          Sebastian Madgwick, 2010 — 梯度下降四元数融合
 *
 *          手册 β 值: ICM-20608 陀螺噪声 0.008°/s/√Hz → β ≈ 0.03~0.05
 */

#ifndef IMU_MADGWICK_H
#define IMU_MADGWICK_H

#include <stdint.h>

/**
 * @brief 初始化 Madgwick 滤波器
 * @param sampleFreq  采样频率 (Hz), 典型 1000
 * @param beta        收敛系数, 典型 0.04
 */
void madgwick_init(float sampleFreq, float beta);

/**
 * @brief Madgwick 更新步（陀螺 + 加速计）
 *
 * @param gx, gy, gz  角速度 (°/s)
 * @param ax, ay, az  加速度 (g)
 * @param dt          时间步长 (s)
 *
 * @note  调用后内部四元数更新。
 *        每次有新 IMU 数据时调用一次即可（包含 predict + update）。
 */
void madgwick_update(float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float dt);

/**
 * @brief 获取欧拉角
 *
 * @param roll  输出 Roll  (°) [-180, 180]
 * @param pitch 输出 Pitch (°) [-90, 90]
 * @param yaw   输出 Yaw   (°) [-180, 180]
 */
void madgwick_get_euler(float *roll, float *pitch, float *yaw);

/**
 * @brief 重置 Yaw 为 0
 */
void madgwick_reset_yaw(void);

#endif /* IMU_MADGWICK_H */