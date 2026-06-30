/**
 * @file    imu_ekf.h
 * @brief   7 状态四元数扩展卡尔曼滤波器（IMU 姿态估计）
 * @note    状态: [qw, qx, qy, qz, gbx, gby, gbz]
 *          - q: 单位四元数（姿态）
 *          - gb: 陀螺仪零偏 (°/s)
 *
 *          测量: 加速度计（归一化重力向量）
 *          无磁力计 → Yaw 从陀螺积分获得，会漂移
 */

#ifndef IMU_EKF_H
#define IMU_EKF_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/**
 * @brief EKF 初始化和参数设置
 *
 * @param sampleFreq  陀螺采样频率 (Hz), 典型 1000
 * @param accelFreq   加速度计更新频率 (Hz), 典型 100
 *
 * @note  调用后状态初始化为水平面朝上
 */
void imu_ekf_init(double sampleFreq, double accelFreq);

/**
 * @brief 预测步: 陀螺积分（每次有新陀螺数据时调用）
 *
 * @param gx, gy, gz  角速度 (°/s)
 * @param dt          时间步长 (s), 典型 0.001
 *
 * @note  内部自动减去估计零偏: ω = [gx-bx, gy-by, gz-bz]
 */
void imu_ekf_predict(double gx, double gy, double gz, double dt);

/**
 * @brief 更新步: 加速度计修正（每次有新加速计数据时调用）
 *
 * @param ax, ay, az  加速度 (任意单位，内部自动归一化)
 *
 * @note  内部使用自适应噪声矩阵: 静止时高置信，运动时低置信
 */
void imu_ekf_update(double ax, double ay, double az);

/**
 * @brief 获取当前欧拉角
 *
 * @param roll  输出: Roll 角 (°)  [-180, 180]
 * @param pitch 输出: Pitch 角 (°)  [-90, 90]
 * @param yaw   输出: Yaw 角 (°)   [-180, 180]
 */
void imu_ekf_get_euler(double *roll, double *pitch, double *yaw);

/**
 * @brief 获取估计的陀螺零偏 (°/s)
 */
void imu_ekf_get_bias(double *bx, double *by, double *bz);

/**
 * @brief 重置 Yaw 角为 0（校准参考方向）
 */
void imu_ekf_reset_yaw(void);

#endif /* IMU_EKF_H */