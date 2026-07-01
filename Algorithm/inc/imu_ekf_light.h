/**
 * @file    imu_ekf_light.h
 * @brief   4 状态四元数 EKF（M0+ float 优化版）
 * @note    状态: q = [q0, q1, q2, q3]
 *          预测: 四元数陀螺积分
 *          更新: 顺序标量卡尔曼（3 次 1D 更新，无矩阵求逆）
 *          零偏: 启动时 5 秒标定 + Q 矩阵吸收残差
 */

#ifndef IMU_EKF_LIGHT_H
#define IMU_EKF_LIGHT_H

#include <stdint.h>

/**
 * @brief 初始化 EKF（四元数 = 水平）
 * @param beta  加速度计融合强度 (0.01~0.1, 典型 0.10)
 */
void ekf_light_init(float beta);

/**
 * @brief 初始化 EKF，用加速度计设置初始姿态
 * @param beta  融合强度
 * @param ax, ay, az  归一化加速度 (g)，用于计算初始 Roll/Pitch
 */
void ekf_light_init_from_accel(float beta, float ax, float ay, float az);

/**
 * @brief EKF 预测+更新（每次 IMU 数据调用一次）
 *
 * @param gx,gy,gz  角速度 (°/s)
 * @param ax,ay,az  加速度 (g)
 * @param dt        时间步长 (s)
 */
void ekf_light_update(float gx, float gy, float gz,
                      float ax, float ay, float az,
                      float dt);

/**
 * @brief 获取欧拉角 (°)
 */
void ekf_light_get_euler(float *roll, float *pitch, float *yaw);

#endif /* IMU_EKF_LIGHT_H */