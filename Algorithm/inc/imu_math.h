/**
 * @file    imu_math.h
 * @brief   四元数/矩阵/向量数学工具（双精度）
 * @note    为 7 状态 EKF 提供底层运算
 *          MSPM0G3507 无 FPU，使用软件 double
 */

#ifndef IMU_MATH_H
#define IMU_MATH_H

#include <stdint.h>
#include <math.h>

/* ================================================================
 *  四元数运算
 * ================================================================ */

/**
 * @brief 四元数乘法: r = p ⊗ q
 * @note  哈密顿约定: q = [qw, qx, qy, qz]
 */
void quat_mul(const double p[4], const double q[4], double r[4]);

/**
 * @brief 四元数归一化: q /= |q|
 */
void quat_normalize(double q[4]);

/**
 * @brief 单位四元数初始化
 */
void quat_identity(double q[4]);

/**
 * @brief 四元数 → 欧拉角 (Roll, Pitch, Yaw) 单位: 度
 * @note  输出顺序: roll[-180,180], pitch[-90,90], yaw[-180,180]
 */
void quat_to_euler(const double q[4], double *roll, double *pitch, double *yaw);

/* ================================================================
 *  向量运算
 * ================================================================ */

/** 向量叉积: c = a × b */
void vec3_cross(const double a[3], const double b[3], double c[3]);

/** 向量点积 */
double vec3_dot(const double a[3], const double b[3]);

/** 向量归一化 */
void vec3_normalize(double v[3]);

/** 向量范数 */
double vec3_norm(const double v[3]);

/** 向量减法: d = a - b */
void vec3_sub(const double a[3], const double b[3], double d[3]);

/* ================================================================
 *  矩阵运算 (对称矩阵优化)
 * ================================================================ */

/**
 * @brief 7×7 对称矩阵乘法: C = A * B (仅上三角 + 对称)
 * @note  A, B 均以完整 49 元素传入
 */
void mat7x7_mul(const double A[49], const double B[49], double C[49]);

/**
 * @brief 7×7 += 外积: C += a * a^T * scalar
 * @note  只更新上三角
 */
void mat7x7_add_outer(double C[49], const double a[7], double scalar);

/**
 * @brief 7×7 = I * value
 */
void mat7x7_identity(double M[49], double value);

/**
 * @brief 7×7 对角线赋值
 */
void mat7x7_diag(double M[49], const double d[7]);

/**
 * @brief 卡尔曼增益计算: K = P * H^T * inv(S)
 *        P: 7×7, H: 3×7, S: 3×3, K: 7×3
 */
void kalman_gain_7x3(const double P[49], const double H[21],
                      const double S[9], double K[21]);

/**
 * @brief 3×3 矩阵求逆: B = inv(A)
 * @return true 成功, false 奇异
 */
bool mat3x3_inv(const double A[9], double B[9]);

/**
 * @brief S = H * P * H^T + R (3×3 = 3×7 * 7×7 * 7×3 + 3×3)
 */
void innovation_cov(const double P[49], const double H[21],
                    const double R[9], double S[9]);

/**
 * @brief 状态更新: x += K * residual (7×1 += 7×3 * 3×1)
 */
void state_update(double x[7], const double K[21], const double residual[3]);

/**
 * @brief 协方差 Joseph 更新: P = (I - KH) * P * (I - KH)^T + K * R * K^T
 *        确保数值对称正定
 */
void cov_joseph_update(double P[49], const double K[21],
                       const double H[21], const double R[9]);

#endif /* IMU_MATH_H */