/**
 * @file    imu_madgwick.c
 * @brief   Madgwick AHRS 6 轴姿态解算实现（float, M0+ 优化版）
 */

#include "imu_madgwick.h"
#include <math.h>

/* ================================================================
 *  内部状态
 * ================================================================ */

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  /* 单位四元数 */
static float beta_ = 0.04f;   /* 收敛系数 */
static float sampleFreq_;

/* ================================================================
 *  辅助函数
 * ================================================================ */

/* 快速 1/sqrt(x) — M0+ 无 FPU 时的轻量替代 */
static float inv_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));  /* 一次 Newton 迭代 */
    return y;
}

#define M_PI_F 3.14159265358979323846f

/* ================================================================
 *  初始化
 * ================================================================ */

void madgwick_init(float sampleFreq, float beta)
{
    sampleFreq_ = sampleFreq;
    beta_ = beta;
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
}

/* ================================================================
 *  核心更新
 *
 *  Madgwick 算法（6 轴，无磁力计）:
 *
 *  1. 梯度: ▽f = J^T · f(q, a)
 *     f = [2(q1·q3 - q0·q2) - ax,
 *          2(q0·q1 + q2·q3) - ay,
 *          2(0.5 - q1² - q2²) - az]
 *
 *  2. 融合: q̇ = ½ q ⊗ ω - β · ▽f/|▽f|
 *
 *  3. 积分: q = q + q̇ · dt
 *
 *  4. 归一化: q /= |q|
 * ================================================================ */

void madgwick_update(float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;

    /* 归一化加速度 */
    recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
    if (recipNorm == 0.0f) return;
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    /* 角速度 (deg/s → rad/s) */
    float wx = gx * (M_PI_F / 180.0f);
    float wy = gy * (M_PI_F / 180.0f);
    float wz = gz * (M_PI_F / 180.0f);

    /* ====== 梯度下降 ====== */

    /* 目标函数 f(q, a) — 重力在体坐标系下的理论值 */
    float f_1 = 2.0f * (q1 * q3 - q0 * q2) - ax;
    float f_2 = 2.0f * (q0 * q1 + q2 * q3) - ay;
    float f_3 = 2.0f * (0.5f - q1 * q1 - q2 * q2) - az;

    /* 雅可比 J 的转置 J^T (4×3) × f (3×1) → ▽f (4×1)
     *
     * J = [ -2q2  2q3  -2q0  2q1   ← ∂ax/∂q
     *        2q1  2q0   2q3  2q2   ← ∂ay/∂q
     *        0   -4q1  -4q2  0  ]  ← ∂az/∂q
     *
     * J^T * f (仅上三角，直接算):
     */
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _4q1 = 4.0f * q1;
    float _4q2 = 4.0f * q2;

    /* J^T * f 的 4 个分量 */
    s0 = _2q2 * f_2 - _2q3 * f_1;                    /* -2q2*f1 + 2q1*f2 + 0*f3? 不对，重新推算...
                                                          J^T[0] = [-2q2, 2q1, 0]·f = -2q2*f1 + 2q1*f2 */
    s1 = _2q3 * f_2 + _2q2 * f_1 - _4q1 * f_3;       /* J^T[1] = [2q3, 2q0, -4q1]·f 错。重新:
                                                          J^T 行1 = [2q3,  2q0, -4q1] → J[0][1]=2q3, J[1][1]=2q0, J[2][1]=-4q1
                                                          Wait, let me recompute J properly.
                                                          
                                                          f = [2(q1·q3 - q0·q2), 2(q0·q1 + q2·q3), 2(½ - q1² - q2²)] - a_n
                                                          
                                                          ∂fx/∂q0 = -2q2, ∂fx/∂q1 = 2q3, ∂fx/∂q2 = -2q0, ∂fx/∂q3 = 2q1
                                                          ∂fy/∂q0 = 2q1,  ∂fy/∂q1 = 2q0, ∂fy/∂q2 = 2q3,  ∂fy/∂q3 = 2q2
                                                          ∂fz/∂q0 = 0,    ∂fz/∂q1 = -4q1, ∂fz/∂q2 = -4q2, ∂fz/∂q3 = 0
                                                          
                                                          J^T * f = [∂fx/∂q0 ∂fy/∂q0 ∂fz/∂q0; ... ]^T * [f1;f2;f3]
                                                                  = [(-2q2)·f1 + 2q1·f2 + 0·f3,
                                                                     (2q3)·f1 + 2q0·f2 + (-4q1)·f3,
                                                                     (-2q0)·f1 + 2q3·f2 + (-4q2)·f3,
                                                                     (2q1)·f1 + 2q2·f2 + 0·f3] */
    /* Corrected: */
    s0 = -_2q2 * f_1 + _2q1 * f_2;
    s1 =  _2q3 * f_1 + _2q0 * f_2 - _4q1 * f_3;
    s2 = -_2q0 * f_1 + _2q3 * f_2 - _4q2 * f_3;
    s3 =  _2q1 * f_1 + _2q2 * f_2;

    /* 归一化梯度 */
    recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (recipNorm == 0.0f) recipNorm = 1.0f;
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    /* ====== 融合: q̇ = ½ q ⊗ ω - β · ▽f ====== */
    float qDot_w = 0.5f * (-q1 * wx - q2 * wy - q3 * wz);
    float qDot_x = 0.5f * ( q0 * wx + q2 * wz - q3 * wy);
    float qDot_y = 0.5f * ( q0 * wy - q1 * wz + q3 * wx);
    float qDot_z = 0.5f * ( q0 * wz + q1 * wy - q2 * wx);

    // 修正的符号：之前 s0..s3 就是 ∇f/|∇f|
    qDot1 = qDot_w - beta_ * s0;  // qDot0
    qDot2 = qDot_x - beta_ * s1;  // qDot1
    qDot3 = qDot_y - beta_ * s2;  // qDot2
    qDot4 = qDot_z - beta_ * s3;  // qDot3

    /* ====== 欧拉积分 ====== */
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    /* ====== 归一化 ====== */
    recipNorm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

/* ================================================================
 *  欧拉角输出
 * ================================================================ */

void madgwick_get_euler(float *roll, float *pitch, float *yaw)
{
    /* Roll (x-axis) */
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    *roll = atan2f(sinr_cosp, cosr_cosp) * (180.0f / M_PI_F);

    /* Pitch (y-axis) */
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f)
        *pitch = copysignf(90.0f, sinp);
    else
        *pitch = asinf(sinp) * (180.0f / M_PI_F);

    /* Yaw (z-axis) */
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    *yaw = atan2f(siny_cosp, cosy_cosp) * (180.0f / M_PI_F);
}

/* ================================================================
 *  重置 Yaw
 * ================================================================ */

void madgwick_reset_yaw(void)
{
    float roll, pitch, yaw;
    madgwick_get_euler(&roll, &pitch, &yaw);

    float cy = cosf(yaw * (M_PI_F / 360.0f));
    float sy = sinf(yaw * (M_PI_F / 360.0f));

    /* 回绕四元数：q_new = q_yaw_inv ⊗ q_current */
    float qy0 = cy, qy1 = 0.0f, qy2 = 0.0f, qy3 = -sy;

    float nq0 = qy0 * q0 - qy1 * q1 - qy2 * q2 - qy3 * q3;
    float nq1 = qy0 * q1 + qy1 * q0 + qy2 * q3 - qy3 * q2;
    float nq2 = qy0 * q2 - qy1 * q3 + qy2 * q0 + qy3 * q1;
    float nq3 = qy0 * q3 + qy1 * q2 - qy2 * q1 + qy3 * q0;

    q0 = nq0; q1 = nq1; q2 = nq2; q3 = nq3;

    float norm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm; q1 *= norm; q2 *= norm; q3 *= norm;
}