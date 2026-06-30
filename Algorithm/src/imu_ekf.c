/**
 * @file    imu_ekf.c
 * @brief   7 状态四元数 EKF 姿态估计实现
 * @note    状态: x = [qw, qx, qy, qz, gbx, gby, gbz]
 *          - q[0..3]: 单位四元数
 *          - gb[4..6]: 陀螺零偏 (rad/s 内部存储)
 *
 *          预测: 陀螺四元数积分 + 零偏不变
 *          更新: 加速度计重力向量测量 + 自适应 R
 *
 *          所有角度输入/输出为 °/s 或 °，内部为 rad 或 rad/s
 */

#include "imu_ekf.h"
#include "imu_math.h"
#include <string.h>

/* ================================================================
 *  内部状态
 * ================================================================ */

static double x_[7];       /* 状态: qw, qx, qy, qz, gbx, gby, gbz (rad/s) */
static double P_[49];      /* 协方差矩阵 7×7 */

static double gyroFreq_;   /* 陀螺采样频率 (Hz) */
static double accelFreq_;  /* 加速计更新频率 (Hz) */

/* ================================================================
 *  过程噪声 Q（7×7 对角）
 *
 *  四元数噪声: 1e-6 (rad²) — 陀螺积分不确定性
 *  零偏噪声:   1e-8 (rad²/s²) — 零偏缓慢漂移
 *
 *  注意: Q 实际是连续时间谱密度，离散化后乘以 dt
 * ================================================================ */
static const double Q_diag_[7] = {
    1e-6, 1e-6, 1e-6, 1e-6,  /* 四元数 */
    5e-10, 5e-10, 5e-10      /* 零偏 (慢变) */
};

/* ================================================================
 *  测量噪声 R（3×3 对角）
 *
 *  静止: R = diag(0.3, 0.3, 0.3) g² — 高置信
 *  运动: R = diag(3.0, 3.0, 3.0) g² — 低置信
 *
 *  自适应判断: |accel| 偏离 1g 超过 0.1g → 运动
 * ================================================================ */
static const double R_static_[9] = {
    0.3, 0.0, 0.0,
    0.0, 0.3, 0.0,
    0.0, 0.0, 0.3
};
static const double R_moving_[9] = {
    3.0, 0.0, 0.0,
    0.0, 3.0, 0.0,
    0.0, 0.0, 3.0
};

/* ================================================================
 *  初始化
 * ================================================================ */

void imu_ekf_init(double sampleFreq, double accelFreq)
{
    gyroFreq_  = sampleFreq;
    accelFreq_ = accelFreq;

    /* 状态初始化: 水平朝上 */
    quat_identity(x_);
    x_[4] = 0.0; x_[5] = 0.0; x_[6] = 0.0;  /* 零偏 = 0 */

    /* 协方差初始化 */
    double P0_diag[7] = { 1.0, 1.0, 1.0, 1.0, 0.01, 0.01, 0.01 };
    mat7x7_diag(P_, P0_diag);
}

/* ================================================================
 *  预测步（陀螺驱动）
 *
 *  状态转移:
 *    q_{k+1} = q_k ⊕ [1, ½ ω̃ dt]   (四元数旋转)
 *    b_{k+1} = b_k                  (零偏不变)
 *
 *  其中 ω̃ = ω_raw - b̂（去偏角速度）
 *
 *  协方差:
 *    P = F·P·F^T + G·Q·G^T
 *    F = I + ∂f/∂x · dt  (7×7 雅可比)
 * ================================================================ */

void imu_ekf_predict(double gx, double gy, double gz, double dt)
{
    double qw = x_[0], qx = x_[1], qy = x_[2], qz = x_[3];
    double bx = x_[4], by = x_[5], bz = x_[6];

    /* 去偏角速度 (deg/s → rad/s) */
    double wx = (gx - bx) * (M_PI / 180.0);
    double wy = (gy - by) * (M_PI / 180.0);
    double wz = (gz - bz) * (M_PI / 180.0);

    /* 四元数预测: q̇ = ½ q ⊗ ω */
    double half_dt = 0.5 * dt;
    double dq[4];
    dq[0] = 1.0;
    dq[1] = wx * half_dt;
    dq[2] = wy * half_dt;
    dq[3] = wz * half_dt;
    quat_normalize(dq);

    double q_new[4];
    quat_mul(x_, dq, q_new);
    quat_normalize(q_new);

    x_[0] = q_new[0]; x_[1] = q_new[1]; x_[2] = q_new[2]; x_[3] = q_new[3];
    /* 零偏不变 */

    /* ============================================================
     *  状态转移雅可比 F = I + ∂f/∂x * dt
     *
     *  f(x) = [q ⊗ [1, ½(ω-b)dt], b]
     *
     *  ∂f/∂q 块 (4×4):  四元数对自身的导数
     *  ∂f/∂b 块 (4×3):  四元数对零偏的导数（陀螺残差）
     *  ∂f/∂q (b 部分):  0
     *  ∂f/∂b (b 部分):  I
     *
     *  具体:
     *    F[0:4, 0:4] = dq_matrix  (4×4 旋转矩阵)
     *    F[i, 4+j]  = -½ dt * ∂(q⊗ω)/∂b_j  │ 4×3 块
     *
     *  节约计算: 直接构建 7×7 的 F
     * ============================================================ */

    double F[49];
    /* F = I 初始化 */
    for (int i = 0; i < 49; i++) F[i] = (i % 8 == 0) ? 1.0 : 0.0;

    /* F 上 4×4 块 = dq 旋转矩阵的线性近似
     * dq = [1, wx*½dt, wy*½dt, wz*½dt]
     * 四元数乘法的左乘矩阵 */
    double hx = wx * half_dt;
    double hy = wy * half_dt;
    double hz = wz * half_dt;

    /* L(q) 四元数左乘矩阵:
     * [ q0  -q1  -q2  -q3 ]
     * [ q1   q0  -q3   q2 ]
     * [ q2   q3   q0  -q1 ]
     * [ q3  -q2   q1   q0 ]
     *
     * 但用 dq 作为增量:
     * F[0:4, 0:4] = 左乘_dq_矩阵 ≈ I + Ω(dq)
     */
    F[0] = 1.0;  F[1] = -hx; F[2] = -hy; F[3] = -hz;
    F[7] = hx;   F[8] = 1.0;  F[9] = hz;   F[10] = -hy;
    F[14] = hy;  F[15] = -hz; F[16] = 1.0;  F[17] = hx;
    F[21] = hz;  F[22] = hy;  F[23] = -hx;  F[24] = 1.0;

    /* F[0:4, 4:6] = ∂q/∂b = -½ * dt * R(q) * [1 0 0; 0 1 0; 0 0 1] 的合适形式
     * R(q) 是从四元数构建的 3×3 旋转矩阵的转置映射到 4×3。
     * 简化为: dq/db = -½·dt * M, M 是 4×3 矩阵 */
    double neg_half_dt = -0.5 * dt;
    /* M[i][j] = d(q_i)/d(ω_j), 即 ∂(½ q⊗ω)/∂ω */
    /* M = [[-qx, -qy, -qz],
     *      [ qw, -qz,  qy],
     *      [ qz,  qw, -qx],
     *      [-qy,  qx,  qw]] */
    F[4]  = neg_half_dt * (-qx);              /* dq0/dωx */
    F[5]  = neg_half_dt * (-qy);              /* dq0/dωy */
    F[6]  = neg_half_dt * (-qz);              /* dq0/dωz */
    F[11] = neg_half_dt *  qw;                /* dq1/dωx */
    F[12] = neg_half_dt * (-qz);              /* dq1/dωy */
    F[13] = neg_half_dt *  qy;                /* dq1/dωz */
    F[18] = neg_half_dt *  qz;                /* dq2/dωx */
    F[19] = neg_half_dt *  qw;                /* dq2/dωy */
    F[20] = neg_half_dt * (-qx);              /* dq2/dωz */
    F[25] = neg_half_dt * (-qy);              /* dq3/dωx */
    F[26] = neg_half_dt *  qx;                /* dq3/dωy */
    F[27] = neg_half_dt *  qw;                /* dq3/dωz */

    /* 协方差预测: P = F·P·F^T */
    double FP[49];
    mat7x7_mul(F, P_, FP);
    /* P = FP * F^T */
    memset(P_, 0, 49 * sizeof(double));
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7; j++)
            for (int k = 0; k < 7; k++)
                P_[i * 7 + j] += FP[i * 7 + k] * F[j * 7 + k];

    /* 加过程噪声: P += G·Q·G^T * dt
     * G = I_7 (输入噪声直接注入状态), Q = 对角 */
    for (int i = 0; i < 7; i++) {
        double q_i = Q_diag_[i] * dt;
        P_[i * 7 + i] += q_i;
    }
}

/* ================================================================
 *  更新步（加速度计驱动）
 *
 *  测量方程:
 *    z = R(q)^T · g   (重力在体坐标系中的投影)
 *    g = [0, 0, 1]   (重力方向在世界坐标系)
 *
 *  h(x) = [2(qx·qz - qw·qy),
 *           2(qy·qz + qw·qx),
 *           1 - 2(qx² + qy²)]^T
 *
 *  自适应 R:
 *    |a| ≈ 1g → R_static (高置信)
 *    |a| ≠ 1g → R_moving (低置信)
 * ================================================================ */

void imu_ekf_update(double ax, double ay, double az)
{
    double qw = x_[0], qx = x_[1], qy = x_[2], qz = x_[3];

    /* 归一化加速度 */
    double a_norm = sqrt(ax*ax + ay*ay + az*az);
    if (a_norm < 1e-6) return;  /* 零加速度 → 跳过 */

    double ax_n = ax / a_norm;
    double ay_n = ay / a_norm;
    double az_n = az / a_norm;

    /* 自适应 R 选择 */
    const double *R_use;
    double dev = fabs(a_norm - 1.0);
    if (dev < 0.15) {
        R_use = R_static_;   /* 接近 1g → 近似静止 */
    } else if (dev < 0.4) {
        /* 中间态: 线性插值 */
        static double R_interp[9];
        double t = (dev - 0.15) / 0.25;
        for (int i = 0; i < 9; i++)
            R_interp[i] = R_static_[i] * (1.0 - t) + R_moving_[i] * t;
        R_use = R_interp;
    } else {
        R_use = R_moving_;     /* 远离 1g → 明显运动 */
    }

    /* 计算预测测量 h(x) */
    double hx = 2.0 * (qx * qz - qw * qy);
    double hy = 2.0 * (qy * qz + qw * qx);
    double hz = 1.0 - 2.0 * (qx * qx + qy * qy);

    /* 测量残差 z - h(x) */
    double residual[3];
    residual[0] = ax_n - hx;
    residual[1] = ay_n - hy;
    residual[2] = az_n - hz;

    /* 测量雅可比 H (3×7)
     * H[0:3, 0:4] = ∂h/∂q (3×4 关于四元数)
     * H[0:3, 4:7] = 0 (3×3, 加速度计不直接测量零偏)
     *
     * ∂h/∂q:
     *   hx = 2(qx·qz - qw·qy)
     *   hy = 2(qy·qz + qw·qx)
     *   hz = 1 - 2(qx² + qy²)
     *
     * ∂/∂qw: [-2qy, 2qx, 0]
     * ∂/∂qx: [2qz,  2qw, -4qx]
     * ∂/∂qy: [-2qw, 2qz, -4qy]
     * ∂/∂qz: [2qx,  2qy, 0]
     */
    double H[21];
    memset(H, 0, sizeof(H));

    H[0] = -2.0 * qy;  /* d(hx)/d(qw) */
    H[1] =  2.0 * qz;  /* d(hx)/d(qx) */
    H[2] = -2.0 * qw;  /* d(hx)/d(qy) */
    H[3] =  2.0 * qx;  /* d(hx)/d(qz) */

    H[7] =  2.0 * qx;  /* d(hy)/d(qw) */
    H[8] =  2.0 * qw;  /* d(hy)/d(qx) */
    H[9] =  2.0 * qz;  /* d(hy)/d(qy) */
    H[10] = 2.0 * qy;  /* d(hy)/d(qz) */

    H[14] = 0.0;       /* d(hz)/d(qw) */
    H[15] = -4.0 * qx; /* d(hz)/d(qx) */
    H[16] = -4.0 * qy; /* d(hz)/d(qy) */
    H[17] = 0.0;       /* d(hz)/d(qz) */

    /* S = H·P·H^T + R */
    double S[9];
    innovation_cov(P_, H, R_use, S);

    /* K = P·H^T·inv(S) */
    double K[21];
    kalman_gain_7x3(P_, H, S, K);

    /* 状态更新: x += K·residual */
    state_update(x_, K, residual);

    /* 协方差 Joseph 更新 */
    cov_joseph_update(P_, K, H, R_use);
}

/* ================================================================
 *  输出接口
 * ================================================================ */

void imu_ekf_get_euler(double *roll, double *pitch, double *yaw)
{
    quat_to_euler(x_, roll, pitch, yaw);
}

void imu_ekf_get_bias(double *bx, double *by, double *bz)
{
    /* 内部存储为 rad/s，输出为 °/s */
    *bx = x_[4] * 180.0 / M_PI;
    *by = x_[5] * 180.0 / M_PI;
    *bz = x_[6] * 180.0 / M_PI;
}

void imu_ekf_reset_yaw(void)
{
    /* 只重置 Yaw（绕 Z 轴旋转），保留 Roll/Pitch
     *
     * 当前欧拉角: roll, pitch, yaw_old
     * 目标欧拉角: roll, pitch, 0
     *
     * 从欧拉角重新构建四元数
     */
    double roll, pitch, yaw;
    quat_to_euler(x_, &roll, &pitch, &yaw);

    double cr = cos(roll * M_PI / 360.0);
    double sr = sin(roll * M_PI / 360.0);
    double cp = cos(pitch * M_PI / 360.0);
    double sp = sin(pitch * M_PI / 360.0);
    double cy = 1.0;  /* cos(0) */
    double sy = 0.0;  /* sin(0) */

    x_[0] = cr * cp * cy + sr * sp * sy;
    x_[1] = sr * cp * cy - cr * sp * sy;
    x_[2] = cr * sp * cy + sr * cp * sy;
    x_[3] = cr * cp * sy - sr * sp * cy;
    quat_normalize(x_);
}