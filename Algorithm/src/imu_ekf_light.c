/**
 * @file    imu_ekf_light.c
 * @brief   4 状态四元数 EKF 实现（M0+ float, 顺序标量更新）
 * @note    状态: q = [q0, q1, q2, q3]
 *          P: 4×4 协方差
 *          Q: 4×4 单位对角过程噪声
 *          R: 1×1 标量测量噪声（每个轴独立）
 */

#include "imu_ekf_light.h"
#include <math.h>
#include <string.h>

#define M_PI_F  3.14159265358979323846f

/* ================================================================
 *  内部状态
 * ================================================================ */

static float q_[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static float P_[16];       /* 4×4 协方差矩阵，行优先 */
static float Q_diag_;      /* 过程噪声对角值 */
static float R_scalar_;    /* 测量噪声标量 */

/* ================================================================
 *  快速 1/sqrt（Quake III 黑魔法）
 * ================================================================ */

static float fast_rsqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - halfx * y * y);
    return y;
}

/* ================================================================
 *  四元数乘法（原地出）
 * ================================================================ */

static void quat_mul_f(const float a[4], const float b[4], float r[4])
{
    r[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    r[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    r[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    r[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

/* ================================================================
 *  初始化
 * ================================================================ */

void ekf_light_init(float beta)
{
    q_[0] = 1.0f; q_[1] = 0.0f; q_[2] = 0.0f; q_[3] = 0.0f;

    /* P 初始: 对角 = 1.0 */
    memset(P_, 0, sizeof(P_));
    for (int i = 0; i < 4; i++) P_[i * 4 + i] = 1.0f;

    /* Q: 过程噪声 — 吸收未建模零偏 */
    Q_diag_ = 1e-4f;

    /* R: 测量噪声 — beta 映射 */
    /* 小 beta → 大 R (信陀螺) */
    /* 大 beta → 小 R (信加计) */
    /* 映射: R = 1/beta (0.02→50, 0.1→10) */
    R_scalar_ = 1.0f / (beta + 0.001f);
    if (R_scalar_ > 100.0f) R_scalar_ = 100.0f;
    if (R_scalar_ < 1.0f)  R_scalar_ = 1.0f;
}

void ekf_light_init_from_accel(float beta, float ax, float ay, float az)
{
    /* 归一化加速度 */
    float n = sqrtf(ax*ax + ay*ay + az*az);
    if (n < 1e-4f) n = 1.0f;
    ax /= n; ay /= n; az /= n;

    /* 从重力方向计算 Roll 和 Pitch
     * Pitch = asin(-ax)  (重力在 X 轴投影)
     * Roll  = atan2(ay, az) (重力在 YZ 平面)
     */
    float roll  = atan2f(ay, az);
    float pitch = asinf(-ax);   /* ax 归一化后范围 [-1,1] */
    float yaw   = 0.0f;         /* 无磁力计，Yaw 从 0 开始 */

    /* 欧拉角 → 四元数 (ZYX 顺序: Yaw → Pitch → Roll) */
    float cy = cosf(yaw * 0.5f),   sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f),  sr = sinf(roll * 0.5f);

    q_[0] = cr * cp * cy + sr * sp * sy;
    q_[1] = sr * cp * cy - cr * sp * sy;
    q_[2] = cr * sp * cy + sr * cp * sy;
    q_[3] = cr * cp * sy - sr * sp * cy;

    /* 归一化 */
    float qn = fast_rsqrt(q_[0]*q_[0] + q_[1]*q_[1] + q_[2]*q_[2] + q_[3]*q_[3]);
    q_[0] *= qn; q_[1] *= qn; q_[2] *= qn; q_[3] *= qn;

    /* 设置 P (与 ekf_light_init 相同) */
    memset(P_, 0, sizeof(P_));
    for (int i = 0; i < 4; i++) P_[i * 4 + i] = 1.0f;

    Q_diag_ = 1e-4f;
    R_scalar_ = 1.0f / (beta + 0.001f);
    if (R_scalar_ > 100.0f) R_scalar_ = 100.0f;
    if (R_scalar_ < 1.0f)  R_scalar_ = 1.0f;
}

/* ================================================================
 *  顺序标量更新（核心优化）
 *
 *  对每个加速度轴 (ax, ay, az) 单独做卡尔曼更新:
 *    measurement = a_i (归一化后)
 *    h = 四元数预测的重力分量
 *    residual = measurement - h
 *    H = ∂h/∂q  (1×4 行向量)
 *    S = H·P·H^T + R   (标量!)
 *    K = P·H^T / S      (4×1)
 *    q += K * residual
 *    P = (I - K·H)·P   (4×4)
 *
 *  三次循环完成 3 轴更新，无 3×3 矩阵求逆！
 * ================================================================ */

static void scalar_update(float measurement, const float H[4])
{
    /* PHt = P * H^T  (4×1) */
    float PHt[4];
    for (int i = 0; i < 4; i++) {
        PHt[i] = P_[i*4+0] * H[0] + P_[i*4+1] * H[1]
               + P_[i*4+2] * H[2] + P_[i*4+3] * H[3];
    }

    /* S = H * PHt + R (标量) */
    float S = H[0]*PHt[0] + H[1]*PHt[1] + H[2]*PHt[2] + H[3]*PHt[3] + R_scalar_;
    if (S < 1e-6f) return;

    /* K = PHt / S (4×1) */
    float K[4];
    float invS = 1.0f / S;
    for (int i = 0; i < 4; i++) K[i] = PHt[i] * invS;

    /* 计算残差 */
    float qw = q_[0], qx = q_[1], qy = q_[2], qz = q_[3];
    float h_val;
    /* h 的计算依赖哪个轴（外部传入 H 时已在外部算好 h） */
    /* 这里 h 在调用方已计算 */

    /* residual = measurement - h_meas
     * 调用方传入 measurement 和 h_meas 的差 */
    float residual = measurement;

    /* q += K * residual */
    q_[0] += K[0] * residual;
    q_[1] += K[1] * residual;
    q_[2] += K[2] * residual;
    q_[3] += K[3] * residual;

    /* P = (I - K*H) * P
     * 因为 K*H 是 4×4 外积: (K*H)[i][j] = K[i] * H[j]
     * (I - KH)*P = P - K*(H*P)
     * 其中 H*P 是 1×4 行向量: HP[j] = Σ H[k]*P[k][j]
     * (K*HP)[i][j] = K[i] * HP[j]
     */
    float HP[4] = {0};
    for (int j = 0; j < 4; j++) {
        HP[j] = H[0]*P_[   +j] + H[1]*P_[ 4+j]
              + H[2]*P_[ 8+j] + H[3]*P_[12+j];
    }

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            P_[i*4 + j] -= K[i] * HP[j];
        }
    }

    /* 对称化（数值稳定） */
    for (int i = 0; i < 4; i++) {
        for (int j = i+1; j < 4; j++) {
            float avg = (P_[i*4+j] + P_[j*4+i]) * 0.5f;
            P_[i*4+j] = P_[j*4+i] = avg;
        }
    }
}

/* ================================================================
 *  主更新
 * ================================================================ */

void ekf_light_update(float gx, float gy, float gz,
                      float ax, float ay, float az,
                      float dt)
{
    /* ====== 归一化加速度 ====== */
    float a_norm = sqrtf(ax*ax + ay*ay + az*az);
    if (a_norm < 1e-4f) return;
    float inv_norm = 1.0f / a_norm;
    ax *= inv_norm;
    ay *= inv_norm;
    az *= inv_norm;

    /* ====== 预测: 四元数陀螺积分 ====== */
    float wx = gx * (M_PI_F / 180.0f);
    float wy = gy * (M_PI_F / 180.0f);
    float wz = gz * (M_PI_F / 180.0f);

    float half_dt = 0.5f * dt;
    float dq[4];
    dq[0] = 1.0f;
    dq[1] = wx * half_dt;
    dq[2] = wy * half_dt;
    dq[3] = wz * half_dt;

    /* 归一化增量四元数 */
    float dq_norm = fast_rsqrt(dq[0]*dq[0] + dq[1]*dq[1] + dq[2]*dq[2] + dq[3]*dq[3]);
    dq[0] *= dq_norm; dq[1] *= dq_norm; dq[2] *= dq_norm; dq[3] *= dq_norm;

    float q_new[4];
    quat_mul_f(q_, dq, q_new);

    /* 归一化预测四元数 */
    float qn = fast_rsqrt(q_new[0]*q_new[0] + q_new[1]*q_new[1] + q_new[2]*q_new[2] + q_new[3]*q_new[3]);
    q_[0] = q_new[0] * qn;
    q_[1] = q_new[1] * qn;
    q_[2] = q_new[2] * qn;
    q_[3] = q_new[3] * qn;

    /* ====== 协方差预测: P = P + Q (F=I 简化) ====== */
    P_[0]  += Q_diag_;
    P_[5]  += Q_diag_;
    P_[10] += Q_diag_;
    P_[15] += Q_diag_;

    /* ====== 顺序更新: 3 轴加速度计 ====== */

    float qw = q_[0], qx = q_[1], qy = q_[2], qz = q_[3];

    /* 重力预测 h = R(q)^T * [0,0,1]:
     * hx = 2(qx·qz - qw·qy)
     * hy = 2(qy·qz + qw·qx)
     * hz = 1 - 2(qx² + qy²)
     */

    /* --- X 轴 --- */
    float hx = 2.0f * (qx * qz - qw * qy);
    float residual_x = ax - hx;
    float Hx[4] = {-2.0f*qy, 2.0f*qz, -2.0f*qw, 2.0f*qx};
    scalar_update(residual_x, Hx);

    /* 更新四元数引用（scalar_update 修改了 q_） */
    qw = q_[0]; qx = q_[1]; qy = q_[2]; qz = q_[3];
    /* 重新归一化 */
    float qn2 = fast_rsqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    q_[0] *= qn2; q_[1] *= qn2; q_[2] *= qn2; q_[3] *= qn2;
    qw = q_[0]; qx = q_[1]; qy = q_[2]; qz = q_[3];

    /* --- Y 轴 --- */
    float hy = 2.0f * (qy * qz + qw * qx);
    float residual_y = ay - hy;
    float Hy[4] = {2.0f*qx, 2.0f*qw, 2.0f*qz, 2.0f*qy};
    scalar_update(residual_y, Hy);

    qw = q_[0]; qx = q_[1]; qy = q_[2]; qz = q_[3];
    qn2 = fast_rsqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    q_[0] *= qn2; q_[1] *= qn2; q_[2] *= qn2; q_[3] *= qn2;
    qw = q_[0]; qx = q_[1]; qy = q_[2]; qz = q_[3];

    /* --- Z 轴 --- */
    float hz = 1.0f - 2.0f * (qx*qx + qy*qy);
    float residual_z = az - hz;
    float Hz[4] = {0.0f, -4.0f*qx, -4.0f*qy, 0.0f};
    scalar_update(residual_z, Hz);

    /* 最终归一化 */
    qw = q_[0]; qx = q_[1]; qy = q_[2]; qz = q_[3];
    qn2 = fast_rsqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    q_[0] *= qn2; q_[1] *= qn2; q_[2] *= qn2; q_[3] *= qn2;
}

/* ================================================================
 *  欧拉角输出
 * ================================================================ */

void ekf_light_get_euler(float *roll, float *pitch, float *yaw)
{
    float qw = q_[0], qx = q_[1], qy = q_[2], qz = q_[3];

    float sinr = 2.0f * (qw * qx + qy * qz);
    float cosr = 1.0f - 2.0f * (qx * qx + qy * qy);
    *roll = atan2f(sinr, cosr) * (180.0f / M_PI_F);

    float sinp = 2.0f * (qw * qy - qz * qx);
    if (fabsf(sinp) >= 1.0f)
        *pitch = copysignf(90.0f, sinp);
    else
        *pitch = asinf(sinp) * (180.0f / M_PI_F);

    float siny = 2.0f * (qw * qz + qx * qy);
    float cosy = 1.0f - 2.0f * (qy * qy + qz * qz);
    *yaw = atan2f(siny, cosy) * (180.0f / M_PI_F);
}