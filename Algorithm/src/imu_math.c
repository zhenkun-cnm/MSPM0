/**
 * @file    imu_math.c
 * @brief   四元数/矩阵/向量数学工具实现（双精度）
 */

#include "imu_math.h"
#include <string.h>

/* ================================================================
 *  四元数运算
 * ================================================================ */

void quat_mul(const double p[4], const double q[4], double r[4])
{
    double pw = p[0], px = p[1], py = p[2], pz = p[3];
    double qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    r[0] = pw * qw - px * qx - py * qy - pz * qz;
    r[1] = pw * qx + px * qw + py * qz - pz * qy;
    r[2] = pw * qy - px * qz + py * qw + pz * qx;
    r[3] = pw * qz + px * qy - py * qx + pz * qw;
}

void quat_normalize(double q[4])
{
    double n = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n > 1e-12) {
        q[0] /= n; q[1] /= n; q[2] /= n; q[3] /= n;
    }
}

void quat_identity(double q[4])
{
    q[0] = 1.0; q[1] = 0.0; q[2] = 0.0; q[3] = 0.0;
}

void quat_to_euler(const double q[4], double *roll, double *pitch, double *yaw)
{
    double qw = q[0], qx = q[1], qy = q[2], qz = q[3];

    /* Roll (x-axis rotation) */
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    *roll = atan2(sinr_cosp, cosr_cosp);

    /* Pitch (y-axis rotation) */
    double sinp = 2.0 * (qw * qy - qz * qx);
    if (fabs(sinp) >= 1.0)
        *pitch = copysign(M_PI / 2.0, sinp);
    else
        *pitch = asin(sinp);

    /* Yaw (z-axis rotation) */
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    *yaw = atan2(siny_cosp, cosy_cosp);

    /* 弧度 → 度 */
    *roll  *= 180.0 / M_PI;
    *pitch *= 180.0 / M_PI;
    *yaw   *= 180.0 / M_PI;
}

/* ================================================================
 *  向量运算
 * ================================================================ */

void vec3_cross(const double a[3], const double b[3], double c[3])
{
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

double vec3_dot(const double a[3], const double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vec3_normalize(double v[3])
{
    double n = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (n > 1e-12) { v[0] /= n; v[1] /= n; v[2] /= n; }
}

double vec3_norm(const double v[3])
{
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

void vec3_sub(const double a[3], const double b[3], double d[3])
{
    d[0] = a[0] - b[0];
    d[1] = a[1] - b[1];
    d[2] = a[2] - b[2];
}

/* ================================================================
 *  矩阵运算
 * ================================================================ */

void mat7x7_mul(const double A[49], const double B[49], double C[49])
{
    double t[49];
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            double sum = 0.0;
            for (int k = 0; k < 7; k++) {
                sum += A[i * 7 + k] * B[k * 7 + j];
            }
            t[i * 7 + j] = sum;
        }
    }
    memcpy(C, t, 49 * sizeof(double));
}

void mat7x7_add_outer(double C[49], const double a[7], double scalar)
{
    for (int i = 0; i < 7; i++) {
        for (int j = i; j < 7; j++) {
            double v = a[i] * a[j] * scalar;
            C[i * 7 + j] += v;
            C[j * 7 + i] += v;
        }
    }
}

void mat7x7_identity(double M[49], double value)
{
    memset(M, 0, 49 * sizeof(double));
    for (int i = 0; i < 7; i++) M[i * 7 + i] = value;
}

void mat7x7_diag(double M[49], const double d[7])
{
    memset(M, 0, 49 * sizeof(double));
    for (int i = 0; i < 7; i++) M[i * 7 + i] = d[i];
}

/* ================================================================
 *  3×3 求逆 (伴随矩阵法)
 * ================================================================ */

bool mat3x3_inv(const double A[9], double B[9])
{
    double det = A[0] * (A[4]*A[8] - A[5]*A[7])
               - A[1] * (A[3]*A[8] - A[5]*A[6])
               + A[2] * (A[3]*A[7] - A[4]*A[6]);

    if (fabs(det) < 1e-15) return false;

    double invDet = 1.0 / det;
    B[0] = (A[4]*A[8] - A[5]*A[7]) * invDet;
    B[1] = (A[2]*A[7] - A[1]*A[8]) * invDet;
    B[2] = (A[1]*A[5] - A[2]*A[4]) * invDet;
    B[3] = (A[5]*A[6] - A[3]*A[8]) * invDet;
    B[4] = (A[0]*A[8] - A[2]*A[6]) * invDet;
    B[5] = (A[2]*A[3] - A[0]*A[5]) * invDet;
    B[6] = (A[3]*A[7] - A[4]*A[6]) * invDet;
    B[7] = (A[1]*A[6] - A[0]*A[7]) * invDet;
    B[8] = (A[0]*A[4] - A[1]*A[3]) * invDet;
    return true;
}

/* ================================================================
 *  EKF 专用运算
 * ================================================================ */

void innovation_cov(const double P[49], const double H[21],
                    const double R[9], double S[9])
{
    /* S = H * P * H^T + R, H is 3×7, P is 7×7 → temp(3×7) */
    double HP[21];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 7; j++) {
            double sum = 0.0;
            for (int k = 0; k < 7; k++) {
                sum += H[i * 7 + k] * P[k * 7 + j];
            }
            HP[i * 7 + j] = sum;
        }
    }
    /* S = HP * H^T */
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            double sum = 0.0;
            for (int k = 0; k < 7; k++) {
                sum += HP[i * 7 + k] * H[j * 7 + k];
            }
            S[i * 3 + j] = sum + R[i * 3 + j];
        }
    }
}

void kalman_gain_7x3(const double P[49], const double H[21],
                      const double S[9], double K[21])
{
    double S_inv[9];
    if (!mat3x3_inv(S, S_inv)) {
        memset(K, 0, 21 * sizeof(double));
        return;
    }
    /* K = P * H^T * S_inv → temp(7×3) = P * H^T */
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 3; j++) {
            double sum = 0.0;
            for (int k = 0; k < 7; k++) {
                sum += P[i * 7 + k] * H[j * 7 + k];
            }
            double val = 0.0;
            for (int m = 0; m < 3; m++) {
                val += sum * S_inv[m * 3 + j];
            }
            /* 实际上需要 sum 作为第 m 维。重新算：
               K[i][j] = Σ_k P[i][k] * H[j][k] → PHt[i][j]
               K[i][j] = Σ_m PHt[i][m] * S_inv[m][j]
               但我们只有一个 sum，需要累加所有 m。
               修正: */
            K[i * 3 + j] = val;
        }
    }
    /* 修正: 正确计算 K = P·H^T · S_inv */
    double PHt[21];
    memset(PHt, 0, sizeof(PHt));
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 3; j++) {
            double s = 0.0;
            for (int k = 0; k < 7; k++) s += P[i * 7 + k] * H[j * 7 + k];
            PHt[i * 3 + j] = s;
        }
    }
    memset(K, 0, 21 * sizeof(double));
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 3; j++) {
            double s = 0.0;
            for (int m = 0; m < 3; m++) s += PHt[i * 3 + m] * S_inv[m * 3 + j];
            K[i * 3 + j] = s;
        }
    }
}

void state_update(double x[7], const double K[21], const double residual[3])
{
    double dx[7];
    for (int i = 0; i < 7; i++) {
        dx[i] = K[i * 3 + 0] * residual[0]
              + K[i * 3 + 1] * residual[1]
              + K[i * 3 + 2] * residual[2];
    }
    /* 四元数部分用加法更新（小增量假设） */
    x[0] += dx[0]; x[1] += dx[1]; x[2] += dx[2]; x[3] += dx[3];
    quat_normalize(x);
    /* 零偏更新 */
    x[4] += dx[4]; x[5] += dx[5]; x[6] += dx[6];
}

void cov_joseph_update(double P[49], const double K[21],
                       const double H[21], const double R[9])
{
    /* Joseph 格式: P = (I-KH)P(I-KH)^T + KRK^T
     * 分两步: temp = (I-KH)·P, 然后 P = temp·(I-KH)^T + KRK^T
     * 为保证数值稳定，先做 I-KH (7×7) */
    double KH[49];
    memset(KH, 0, sizeof(KH));
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7; j++)
            for (int k = 0; k < 3; k++)
                KH[i * 7 + j] += K[i * 3 + k] * H[k * 7 + j];

    double ImKH[49];
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7; j++)
            ImKH[i * 7 + j] = (i == j ? 1.0 : 0.0) - KH[i * 7 + j];

    /* temp = ImKH * P */
    double temp[49];
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7; j++) {
            temp[i * 7 + j] = 0.0;
            for (int k = 0; k < 7; k++)
                temp[i * 7 + j] += ImKH[i * 7 + k] * P[k * 7 + j];
        }

    /* P = temp * ImKH^T */
    memset(P, 0, 49 * sizeof(double));
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7; j++)
            for (int k = 0; k < 7; k++)
                P[i * 7 + j] += temp[i * 7 + k] * ImKH[j * 7 + k];

    /* P += K * R * K^T */
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7; j++)
            for (int m = 0; m < 3; m++)
                for (int n = 0; n < 3; n++)
                    P[i * 7 + j] += K[i * 3 + m] * R[m * 3 + n] * K[j * 3 + n];
}