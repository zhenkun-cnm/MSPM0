/**
 * @file    imu_filter.c
 * @brief   IMU 数据预处理滤波实现（按 com_filter 基准）
 * @note    算法来源: com_filter.c
 *          Q=0.001, R=0.543, P0=0.02
 */

#include "imu_filter.h"

void imu_kalman_init(ImuKalmanFilter *kf, float q, float r,
                     float init_p, float init_val)
{
    kf->Q     = q;
    kf->R     = r;
    kf->LastP = init_p;
    kf->Now_P = 0.0f;
    kf->Kg    = 0.0f;
    kf->out   = init_val;
}

float imu_kalman_update(ImuKalmanFilter *kf, float input)
{
    kf->Now_P = kf->LastP + kf->Q;
    kf->Kg    = kf->Now_P / (kf->Now_P + kf->R);
    kf->out   = kf->out + kf->Kg * (input - kf->out);
    kf->LastP = (1.0f - kf->Kg) * kf->Now_P;
    return kf->out;
}

float imu_lowpass(float new_val, float prev, float alpha)
{
    return alpha * new_val + (1.0f - alpha) * prev;
}