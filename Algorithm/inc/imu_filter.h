/**
 * @file    imu_filter.h
 * @brief   IMU 数据预处理滤波（按 com_filter 基准）
 *          一阶低通: 角速度 3 轴
 *          1D 卡尔曼: 加速度 3 轴
 */

#ifndef IMU_FILTER_H
#define IMU_FILTER_H

typedef struct {
    float LastP;
    float Now_P;
    float out;
    float Kg;
    float Q;
    float R;
} ImuKalmanFilter;

void imu_kalman_init(ImuKalmanFilter *kf, float q, float r,
                     float init_p, float init_val);
float imu_kalman_update(ImuKalmanFilter *kf, float input);
float imu_lowpass(float new_val, float prev, float alpha);

#endif