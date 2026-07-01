/**
 * @file    imu_complementary.h
 * @brief   互补滤波姿态解算（按 com_imu 基准）
 *          PI 控制器 + 叉积误差 + 四元数龙格库塔
 *          陀螺量程: ±2000°/s (Gyro_G=4000/65536)
 */

#ifndef IMU_COMPLEMENTARY_H
#define IMU_COMPLEMENTARY_H

void complementary_init(void);
void complementary_update(float gx, float gy, float gz,
                           float ax, float ay, float az,
                           float dt);
void complementary_get_euler(float *roll, float *pitch, float *yaw);
float complementary_get_norm_acc_z(void);

#endif