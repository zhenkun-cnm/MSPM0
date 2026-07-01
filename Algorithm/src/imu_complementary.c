/**
 * @file    imu_complementary.c
 * @brief   互补滤波姿态解算实现（按 com_imu 基准）
 *          PI 控制器(Kp=0.8, Ki=0.0003) + 叉积误差 + 一阶龙格库塔四元数
 *          陀螺量程: ±2000°/s
 */

#include "imu_complementary.h"
#include <math.h>

#define M_PI_F  3.14159265358979323846f
#define RT_A    57.2957795f
#define KP_DEF  0.8f
#define KI_DEF  0.0003f
#define SQU(x)  ((float)(x) * (float)(x))

/* 内部状态 */
static float q0=1,q1=0,q2=0,q3=0;
static float int_err_x=0, int_err_y=0, int_err_z=0;
static float norm_acc_z=0;
static float yaw_accum=0;

/* Quake III 快速 1/sqrt */
static float q_rsqrt(float x)
{
    float h=0.5f*x, y=x;
    long i=*(long*)&y;
    i=0x5f3759df-(i>>1);
    y=*(float*)&i;
    y=y*(1.5f-h*y*y);
    return y;
}

void complementary_init(void)
{
    q0=1; q1=0; q2=0; q3=0;
    int_err_x=0; int_err_y=0; int_err_z=0;
    norm_acc_z=0; yaw_accum=0;
}

void complementary_update(float gx, float gy, float gz,
                           float ax, float ay, float az,
                           float dt)
{
    /* 加速度归一化 */
    float inv= q_rsqrt(SQU(ax)+SQU(ay)+SQU(az));
    float acc_x=ax*inv, acc_y=ay*inv, acc_z=az*inv;

    /* 重力分量 */
    float grav_x=2*(q1*q3-q0*q2);
    float grav_y=2*(q0*q1+q2*q3);
    float grav_z=1-2*(q1*q1+q2*q2);

    /* 叉积误差 Acc×Gravity */
    float err_x=acc_y*grav_z-acc_z*grav_y;
    float err_y=acc_z*grav_x-acc_x*grav_z;
    float err_z=acc_x*grav_y-acc_y*grav_x;

    /* PI 积分 */
    int_err_x+=err_x*KI_DEF;
    int_err_y+=err_y*KI_DEF;
    int_err_z+=err_z*KI_DEF;

    /* 角速度 PI 融合 (gx 已经是 °/s → rad/s) */
    float wx=gx*(M_PI_F/180.0f)+KP_DEF*err_x+int_err_x;
    float wy=gy*(M_PI_F/180.0f)+KP_DEF*err_y+int_err_y;
    float wz=gz*(M_PI_F/180.0f)+KP_DEF*err_z+int_err_z;

    /* 一阶龙格库塔更新四元数 */
    float ht=dt*0.5f;
    q0+=(-q1*wx-q2*wy-q3*wz)*ht;
    q1+=( q0*wx-q3*wy+q2*wz)*ht;
    q2+=( q3*wx+q0*wy-q1*wz)*ht;
    q3+=(-q2*wx+q1*wy+q0*wz)*ht;

    /* 归一化 */
    inv=q_rsqrt(SQU(q0)+SQU(q1)+SQU(q2)+SQU(q3));
    q0*=inv; q1*=inv; q2*=inv; q3*=inv;

    /* Z 轴投影 (用于姿态补偿加速度) */
    float vzx=2*q0*q2-2*q1*q3;
    float vzy=2*q2*q3+2*q0*q1;
    float vzz=1-2*q1*q1-2*q2*q2;
    norm_acc_z=ax*vzx+ay*vzy+az*vzz;

    /* Yaw: Z陀螺积分+死区0.5°/s */
    if(gz>0.5f||gz<-0.5f) yaw_accum+=gz*dt;
    while(yaw_accum> 180) yaw_accum-=360;
    while(yaw_accum<-180) yaw_accum+=360;
}

void complementary_get_euler(float *roll, float *pitch, float *yaw)
{
    float vzx=2*q0*q2-2*q1*q3;
    float vzy=2*q2*q3+2*q0*q1;
    float vzz=1-2*q1*q1-2*q2*q2;
    *pitch=asinf(vzx)*RT_A;
    *roll=atan2f(vzy,vzz)*RT_A;
    *yaw=yaw_accum;
}

float complementary_get_norm_acc_z(void) { return norm_acc_z; }