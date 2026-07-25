/**
 * @file    port_imu.h
 * @brief   Port 层 ICM-20608 硬件 I2C 驱动声明
 * @note    I2C0: PA0=SDA, PA1=SCL, 400kHz
 *          ICM-20608 I2C 地址: AD0=0 → 0x68
 */
#ifndef PORT_IMU_H
#define PORT_IMU_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ICM-20608 I2C 地址 */
#define ICM_I2C_ADDR            0x68

/* 初始化 ICM-20608 (I2C WHO_AM_I 验证 + 传感器配置) */
void PORT_IMU_Init(void);

/* 检查 ICM-20608 是否初始化成功 */
bool PORT_IMU_IsOk(void);

/* 读取加速度计原始数据 (X, Y, Z) — 量程 ±16g */
bool PORT_IMU_ReadAccelRaw(int16_t *ax, int16_t *ay, int16_t *az);

/* 读取陀螺仪原始数据 (X, Y, Z) — 量程 ±2000°/s */
bool PORT_IMU_ReadGyroRaw(int16_t *gx, int16_t *gy, int16_t *gz);

#ifdef __cplusplus
}
#endif

#endif /* PORT_IMU_H */
