/**
 * @file    port_lis3mdl.h
 * @brief   Port 层 LIS3MDLRT 磁力计 I2C 驱动声明
 * @note    I2C0: PA0=SDA, PA1=SCL, 400kHz
 *          LIS3MDLRT I2C 地址: SD0=1 → 0x1E
 */
#ifndef PORT_LIS3MDL_H
#define PORT_LIS3MDL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* LIS3MDLRT I2C 地址 */
#define LIS3MDL_I2C_ADDR        0x1E

/* 初始化 LIS3MDLRT (I2C WHO_AM_I 验证 + 工作模式配置) */
void PORT_LIS3MDL_Init(void);

/* 检查 LIS3MDLRT 是否初始化成功 */
bool PORT_LIS3MDL_IsOk(void);

/* 读取三轴磁场原始数据 (X, Y, Z) — 量程 ±16 gauss */
void PORT_LIS3MDL_ReadMagRaw(int16_t *mx, int16_t *my, int16_t *mz);

#ifdef __cplusplus
}
#endif

#endif /* PORT_LIS3MDL_H */