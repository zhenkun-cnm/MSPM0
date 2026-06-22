/**
 * @file    port_imu.h
 * @brief   Port 层 ICM-20948 软件 SPI 驱动声明
 * @note    实现 dev_imu.h 定义的 DevIMU OOP 契约
 *          软件 SPI bit-bang，严格按 ICM-20948 SPI 时序
 *          上升沿锁存数据（Mode 0, CPOL=0, CPHA=0）
 */

#ifndef PORT_IMU_H
#define PORT_IMU_H

#include "dev_imu.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 软件 SPI 的 SCK 半周期延时（微秒）
 * @note  决定 SCK 实际频率：full = 2 * IMU_SCK_HALF_PERIOD_US。
 *        瓶颈是 LSF0108PWR 无源 FET 电平转换器的 RC 边沿（靠上拉给线容充电），
 *        不是 ICM-20948（可达 7MHz）。
 *        - 3.3V 侧 10kΩ 上拉、1.8V 侧无上拉时取保守值 3us(≈150kHz)；
 *        - 两侧都补到 ~2.2kΩ 上拉后可调小（如 1us≈400kHz）提速。
 */
#ifndef IMU_SCK_HALF_PERIOD_US
#define IMU_SCK_HALF_PERIOD_US   3
#endif

/**
 * @brief 初始化 ICM-20948 引脚（软件 SPI GPIO）
 * @note  调度器启动前调用
 */
void PORT_IMU_InitGPIO(void);

/**
 * @brief SPI 读一个字节（先发地址，再收数据）
 * @param regAddr  寄存器地址（7-bit，不含 R/W 位）
 * @return uint8_t  读取的字节
 */
uint8_t PORT_IMU_ReadReg(uint8_t regAddr);

/**
 * @brief SPI 写一个字节
 * @param regAddr  寄存器地址（7-bit）
 * @param data     写入的数据
 */
void PORT_IMU_WriteReg(uint8_t regAddr, uint8_t data);

/**
 * @brief SPI 连续读多个字节
 * @param regAddr  起始寄存器地址（7-bit）
 * @param buf      输出缓冲区
 * @param len      读取字节数
 */
void PORT_IMU_ReadRegs(uint8_t regAddr, uint8_t *buf, uint16_t len);

/**
 * @brief 切换寄存器 Bank
 * @param bank  Bank 编号（0~3）
 */
void PORT_IMU_SetBank(uint8_t bank);

#endif /* PORT_IMU_H */