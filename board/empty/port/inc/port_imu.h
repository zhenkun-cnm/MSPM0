/**
 * @file    port_imu.h
 * @brief   Port 层 ICM-20608 硬件 SPI 驱动声明
 * @note    实现 dev_imu.h 定义的 DevIMU OOP 契约
 *          使用 SPI1 硬件外设 + GPIO CS（与 W25Q64 共享 SPI1 总线）
 *
 *          手册参考:
 *          - §6.5 SPI Interface (p.28): R/W + 7bit addr, MSB 先, 上升沿锁存
 *          - §3.5 SPI Timing (p.15): 最大 8MHz, Mode 0/3
 *          - §6.1 (p.25): SPI 模式下必须设 I2C_IF_DIS 禁用 I2C
 *
 *          引脚 (SPI1 总线，与 W25Q64 共用 SCK/MOSI/MISO):
 *          PA17 - SCK  (SPI1_SCLK)
 *          PA18 - MOSI (SPI1_PICO)
 *          PA16 - MISO (SPI1_POCI)
 *          PB6  - CS   (GPIO 独立控制, IOMUX_PINCM23)
 */

#ifndef PORT_IMU_H
#define PORT_IMU_H

#include "dev_imu.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 初始化 ICM-20608 引脚（SPI1 复用 + CS GPIO）
 * @note  SPI1 硬件外设由 SysConfig 生成的 W25Q64_init() 初始化，
 *        本函数只配置 CS 引脚为 GPIO 输出高。
 *        调度器启动前调用。
 */
void PORT_IMU_InitGPIO(void);

/**
 * @brief SPI 写一个字节到 ICM-20608 寄存器
 * @param regAddr  寄存器地址（7-bit，不含 R/W 位）
 * @param data     写入的数据
 * @note  手册 §6.5: 首字节 bit7=0(写) + 7bit addr，第二字节为数据
 */
void PORT_IMU_WriteReg(uint8_t regAddr, uint8_t data);

/**
 * @brief SPI 从 ICM-20608 寄存器读一个字节
 * @param regAddr  寄存器地址（7-bit）
 * @return uint8_t  读取的字节
 * @note  手册 §6.5: 首字节 bit7=1(读) + 7bit addr，第二字节为读回数据
 */
uint8_t PORT_IMU_ReadReg(uint8_t regAddr);

/**
 * @brief SPI 从 ICM-20608 连续读多个字节
 * @param regAddr  起始寄存器地址（7-bit）
 * @param buf      输出缓冲区
 * @param len      读取字节数
 * @note  手册 §6.5: 支持突发读，地址自动递增
 */
void PORT_IMU_ReadRegs(uint8_t regAddr, uint8_t *buf, uint16_t len);

/**
 * @brief CS 拉低（选中 ICM-20608）
 */
void PORT_IMU_CS_Low(void);

/**
 * @brief CS 拉高（释放 ICM-20608）
 */
void PORT_IMU_CS_High(void);

/**
 * @brief SPI 单字节收发（发送一字节，返回收到的字节）
 * @param tx  发送字节
 * @return uint8_t  接收字节
 * @note  直接复用 SPI1 硬件外设
 */
uint8_t PORT_IMU_SPI_TransferByte(uint8_t tx);

#endif /* PORT_IMU_H */