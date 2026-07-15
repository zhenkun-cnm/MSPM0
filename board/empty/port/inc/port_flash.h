/**
 * @file    port_flash.h
 * @brief   Port 层 W25Q64 Flash 驱动声明
 * @note    实现 dev_flash.h 定义的 DevFlash OOP 契约
 *          内部调用 SPI1 + GPIO CS 操作 Flash
 */

#ifndef PORT_FLASH_H
#define PORT_FLASH_H

#include "dev_flash.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 初始化 Flash 硬件（SPI1 + CS GPIO）
 * @note  调度器启动前调用
 */
void PORT_FLASH_InitHW(void);

/**
 * @brief SPI 收发一个字节
 * @param tx  发送字节
 * @return uint8_t  接收字节
 */
uint8_t PORT_FLASH_SPI_TransferByte(uint8_t tx);

/**
 * @brief CS 拉低（选中）
 */
void PORT_FLASH_CS_Low(void);

/**
 * @brief CS 拉高（释放）
 */
void PORT_FLASH_CS_High(void);

#endif /* PORT_FLASH_H */