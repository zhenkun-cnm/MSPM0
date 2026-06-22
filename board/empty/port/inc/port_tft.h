/**
 * @file    port_tft.h
 * @brief   Port 层 TFT (ST7735) 驱动接口
 * @note    四层解耦架构 - Port 层
 *          实现 dev_tft.h 的 DevTFT OOP 契约
 */

#ifndef PORT_TFT_H
#define PORT_TFT_H

#include "dev_tft.h"

/**
 * @brief 获取 TFT 设备句柄（工厂函数）
 * @return DevTFT* 指向 TFT 设备接口的指针
 * @note   调用此函数前已通过 GetTFT() 获取全局实例，
 *         本头文件仅供外部模块引用时作前向声明。
 */
DevTFT* GetTFT(void);

#endif /* PORT_TFT_H */