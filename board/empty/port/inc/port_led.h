/**
 * @file    port_led.h
 * @brief   Port 层 LED 驱动实现声明
 * @note    实现 dev_led.h 定义的 DevLED OOP 契约
 *          内部调用 BSP 层 DL_GPIO_* API
 */

#ifndef PORT_LED_H
#define PORT_LED_H

#include "dev_led.h"

/**
 * @brief 初始化 LED 硬件端口
 * @note  调度器启动前调用，完成 GPIO 引脚配置
 */
void PORT_LED_InitHW(void);

#endif /* PORT_LED_H */