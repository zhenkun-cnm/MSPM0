/**
 * @file    app_led.h
 * @brief   App 层 LED 闪烁任务声明
 * @note    四层解耦架构 - App 层
 *          通过 Device 句柄操作 LED，无任何硬件寄存器直接调用
 */

#ifndef APP_LED_H
#define APP_LED_H

#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief LED 闪烁任务函数
 * @param pvParameters  未使用
 * @note   每 500ms 翻转一次 LED
 */
void led_task(void *pvParameters);

#endif /* APP_LED_H */