/**
 * @file    port_button.h
 * @brief   Port 层独立按键驱动声明
 * @note    实现 dev_button.h 定义的 DevButton OOP 契约
 *          内部调用 BSP 层 DL_GPIO_* API 操作 PA27 / PB27
 *          纯轮询，不使用中断
 */

#ifndef PORT_BUTTON_H
#define PORT_BUTTON_H

#include "dev_button.h"

/**
 * @brief 按键初始化（清零 FSM / tick / 事件队列）
 * @note  GPIO 引脚已由 SYSCFG_DL_GPIO_init() 配置为输入
 */
void PORT_BUTTON_Init(void);

/**
 * @brief 按键周期性轮询处理（5ms 调用一次）
 * @note  完成两路按键采样、消抖、短/长/双击 FSM
 *        产生的事件写入内部环形队列供 APP 层读取
 */
void PORT_BUTTON_Poll(void);

/**
 * @brief 将按键事件类型转换为可读字符串
 * @param t  按键事件类型
 * @return const char*  事件名称字符串
 */
const char* PORT_BUTTON_EvtToStr(DevButton_EvtType_t t);

/**
 * @brief 获取全局按键设备句柄
 * @return DevButton* 指向按键设备接口的指针
 */
DevButton* GetButton(void);

#endif /* PORT_BUTTON_H */
