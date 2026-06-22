/**
 * @file    port_encoder.h
 * @brief   Port 层编码器驱动实现声明
 * @note    实现 dev_encoder.h 定义的 DevEncoder OOP 契约
 *          内部调用 BSP 层 DL_GPIO_* API
 *          纯轮询，不使用中断
 */

#ifndef PORT_ENCODER_H
#define PORT_ENCODER_H

#include "dev_encoder.h"

/**
 * @brief 编码器初始化（硬件初始化 + 创建内部轮询任务）
 * @note  调度器启动前调用，完成 GPIO 引脚配置
 */
void PORT_ENCODER_Init(void);

/**
 * @brief 编码器周期性轮询处理（5ms 调用一次）
 * @note  在内部轮询任务中调用
 *        完成 A/B 相采样、消抖、Gray 码解码、按键检测
 *        产生的事件写入内部环形队列供 APP 层读取
 */
void PORT_ENCODER_Poll(void);

/**
 * @brief 将编码器事件转换为可读字符串
 * @param evt  编码器事件
 * @return const char*  事件名称字符串
 */
const char* PORT_ENCODER_EventToStr(DevEncoder_Event_t evt);

/**
 * @brief 获取全局编码器设备句柄
 * @return DevEncoder* 指向编码器设备接口的指针
 */
DevEncoder* GetEncoder(void);

#endif /* PORT_ENCODER_H */