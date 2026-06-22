/**
 * @file    app_encoder.h
 * @brief   App 层编码器任务声明
 * @note    四层解耦架构 - App 层
 *          通过 Device 句柄操作编码器，无任何硬件寄存器直接调用
 *          编码器轮询由 Port 层内部任务驱动，APP 层仅消费事件
 */

#ifndef APP_ENCODER_H
#define APP_ENCODER_H

/**
 * @brief 编码器应用任务
 * @param pvParameters  传入参数
 * @note  FreeRTOS 任务函数，由 start_task 创建
 *        内部周期调用 PORT_ENCODER_Poll() 并输出日志
 */
void encoder_task(void *pvParameters);

#endif /* APP_ENCODER_H */