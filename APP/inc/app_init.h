/**
 * @file    app_init.h
 * @brief   App 层初始化与启动接口
 * @note    四层解耦架构 - App 层
 *          app_init()  在调度器启动前调用，创建 start_task
 *          app_start() 启动 FreeRTOS 调度器
 */

#ifndef APP_INIT_H
#define APP_INIT_H

/**
 * @brief App 层初始化
 * @note  创建 start_task（负责创建所有应用任务）
 *        必须在 FreeRTOS 调度器启动前调用
 */
void app_init(void);

/**
 * @brief 启动 FreeRTOS 调度器
 * @note  调用 vTaskStartScheduler()，不会返回
 */
void app_start(void);

#endif /* APP_INIT_H */