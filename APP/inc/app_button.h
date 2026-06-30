/**
 * @file    app_button.h
 * @brief   App 层独立按键任务声明
 * @note    四层解耦架构 - App 层
 */

#ifndef APP_BUTTON_H
#define APP_BUTTON_H

/**
 * @brief 独立按键应用任务（PA27 / PB27 短按/长按/双击，非阻塞）
 * @param pvParameters  FreeRTOS 任务参数（未使用）
 */
void button_task(void *pvParameters);

#endif /* APP_BUTTON_H */
