/**
 * @file    app_tft.h
 * @brief   App 层 TFT 显示任务声明
 * @note    四层解耦架构 - App 层
 *          通过 Device 句柄操作 ST7735 TFT 彩屏
 *          演示：清屏 → 显示标题 → 滚动消息
 */

#ifndef APP_TFT_H
#define APP_TFT_H

/**
 * @brief TFT 显示任务
 * @param pvParameters  传入参数
 * @note  FreeRTOS 任务函数，由 start_task 创建
 *        初始化 TFT 并周期性刷新显示内容
 */
void tft_task(void *pvParameters);

#endif /* APP_TFT_H */