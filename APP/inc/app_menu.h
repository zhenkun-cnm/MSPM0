/**
 * @file    app_menu.h
 * @brief   App 层菜单事件共享定义
 * @note    四层解耦架构 - App 层
 *          encoder_task 把编码器事件投递到 g_menuEvtQueue，
 *          tft_task 阻塞读取并驱动多级菜单刷新。
 */

#ifndef APP_MENU_H
#define APP_MENU_H

#include "FreeRTOS.h"
#include "queue.h"

/* 编码器→菜单事件队列（在 app_init.c 中创建，任务启动前完成） */
extern QueueHandle_t g_menuEvtQueue;

/* 队列深度（DevEncoder_Event_t 元素个数） */
#define MENU_EVT_QUEUE_LEN      8U

#endif /* APP_MENU_H */
