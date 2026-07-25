/**
 * @file    app_flash.h
 * @brief   App 层 Flash 验证任务声明
 * @note    四层解耦架构 - App 层
 *          通过 Device 句柄读取 JEDEC ID 验证 SPI 通信
 */

#ifndef APP_FLASH_H
#define APP_FLASH_H

/**
 * @brief Flash 初始化验证任务
 * @param pvParameters  传入参数
 * @note  上电后创建，读取 JEDEC ID 并打印，完成后删除自身
 */
void flash_init_task(void *pvParameters);

#endif /* APP_FLASH_H */
