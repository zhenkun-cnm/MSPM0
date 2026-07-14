/**
 * @file    empty.c
 * @brief   工程唯一入口
 * @note    四层解耦架构：
 *          1. PORT_SYSTEM_Init()  - 调度器启动前硬件初始化
 *          2. app_init()         - App 层初始化（创建 start_task）
 *          3. app_start()        - 启动 FreeRTOS 调度器
 */

#include "port_system.h"
#include "app_init.h"

int main(void)
{
    /* Step 1: 纯硬件寄存器配置（时钟、GPIO、UART），调度器启动前完成 */
    PORT_SYSTEM_Init();

    /* Step 2: App 层初始化 - 创建 start_task */
    app_init();

    /* Step 3: 启动 FreeRTOS 调度器 */
    app_start();

    /* 不应到达此处 */
    return 0;
}
//883 046 809 046