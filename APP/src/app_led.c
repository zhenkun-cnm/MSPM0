/**
 * @file    app_led.c
 * @brief   App 层 LED 闪烁任务实现
 * @note    四层解耦架构 - App 层
 *          通过 Device 句柄 led->toggle(led) 操作 LED
 *          无任何 DL_GPIO_* / 寄存器操作
 */

#include "app_led.h"
#include "dev_led.h"
#include "port_log.h"

void led_task(void *pvParameters)
{
    (void)pvParameters;

    /* 获取 LED 设备句柄（OOP 契约） */
    DevLED *led = GetLED();
    if (led == NULL) {
        LOG_ERROR("LED device handle is NULL!\r\n");
        vTaskDelete(NULL);
        return;
    }

    /* 初始化 LED */
    led->init(led);

    LOG_DEBUG("LED task running, toggle every 500ms\r\n");

    while (1)
    {
        /* 通过 Device 层接口翻转 LED，非直接寄存器操作 */
        led->toggle(led);

        /* FreeRTOS 阻塞延时，让出 CPU */
        vTaskDelay(500);
    }
}