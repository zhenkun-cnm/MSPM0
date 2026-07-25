/**
 * @file    app_button.c
 * @brief   App 层独立按键应用任务实现
 * @note    四层解耦架构 - App 层
 *          通过 Device 句柄操作按键，无任何 DL_GPIO_* / 寄存器操作
 *          按键轮询由内部周期性调用 PORT_BUTTON_Poll() 驱动，全程非阻塞
 *
 *          事件输出示例:
 *          [INFO] BTN1: SHORT_PRESS\r\n
 *          [INFO] BTN2: LONG_PRESS\r\n
 */

#include "app_button.h"
#include "dev_button.h"
#include "port_button.h"
#include "port_log.h"
#include "FreeRTOS.h"
#include "task.h"

#define BTN_TASK_DELAY_MS       5U      /* 轮询周期 5ms */

void button_task(void *pvParameters)
{
    (void)pvParameters;

    /* 获取按键设备句柄 */
    DevButton *btn = GetButton();

    /* 初始化 Port 层按键状态 */
    PORT_BUTTON_Init();

    while (1)
    {
        /* 执行按键轮询：读两路 GPIO → 消抖 → 短/长/双击 FSM */
        PORT_BUTTON_Poll();

        /* 读取并消费所有待处理事件：串口打印（非阻塞验证手段） */
        DevButton_Event_t e;
        while ((e = btn->getEvent(btn)).type != BUTTON_EVT_NONE) {
            LOGI(LOG_MOD_BUTTON, "BTN%u: %s\r\n",
                     (unsigned)e.id, PORT_BUTTON_EvtToStr(e.type));
        }

        /* 让出 CPU，等待下一次轮询 */
        vTaskDelay(pdMS_TO_TICKS(BTN_TASK_DELAY_MS));
    }
}
