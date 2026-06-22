/**
 * @file    app_encoder.c
 * @brief   App 层编码器应用任务实现
 * @note    四层解耦架构 - App 层
 *          通过 Device 句柄操作编码器，无任何 DL_GPIO_* / 寄存器操作
 *          编码器轮询由内部周期性调用 PORT_ENCODER_Poll() 驱动
 *
 *          事件输出示例:
 *          [INFO] ENC: CW,  pos=5\r\n
 *          [INFO] KEY: SHORT_PRESS\r\n
 */

#include "app_encoder.h"
#include "app_menu.h"
#include "dev_encoder.h"
#include "port_encoder.h"
#include "port_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define ENC_TASK_DELAY_MS       5U      /* 轮询周期 5ms */

void encoder_task(void *pvParameters)
{
    (void)pvParameters;

    /* 获取编码器设备句柄 */
    DevEncoder *enc = GetEncoder();

    /* 初始化 Port 层编码器硬件状态 */
    PORT_ENCODER_Init();

    while (1)
    {
        /* 执行编码器轮询：读 GPIO → 消抖 → Gray 码解码 → 按键 FSM */
        PORT_ENCODER_Poll();

        /* 读取并消费所有待处理事件：投递给菜单任务（队列），并打印日志 */
        DevEncoder_Event_t evt;
        while ((evt = enc->getEvent(enc)) != ENCODER_EVT_NONE) {
            /* 转发给 TFT 菜单任务（队列未就绪时跳过） */
            if (g_menuEvtQueue != NULL) {
                xQueueSend(g_menuEvtQueue, &evt, 0);
            }

            if (evt == ENCODER_EVT_CW) {
                LOG_INFO("RIGHT\r\n");
            } else if (evt == ENCODER_EVT_CCW) {
                LOG_INFO("LEFT\r\n");
            } else {
                LOG_INFO("KEY: %s\r\n",
                         PORT_ENCODER_EventToStr(evt));
            }
        }

        /* 让出 CPU，等待下一次轮询 */
        vTaskDelay(pdMS_TO_TICKS(ENC_TASK_DELAY_MS));
    }
}
