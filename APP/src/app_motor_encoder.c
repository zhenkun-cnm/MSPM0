/**
 * @file    app_motor_encoder.c
 * @brief   App 层电机编码器任务实现 (双路合并)
 * @note    每 10ms 同时采集两路编码器:
 *          Motor1: TIMG8 QEI 硬件编码器 (PB15/PB16)
 *          Motor2: TIMG7 Capture + DMA GPIO 快照解码 (PA28/PA29)
 *
 *          全局变量:
 *            g_motorPulseCount  - Motor1 每 10ms 脉冲增量
 *            g_motor2PulseCount - Motor2 每 10ms 脉冲增量
 *
 *          Motor1: 理论 1040 脉冲/输出轴转, 实测约 1054
 *          Motor2: A 相上升沿 1x 捕获后软件 ×4, 实测约 985 脉冲/输出轴转
 */
#include "app_motor_encoder.h"
#include "port_motor_encoder.h"
#include "port_motor_encoder2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "port_log.h"

/* 全局变量: 每 10ms 脉冲增量 */
int32_t g_motorPulseCount  = 0;
int32_t g_motor2PulseCount = 0;
QueueHandle_t g_motorOdomQueue = NULL;

/* 轮询周期 (ms) */

void motor_encoder_task(void *pvParameters)
{
    (void)pvParameters;

    /*
     * 双路编码器初始化:
     *   Motor1: 启动 TIMG8 QEI 计数器
     *   Motor2: 启动 TIMG7 + 配置 DMA_CH0 + 启动 DMA
     */
    PORT_MOTOR_ENCODER_Init();   /* Motor1: TIMG8 QEI */
    PORT_MOTOR_ENCODER2_Init();  /* Motor2: DMA-GPIO */

    uint16_t m1_prevCount = 0;
    bool     m1_firstRun  = true;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        /* 精确周期 10ms */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MOTOR_ENC_PERIOD_MS));

        /* 100ms 周期串口打印编码器脉冲累积（带正负），打印后清零 */
        {
            static int32_t m1Accum = 0;
            static int32_t m2Accum = 0;
            static uint8_t printCnt = 0;

            m1Accum += g_motorPulseCount;
            m2Accum += g_motor2PulseCount;
            printCnt++;

             if (printCnt >= 1)
             {
                 //LOG_RAW("M1:%+ld M2:%+ld\r\n", (long)m1Accum, (long)m2Accum);
                 m1Accum  = 0;
                 m2Accum  = 0;
                 printCnt = 0;
             }
        }

        /* ================================================================
         *  Motor1: TIMG8 QEI 硬件编码器
         *  读取 16-bit 计数器, int16_t 差值自动处理回绕
         * ================================================================ */
        {
            uint16_t curCount = PORT_MOTOR_ENCODER_GetCount();

            if (m1_firstRun)
            {
                m1_prevCount    = curCount;
                m1_firstRun     = false;
                g_motorPulseCount = 0;
            }
            else
            {
                int16_t delta = (int16_t)(curCount - m1_prevCount);
                g_motorPulseCount = -(int32_t)delta;
                m1_prevCount = curCount;
            }
        }

        /* ================================================================
         *  Motor2: DMA-GPIO 快照环形缓冲区解码
         *  Poll() 内部遍历新快照, 按 bit28=A相 bit29=B相 判向计数
         * ================================================================ */
        {
            int32_t delta = 0;
            PORT_MOTOR_ENCODER2_Poll(&delta);
            /* M2 DMA 仅捕获 A 相上升沿(1×)，软件补偿为 4× 以对齐 M1 QEI */
            g_motor2PulseCount = delta * MOTOR2_ENCODER_SOFTWARE_SCALE;
        }

        if (g_motorOdomQueue != NULL) {
            MotorOdomDelta_t odom;
            odom.left_counts = g_motorPulseCount;
            odom.right_counts = g_motor2PulseCount;
            odom.tick = xTaskGetTickCount();
            if (xQueueSend(g_motorOdomQueue, &odom, 0) != pdTRUE) {
                MotorOdomDelta_t dropped;
                (void)xQueueReceive(g_motorOdomQueue, &dropped, 0);
                (void)xQueueSend(g_motorOdomQueue, &odom, 0);
            }
        }
    }
}
