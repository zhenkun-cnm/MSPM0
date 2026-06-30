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
 *          Motor1: 13 线 × 4 倍频 = 52 脉冲/电机转, 减速比 1:20
 *          Motor2: 13 线 × 4 倍频 = 52 脉冲/电机转, 减速比 1:30
 */
#include "app_motor_encoder.h"
#include "port_motor_encoder.h"
#include "port_motor_encoder2.h"
#include "FreeRTOS.h"
#include "task.h"

/* 全局变量: 每 10ms 脉冲增量 */
int32_t g_motorPulseCount  = 0;
int32_t g_motor2PulseCount = 0;

/* 轮询周期 (ms) */
#define MOTOR_ENC_PERIOD_MS  10U

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
                g_motorPulseCount = (int32_t)delta;
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
            g_motor2PulseCount = delta;
        }
    }
}
