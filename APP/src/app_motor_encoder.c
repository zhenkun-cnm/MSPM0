/**
 * @file    app_motor_encoder.c
 * @brief   App 层电机编码器任务实现 (双路合并)
 * @note    每 10ms 同时采集两路编码器:
 *          Motor1: TIMG8 QEI 硬件编码器 (PB15/PB16)
 *          Motor2: TIMG7 dual capture + DMA timestamp decode (PA28/PA31)
 *
 *          全局变量:
 *            g_motorPulseCount  - Motor1 每 10ms 脉冲增量
 *            g_motor2PulseCount - Motor2 每 10ms 脉冲增量
 *
 *          Motor1: 理论 1040 脉冲/输出轴转, 实测约 1054
 *          Motor2: TIMG7 dual capture 4x decode, 实测约 1054 脉冲/输出轴转
 */
#include "app_motor_encoder.h"
#include "port_motor_encoder.h"
#include "port_motor_encoder2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "port_log.h"

/* Current encoder calibration is defined centrally in app_vehicle_config.h. */

/* 全局变量: 每 10ms 脉冲增量 */
int32_t g_motorPulseCount  = 0;
int32_t g_motor2PulseCount = 0;
static MotorEncoderSnapshot_t s_motorSnapshot = {0};
static volatile bool s_resyncRequested = false;
static int32_t s_motor2Sign = 1;

/* 轮询周期 (ms) */

bool MotorEncoder_ReadSnapshot(MotorEncoderSnapshot_t *out)
{
    if (out == NULL) {
        return false;
    }

    taskENTER_CRITICAL();
    *out = s_motorSnapshot;
    taskEXIT_CRITICAL();
    return true;
}

void MotorEncoder_RequestResync(void)
{
    taskENTER_CRITICAL();
    s_resyncRequested = true;
    taskEXIT_CRITICAL();
}

void MotorEncoder_FlipM2Direction(void)
{
    taskENTER_CRITICAL();
    s_motor2Sign = -s_motor2Sign;
    s_resyncRequested = true;
    taskEXIT_CRITICAL();
}

void motor_encoder_task(void *pvParameters)
{
    (void)pvParameters;

    /*
     * 双路编码器初始化:
     *   Motor1: 启动 TIMG8 QEI 计数器
     *   Motor2: 启动 TIMG7 + 配置 DMA_CH0 + 启动 DMA
     */
    PORT_MOTOR_ENCODER_Init();   /* Motor1: TIMG8 QEI */
    PORT_MOTOR_ENCODER2_Init();  /* Motor2: TIMG7 dual capture DMA */

    uint16_t m1_prevCount = 0;
    bool     m1_firstRun  = true;
    uint8_t  printCnt     = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        /* 精确周期 10ms */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MOTOR_ENC_PERIOD_MS));

        {
            bool doResync;

            taskENTER_CRITICAL();
            doResync = s_resyncRequested;
            s_resyncRequested = false;
            taskEXIT_CRITICAL();

            if (doResync) {
                m1_firstRun = true;
                PORT_MOTOR_ENCODER2_RequestResync();
                g_motorPulseCount = 0;
                g_motor2PulseCount = 0;
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
         *  Motor2: TIMG7 dual-capture + DMA timestamp decoder
         *  Poll() returns 4x quadrature counts already; do not scale again.
         * ================================================================ */
        {
            int32_t delta = 0;
            int32_t sign;

            PORT_MOTOR_ENCODER2_Poll(&delta);
            taskENTER_CRITICAL();
            sign = s_motor2Sign;
            taskEXIT_CRITICAL();
            g_motor2PulseCount = delta * sign;
        }

        taskENTER_CRITICAL();
        s_motorSnapshot.left_delta_counts = g_motorPulseCount;
        s_motorSnapshot.right_delta_counts = g_motor2PulseCount;
        s_motorSnapshot.left_total_counts += g_motorPulseCount;
        s_motorSnapshot.right_total_counts += g_motor2PulseCount;
        s_motorSnapshot.tick = xTaskGetTickCount();
        s_motorSnapshot.seq++;
        taskEXIT_CRITICAL();

        printCnt++;
        if (printCnt >= (100U / MOTOR_ENC_PERIOD_MS)) {
            PortMotorEncoder2Diag_t enc2Diag;
            printCnt = 0;

            if (PORT_MOTOR_ENCODER2_GetDiag(&enc2Diag)) {
                //LOGI(LOG_MOD_MOTOR, "ENC t=%lu Ld=%+ld Rd=%+ld Lt=%+ld Rt=%+ld Ao=%lu Bo=%lu AB=%u Ac=%u Bc=%u Eq=%u bad=%lu ov=%lu\r\n",
//                    (unsigned long) s_motorSnapshot.tick,
//                    (long) s_motorSnapshot.left_delta_counts,
//                    (long) s_motorSnapshot.right_delta_counts,
//                    (long) ((int32_t) s_motorSnapshot.left_total_counts),
//                    (long) ((int32_t) s_motorSnapshot.right_total_counts),
//                    (unsigned long) enc2Diag.a_dma_offset,
//                    (unsigned long) enc2Diag.b_dma_offset,
//                    (unsigned int) enc2Diag.ab_state,
//                    (unsigned int) enc2Diag.last_a_events,
//                    (unsigned int) enc2Diag.last_b_events,
//                    (unsigned int) enc2Diag.last_equal_timestamps,
//                    (unsigned long) enc2Diag.invalid_transitions,
//                    (unsigned long) enc2Diag.overrun_warnings);
            }
        }
    }
}
