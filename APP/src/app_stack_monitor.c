/**
 * @file    app_stack_monitor.c
 * @brief   每 10s 打印所有任务的栈峰值使用率
 */
#include "app_stack_monitor.h"
#include "port_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

#define MONITOR_PERIOD_MS         10000U
#define MONITOR_STACK_DEPTH        200U

/* ──────── 各任务栈大小（words）── 与 xTaskCreate 中保持一致 ──────── */
static const struct {
    const char *name;
    uint16_t    stackWords;
} s_tasks[] = {
    { "encoder_task", 128 },
    { "tft_task",   256 },
    { "tb6612",     128 },
    { "motor_enc",  128 },
    { "imu_task",   512 },
    { "ins",        512 },
    { "ins_cmd",    256 },
    { "motion",     384 },
    { "flash_init", 256 },
    { "IDLE",       configMINIMAL_STACK_SIZE },
    { "Tmr Svc",    configTIMER_TASK_STACK_DEPTH },
};

#define MONITOR_TASK_COUNT  (sizeof(s_tasks) / sizeof(s_tasks[0]))

static void stack_monitor_task(void *pvParameters)
{
    TickType_t lastWake = xTaskGetTickCount();

    (void)pvParameters;

    while (1) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MONITOR_PERIOD_MS));

        LOG_RAW("[STACK] ----- Task Stack Usage (peak) -----\r\n");

        for (uint32_t i = 0; i < MONITOR_TASK_COUNT; i++) {
            TaskHandle_t h = xTaskGetHandle(s_tasks[i].name);

            if (h == NULL) {
                /* 任务可能尚未创建或已删除（如 encoder_task/flash_init） */
                continue;
            }

            uint32_t remaining = (uint32_t)uxTaskGetStackHighWaterMark(h);
            uint32_t total     = (uint32_t)s_tasks[i].stackWords;
            uint32_t peakPct   = (total > 0U)
                                 ? ((total - remaining) * 100U) / total
                                 : 0U;

            LOG_RAW("[STACK] %-12s  peak=%3lu%%  (%lu/%lu words)\r\n",
                    s_tasks[i].name,
                    (unsigned long)peakPct,
                    (unsigned long)(total - remaining),
                    (unsigned long)total);
        }
    }
}

void app_stack_monitor_start(void)
{
    BaseType_t status = xTaskCreate(stack_monitor_task,
                                    "stack_mon",
                                    MONITOR_STACK_DEPTH,
                                    NULL,
                                    1,
                                    NULL);
    if (status != pdPASS) {
        LOG_ERROR("[STACK] monitor task create failed\r\n");
    } else {
        LOG_INFO("[STACK] monitor started, period=%lums\r\n",
                 (unsigned long)MONITOR_PERIOD_MS);
    }
}