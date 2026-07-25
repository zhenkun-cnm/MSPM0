/**
 * @file    app_stack_monitor.c
 * @brief   Print FreeRTOS task stack high-water marks every 100 seconds.
 */
#include "app_stack_monitor.h"
#include "portable.h"
#include "port_log.h"
#include "timers.h"
#include <stdint.h>

#define MONITOR_PERIOD_MS          100000U
#define MONITOR_STACK_DEPTH        192U

typedef struct {
    const char *name;
    TaskHandle_t handle;
    uint16_t stackWords;
} StackMonEntry;

static StackMonEntry s_tasks[APP_STACK_MON_COUNT] = {
    [APP_STACK_MON_START]     = { "start_task",   NULL, 256 },
    [APP_STACK_MON_ENCODER]   = { "encoder_task", NULL, 96 },
    [APP_STACK_MON_TFT]       = { "tft_task",     NULL, 256 },
    [APP_STACK_MON_TB6612]    = { "tb6612",       NULL, 96 },
    [APP_STACK_MON_MOTOR_ENC] = { "motor_enc",    NULL, 128 },
    [APP_STACK_MON_IMU]       = { "imu_task",     NULL, 448 },
    [APP_STACK_MON_INS]       = { "ins",          NULL, 288 },
    [APP_STACK_MON_INS_CMD]   = { "ins_cmd",      NULL, 256 },
    [APP_STACK_MON_MOTION]    = { "motion",       NULL, 192 },
    [APP_STACK_MON_NAV]       = { "nav",          NULL, 160 },
    [APP_STACK_MON_PATH]      = { "path",         NULL, 384 },
    [APP_STACK_MON_GRAY]      = { "gray",         NULL, 154 },
    [APP_STACK_MON_GRAYLINE]  = { "grayline",     NULL, 192 },
    [APP_STACK_MON_FLASH]     = { "flash_test",   NULL, 192 },
    [APP_STACK_MON_MONITOR]   = { "stack_mon",    NULL, MONITOR_STACK_DEPTH },
};

void app_stack_monitor_set_task(AppStackMon_TaskId id,
                                TaskHandle_t handle,
                                uint16_t stackWords)
{
    if (id >= APP_STACK_MON_COUNT) {
        return;
    }

    s_tasks[id].handle = handle;
    s_tasks[id].stackWords = stackWords;
}

void app_stack_monitor_clear_task(AppStackMon_TaskId id)
{
    if (id >= APP_STACK_MON_COUNT) {
        return;
    }

    s_tasks[id].handle = NULL;
}

static void print_task_stack(const StackMonEntry *entry)
{
    uint32_t remaining;
    uint32_t total;
    uint32_t used;
    uint32_t peakPct;

    if (entry == NULL || entry->handle == NULL || entry->stackWords == 0U) {
        return;
    }

    remaining = (uint32_t)uxTaskGetStackHighWaterMark(entry->handle);
    total = (uint32_t)entry->stackWords;
    used = (remaining < total) ? (total - remaining) : 0U;
    peakPct = (used * 100U) / total;

    LOGI_RELIABLE(LOG_MOD_SYS, "%-12s peak=%3lu%% used=%lu/%lu words free=%lu words\r\n",
            entry->name,
            (unsigned long)peakPct,
            (unsigned long)used,
            (unsigned long)total,
            (unsigned long)remaining);
}

static void stack_monitor_task(void *pvParameters)
{
    TickType_t lastWake = xTaskGetTickCount();

    (void)pvParameters;

    while (1) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MONITOR_PERIOD_MS));

        LOGI_RELIABLE(LOG_MOD_SYS, "----- Task Stack Usage (peak) -----\r\n");

        for (uint32_t i = 0; i < APP_STACK_MON_COUNT; i++) {
            print_task_stack(&s_tasks[i]);
        }

        LOGI_RELIABLE(LOG_MOD_SYS, "free=%lu bytes min_ever=%lu bytes\r\n",
                (unsigned long)xPortGetFreeHeapSize(),
                (unsigned long)xPortGetMinimumEverFreeHeapSize());

#if ( INCLUDE_xTaskGetIdleTaskHandle == 1 )
        {
            StackMonEntry idle = {
                "IDLE",
                xTaskGetIdleTaskHandle(),
                configMINIMAL_STACK_SIZE
            };
            print_task_stack(&idle);
        }
#endif

#if ( configUSE_TIMERS == 1 )
        {
            StackMonEntry timer = {
                "Tmr Svc",
                xTimerGetTimerDaemonTaskHandle(),
                configTIMER_TASK_STACK_DEPTH
            };
            print_task_stack(&timer);
        }
#endif
    }
}

void app_stack_monitor_start(void)
{
    TaskHandle_t handle = NULL;
    BaseType_t status = xTaskCreate(stack_monitor_task,
                                    "stack_mon",
                                    MONITOR_STACK_DEPTH,
                                    NULL,
                                    1,
                                    &handle);
    if (status != pdPASS) {
        LOGE(LOG_MOD_SYS, "monitor task create failed\r\n");
    } else {
        app_stack_monitor_set_task(APP_STACK_MON_MONITOR,
                                   handle,
                                   MONITOR_STACK_DEPTH);
    }
}
