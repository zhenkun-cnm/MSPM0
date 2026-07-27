/**
 * @file    app_stack_monitor.h
 * @brief   FreeRTOS task stack high-water monitor.
 */
#ifndef APP_STACK_MONITOR_H
#define APP_STACK_MONITOR_H

#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

typedef enum {
    APP_STACK_MON_START = 0,
    APP_STACK_MON_ENCODER,
    APP_STACK_MON_TFT,
    APP_STACK_MON_TB6612,
    APP_STACK_MON_MOTOR_ENC,
    APP_STACK_MON_IMU,
    APP_STACK_MON_INS,
    APP_STACK_MON_INS_CMD,
    APP_STACK_MON_CAR_COMM,
    APP_STACK_MON_MOTION,
    APP_STACK_MON_NAV,
    APP_STACK_MON_PATH,
    APP_STACK_MON_GRAY,
    APP_STACK_MON_GRAYLINE,
    APP_STACK_MON_FLASH,
    APP_STACK_MON_MONITOR,
    APP_STACK_MON_COUNT
} AppStackMon_TaskId;

void app_stack_monitor_set_task(AppStackMon_TaskId id,
                                TaskHandle_t handle,
                                uint16_t stackWords);
void app_stack_monitor_clear_task(AppStackMon_TaskId id);
void app_stack_monitor_start(void);

#endif /* APP_STACK_MONITOR_H */
