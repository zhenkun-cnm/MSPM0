/**
 * @file    app_gray_line.h
 * @brief   Gray sensor line-following control task.
 */
#ifndef APP_GRAY_LINE_H
#define APP_GRAY_LINE_H

#include "FreeRTOS.h"
#include "queue.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GRAYLINE_CMD_QUEUE_LEN 4U

typedef struct {
    float kp;
    float ki;
    float kd;
    float turn_mps_max;
    float base_mps;
    float lost_timeout_ms;
    float pwm_slew;
    float reverse_mps_max;
} GrayLine_PidConfig_t;

typedef enum {
    GRAYLINE_CMD_HELP = 0,
    GRAYLINE_CMD_STATUS,
    GRAYLINE_CMD_START,
    GRAYLINE_CMD_STOP,
    GRAYLINE_CMD_PID
} GrayLine_CommandType_t;

typedef struct {
    GrayLine_CommandType_t type;
} GrayLine_Command_t;

typedef struct {
    bool running;
    bool line_valid;
    float target_pos;
    float line_pos;
    float error;
    float turn_mps;
    float left_target_mps;
    float left_actual_mps;
    float right_target_mps;
    float right_actual_mps;
    int16_t left_pwm;
    int16_t right_pwm;
    uint8_t active_mask;
    uint32_t seq;
} GrayLine_Status_t;

extern QueueHandle_t g_grayLineCmdQueue;
extern GrayLine_PidConfig_t g_grayLinePid;

bool GrayLine_ReadStatus(GrayLine_Status_t *out);
void grayline_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* APP_GRAY_LINE_H */
