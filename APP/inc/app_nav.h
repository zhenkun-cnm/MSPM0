/**
 * @file    app_nav.h
 * @brief   INS-based point navigation command layer.
 */
#ifndef APP_NAV_H
#define APP_NAV_H

#include "FreeRTOS.h"
#include "queue.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NAV_CMD_QUEUE_LEN 4U

typedef enum {
    NAV_STATE_IDLE = 0,
    NAV_STATE_TURN_TO_TARGET,
    NAV_STATE_DRIVE_TO_TARGET,
    NAV_STATE_TURN_TO_FINAL_YAW,
    NAV_STATE_SQUARE_DRIVE,
    NAV_STATE_SQUARE_TURN,
    NAV_STATE_DONE,
    NAV_STATE_ERROR
} Nav_State_t;

typedef enum {
    NAV_CMD_HELP = 0,
    NAV_CMD_STATUS,
    NAV_CMD_STOP,
    NAV_CMD_GOTO,
    NAV_CMD_SQUARE
} Nav_CommandType_t;

typedef struct {
    Nav_CommandType_t type;
    float x_m;
    float y_m;
    float yaw_deg;
    float side_m;
} Nav_Command_t;

extern QueueHandle_t g_navCmdQueue;

void nav_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* APP_NAV_H */
