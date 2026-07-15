/**
 * @file    app_motion.h
 * @brief   Basic INS-based motion commands.
 */
#ifndef APP_MOTION_H
#define APP_MOTION_H

#include "FreeRTOS.h"
#include "queue.h"
#include <stdint.h>

#define MOTION_CMD_QUEUE_LEN 4U

typedef struct {
    float kp;
    float ki;
    float kd;
    float trim_max;
} Motion_StraightYawPid_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float deadband_deg;
    float pwm_min;
    float pwm_max;
} Motion_TurnYawPid_t;

typedef struct {
    Motion_StraightYawPid_t straight_yaw;
    Motion_TurnYawPid_t turn_yaw;
} Motion_PidConfig_t;

typedef enum {
    MOTION_CMD_HELP = 0,
    MOTION_CMD_STATUS,
    MOTION_CMD_STOP,
    MOTION_CMD_FWD,
    MOTION_CMD_BACK,
    MOTION_CMD_TURN
} Motion_CommandType_t;

typedef struct {
    Motion_CommandType_t type;
    float value;
} Motion_Command_t;

extern QueueHandle_t g_motionCmdQueue;
extern Motion_PidConfig_t g_motionPid;

void motion_task(void *pvParameters);

#endif /* APP_MOTION_H */
