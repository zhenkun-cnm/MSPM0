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
    float kp;
    float ki;
    float kd;
    float pwm_trim_max;
} Motion_WheelSpeedPid_t;

typedef struct {
    Motion_StraightYawPid_t straight_yaw;
    Motion_TurnYawPid_t turn_yaw;
    Motion_WheelSpeedPid_t wheel_speed;
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
    uint32_t cmd_id;
} Motion_Command_t;

extern QueueHandle_t g_motionCmdQueue;
extern Motion_PidConfig_t g_motionPid;

typedef enum {
    MOTION_RT_IDLE = 0,
    MOTION_RT_FWD,
    MOTION_RT_BACK,
    MOTION_RT_TURN
} Motion_RtState_t;

typedef enum {
    MOTION_RESULT_NONE = 0,
    MOTION_RESULT_ACCEPTED,
    MOTION_RESULT_DONE,
    MOTION_RESULT_STOPPED,
    MOTION_RESULT_REJECTED,
    MOTION_RESULT_TIMEOUT,
    MOTION_RESULT_ABORTED
} Motion_Result_t;

typedef struct {
    Motion_RtState_t state;
    float target_yaw_deg;
    float actual_yaw_deg;
    uint32_t active_cmd_id;
    uint32_t done_cmd_id;
    uint32_t rejected_cmd_id;
    Motion_Result_t last_result;
} Motion_RuntimeStatus_t;

extern Motion_RuntimeStatus_t g_motionRtStatus;

void motion_task(void *pvParameters);

#endif /* APP_MOTION_H */
