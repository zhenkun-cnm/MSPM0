/**
 * @file    app_ins.h
 * @brief   App layer inertial/odometry pose estimator.
 */
#ifndef APP_INS_H
#define APP_INS_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"

#ifdef __cplusplus
extern "C" {
#endif

#define INS_POSE_QUEUE_LEN  1U
#define INS_CMD_QUEUE_LEN   4U

typedef enum {
    INS_FLAG_IMU_VALID       = (1U << 0),
    INS_FLAG_YAW_ZERO_READY  = (1U << 1),
    INS_FLAG_FLASH_READY     = (1U << 2),
    INS_FLAG_FLASH_FULL      = (1U << 3),
    INS_FLAG_LOG_ENABLED     = (1U << 4),
} INS_Flags_t;

typedef enum {
    INS_CMD_HELP = 0,
    INS_CMD_STATUS,
    INS_CMD_RESET,
    INS_CMD_LOG_ON,
    INS_CMD_LOG_OFF,
    INS_CMD_LOG_PRINT,
} INS_CommandType_t;

typedef struct {
    INS_CommandType_t type;
} INS_Command_t;

typedef struct {
    float x_m;
    float y_m;
    float yaw_deg;
    float v_mps;
    float w_dps;
    float left_m;
    float right_m;
    uint32_t flags;
} INS_Pose_t;

extern QueueHandle_t g_insPoseQueue;
extern QueueHandle_t g_insCmdQueue;

void ins_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* APP_INS_H */
