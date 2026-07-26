/**
 * @file    app_drive_mode.h
 * @brief   Exclusive APP-layer drive-mode coordinator.
 */
#ifndef APP_DRIVE_MODE_H
#define APP_DRIVE_MODE_H

#include "FreeRTOS.h"
#include "queue.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DRIVE_MODE_CMD_QUEUE_LEN  4U

typedef enum {
    DRIVE_MODE_IDLE = 0,
    DRIVE_MODE_GRAYLINE,
    DRIVE_MODE_PATH,
    DRIVE_MODE_PATH_FUSION
} DriveMode_t;

typedef enum {
    DRIVE_MODE_CMD_START_GRAYLINE = 0,
    DRIVE_MODE_CMD_START_PATH,
    DRIVE_MODE_CMD_START_PATH_FUSION,
    DRIVE_MODE_CMD_STOP,
    DRIVE_MODE_CMD_STATUS
} DriveMode_CommandType_t;

typedef struct {
    DriveMode_CommandType_t type;
} DriveMode_Command_t;

typedef struct {
    DriveMode_t mode;
    DriveMode_CommandType_t last_command;
    bool transition_ok;
    uint32_t sequence;
} DriveMode_Status_t;

extern QueueHandle_t g_driveModeCmdQueue;

bool DriveMode_Request(DriveMode_CommandType_t type, TickType_t wait_ticks);
bool DriveMode_Status_Read(DriveMode_Status_t *out);
bool DriveMode_Owns(DriveMode_t mode);
bool DriveMode_IsActive(void);
void drive_mode_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* APP_DRIVE_MODE_H */
