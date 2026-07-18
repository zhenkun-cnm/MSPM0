/**
 * @file    app_path.h
 * @brief   Teach-and-replay path test based on INS pose and motion commands.
 */
#ifndef APP_PATH_H
#define APP_PATH_H

#include "FreeRTOS.h"
#include "queue.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PATH_CMD_QUEUE_LEN 4U

typedef enum {
    PATH_CMD_HELP = 0,
    PATH_CMD_STATUS,
    PATH_CMD_CLEAR,
    PATH_CMD_RECORD_START,
    PATH_CMD_RECORD_STOP,
    PATH_CMD_PRINT,
    PATH_CMD_REPLAY,
    PATH_CMD_STOP
} Path_CommandType_t;

typedef struct {
    Path_CommandType_t type;
} Path_Command_t;

extern QueueHandle_t g_pathCmdQueue;

void path_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* APP_PATH_H */
