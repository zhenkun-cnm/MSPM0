/**
 * @file    app_test1.h
 * @brief   Fixed competition route test based on INS and motion.
 */
#ifndef APP_TEST1_H
#define APP_TEST1_H

#include "FreeRTOS.h"
#include "queue.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APP_TEST1_ENABLE     0
#define TEST1_CMD_QUEUE_LEN  4U

typedef enum {
    TEST1_CMD_HELP = 0,
    TEST1_CMD_START,
    TEST1_CMD_STATUS,
    TEST1_CMD_STOP
} Test1_CommandType_t;

typedef struct {
    Test1_CommandType_t type;
} Test1_Command_t;

#if APP_TEST1_ENABLE
extern QueueHandle_t g_test1CmdQueue;
void test1_task(void *pvParameters);
#endif

#ifdef __cplusplus
}
#endif

#endif /* APP_TEST1_H */
