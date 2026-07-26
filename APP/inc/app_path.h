/**
 * @file    app_path.h
 * @brief   Teach-and-replay path test based on INS pose and motion commands.
 */
#ifndef APP_PATH_H
#define APP_PATH_H

#include "FreeRTOS.h"
#include "queue.h"
#include <stdbool.h>
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
    PATH_CMD_STOP,
    PATH_CMD_SAVE,
    PATH_CMD_LOAD,
    PATH_CMD_FUSION_START
} Path_CommandType_t;

typedef struct {
    Path_CommandType_t type;
} Path_Command_t;

typedef enum {
    PATH_STATE_IDLE = 0,
    PATH_STATE_RECORDING,
    PATH_STATE_REPLAY_TURN,
    PATH_STATE_REPLAY_DRIVE,
    PATH_STATE_REPLAY_SEGMENT,
    PATH_STATE_REPLAY_TRACK,
    PATH_STATE_DONE,
    PATH_STATE_ERROR
} Path_State_t;

typedef enum {
    PATH_RESULT_NONE = 0,
    PATH_RESULT_OK,
    PATH_RESULT_RUNNING,
    PATH_RESULT_DONE,
    PATH_RESULT_ERROR
} Path_Result_t;

typedef enum {
    PATH_ERROR_NONE = 0,
    PATH_ERROR_INS_NOT_READY,
    PATH_ERROR_ACTIVE,
    PATH_ERROR_TOO_SHORT,
    PATH_ERROR_FLASH,
    PATH_ERROR_CHECKSUM,
    PATH_ERROR_MOTION_BUSY,
    PATH_ERROR_MOTION
} Path_Error_t;

typedef struct {
    float gray_weight;
    float gray_turn_to_pwm;
    float gray_trim_max_pwm;
} PathFusion_Config_t;

typedef struct {
    Path_State_t       state;
    Path_CommandType_t last_command;
    Path_Result_t      result;
    Path_Error_t       error;
    uint16_t           point_count;
    uint16_t           replay_index;
    uint16_t           replay_total;
    bool               fusion_active;
    bool               fusion_gray_ready;
    float              fusion_gray_trim_pwm;
    float              fusion_path_trim_pwm;
    float              fusion_final_trim_pwm;
    uint8_t            fusion_mode;
    uint32_t           sequence;
} Path_RuntimeStatus_t;

extern QueueHandle_t g_pathCmdQueue;
extern PathFusion_Config_t g_pathFusionConfig;

bool Path_Status_Read(Path_RuntimeStatus_t *out);
void path_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* APP_PATH_H */
