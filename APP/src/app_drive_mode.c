/**
 * @file    app_drive_mode.c
 * @brief   Serializes GrayLine, path replay, and path-fusion ownership.
 */
#include "app_drive_mode.h"
#include "app_gray_line.h"
#include "app_path.h"
#include "app_tb6612.h"
#include "port_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define DRIVE_MODE_TASK_PERIOD_MS   10U
#define DRIVE_MODE_SETTLE_MS         40U

QueueHandle_t g_driveModeCmdQueue = NULL;

static DriveMode_Status_t s_status = {
    DRIVE_MODE_IDLE, DRIVE_MODE_CMD_STATUS, true, 0U
};
static TickType_t s_pathStartTick = 0U;

static const char *drive_mode_name(DriveMode_t mode)
{
    switch (mode) {
        case DRIVE_MODE_IDLE:        return "IDLE";
        case DRIVE_MODE_GRAYLINE:    return "GRAY";
        case DRIVE_MODE_PATH:        return "PATH";
        case DRIVE_MODE_PATH_FUSION: return "FUSION";
        default:                     return "?";
    }
}

static void drive_mode_publish(DriveMode_t mode,
                               DriveMode_CommandType_t command,
                               bool ok)
{
    taskENTER_CRITICAL();
    s_status.mode = mode;
    s_status.last_command = command;
    s_status.transition_ok = ok;
    s_status.sequence++;
    taskEXIT_CRITICAL();
}

bool DriveMode_Status_Read(DriveMode_Status_t *out)
{
    if (out == NULL) {
        return false;
    }

    taskENTER_CRITICAL();
    *out = s_status;
    taskEXIT_CRITICAL();
    return true;
}

bool DriveMode_Owns(DriveMode_t mode)
{
    DriveMode_t active;

    taskENTER_CRITICAL();
    active = s_status.mode;
    taskEXIT_CRITICAL();
    return active == mode;
}

bool DriveMode_IsActive(void)
{
    return !DriveMode_Owns(DRIVE_MODE_IDLE);
}

bool DriveMode_Request(DriveMode_CommandType_t type, TickType_t wait_ticks)
{
    DriveMode_Command_t cmd;

    if (g_driveModeCmdQueue == NULL) {
        return false;
    }
    cmd.type = type;
    return xQueueSend(g_driveModeCmdQueue, &cmd, wait_ticks) == pdTRUE;
}

static bool drive_mode_send_gray(GrayLine_CommandType_t type, bool assist)
{
    GrayLine_Command_t cmd;

    if (g_grayLineCmdQueue == NULL) {
        return false;
    }
    memset(&cmd, 0, sizeof(cmd));
    cmd.type = type;
    cmd.path_assist_enabled = assist;
    return xQueueSend(g_grayLineCmdQueue, &cmd, pdMS_TO_TICKS(20)) == pdTRUE;
}

static bool drive_mode_send_path(Path_CommandType_t type)
{
    Path_Command_t cmd;

    if (g_pathCmdQueue == NULL) {
        return false;
    }
    cmd.type = type;
    return xQueueSend(g_pathCmdQueue, &cmd, pdMS_TO_TICKS(20)) == pdTRUE;
}

static void drive_mode_send_motor_stop(void)
{
    MotorCmd cmd;

    if (g_motorCmdQueue == NULL) {
        return;
    }
    cmd.type = MOTOR_CMD_LEFT_SIGNED_SPEED;
    cmd.val = 0;
    (void)xQueueSend(g_motorCmdQueue, &cmd, pdMS_TO_TICKS(10));
    cmd.type = MOTOR_CMD_RIGHT_SIGNED_SPEED;
    (void)xQueueSend(g_motorCmdQueue, &cmd, pdMS_TO_TICKS(10));
    cmd.type = MOTOR_CMD_ONOFF;
    cmd.val = 0;
    (void)xQueueSend(g_motorCmdQueue, &cmd, pdMS_TO_TICKS(10));
}

static void drive_mode_stop_current(void)
{
    DriveMode_Status_t previous;

    (void)DriveMode_Status_Read(&previous);
    /* Revoke first: stale controller iterations can no longer enqueue PWM. */
    drive_mode_publish(DRIVE_MODE_IDLE, DRIVE_MODE_CMD_STOP, true);
    if (previous.mode == DRIVE_MODE_GRAYLINE) {
        (void)drive_mode_send_gray(GRAYLINE_CMD_STOP, false);
    } else if (previous.mode == DRIVE_MODE_PATH ||
               previous.mode == DRIVE_MODE_PATH_FUSION) {
        (void)drive_mode_send_path(PATH_CMD_STOP);
        if (previous.mode == DRIVE_MODE_PATH_FUSION) {
            (void)drive_mode_send_gray(GRAYLINE_CMD_STOP, false);
        }
    }
    if (previous.mode != DRIVE_MODE_IDLE) {
        drive_mode_send_motor_stop();
        vTaskDelay(pdMS_TO_TICKS(DRIVE_MODE_SETTLE_MS));
    }
}

static void drive_mode_start(DriveMode_CommandType_t command)
{
    DriveMode_t target = DRIVE_MODE_IDLE;
    bool ok = true;

    switch (command) {
        case DRIVE_MODE_CMD_START_GRAYLINE:
            target = DRIVE_MODE_GRAYLINE;
            break;
        case DRIVE_MODE_CMD_START_PATH:
            target = DRIVE_MODE_PATH;
            break;
        case DRIVE_MODE_CMD_START_PATH_FUSION:
            target = DRIVE_MODE_PATH_FUSION;
            break;
        default:
            return;
    }

    drive_mode_stop_current();
    drive_mode_publish(target, command, true);

    if (target == DRIVE_MODE_GRAYLINE) {
        ok = drive_mode_send_gray(GRAYLINE_CMD_START, false);
    } else if (target == DRIVE_MODE_PATH) {
        ok = drive_mode_send_path(PATH_CMD_REPLAY);
        s_pathStartTick = xTaskGetTickCount();
    } else {
        ok = drive_mode_send_gray(GRAYLINE_CMD_SET_PATH_ASSIST, true);
        if (ok) {
            ok = drive_mode_send_path(PATH_CMD_FUSION_START);
            s_pathStartTick = xTaskGetTickCount();
        }
    }

    if (!ok) {
        drive_mode_send_motor_stop();
        drive_mode_publish(DRIVE_MODE_IDLE, command, false);
        LOGE(LOG_MOD_SYS, "drive mode start queue failed\r\n");
    } else {
        LOGI(LOG_MOD_SYS, "drive mode=%s\r\n", drive_mode_name(target));
    }
}

static void drive_mode_check_path_terminal(void)
{
    DriveMode_Status_t mode;
    Path_RuntimeStatus_t path;

    (void)DriveMode_Status_Read(&mode);
    if ((mode.mode != DRIVE_MODE_PATH && mode.mode != DRIVE_MODE_PATH_FUSION) ||
        !Path_Status_Read(&path)) {
        return;
    }
    if ((uint32_t)((xTaskGetTickCount() - s_pathStartTick) * portTICK_PERIOD_MS) <
        DRIVE_MODE_SETTLE_MS) {
        return;
    }
    if (path.state != PATH_STATE_DONE && path.state != PATH_STATE_ERROR &&
        path.result != PATH_RESULT_ERROR) {
        return;
    }

    drive_mode_stop_current();
    LOGI(LOG_MOD_SYS, "drive mode complete -> IDLE\r\n");
}

void drive_mode_task(void *pvParameters)
{
    TickType_t lastWake = xTaskGetTickCount();

    (void)pvParameters;
    while (1) {
        DriveMode_Command_t cmd;

        while (g_driveModeCmdQueue != NULL &&
               xQueueReceive(g_driveModeCmdQueue, &cmd, 0) == pdTRUE) {
            if (cmd.type == DRIVE_MODE_CMD_STOP) {
                drive_mode_stop_current();
            } else if (cmd.type == DRIVE_MODE_CMD_STATUS) {
                DriveMode_Status_t status;
                (void)DriveMode_Status_Read(&status);
                LOGI_RELIABLE(LOG_MOD_SYS, "drive mode=%s ok=%u seq=%lu\r\n",
                              drive_mode_name(status.mode),
                              status.transition_ok ? 1U : 0U,
                              (unsigned long)status.sequence);
            } else {
                drive_mode_start(cmd.type);
            }
        }

        drive_mode_check_path_terminal();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(DRIVE_MODE_TASK_PERIOD_MS));
    }
}
