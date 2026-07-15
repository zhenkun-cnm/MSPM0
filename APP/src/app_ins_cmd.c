/**
 * @file    app_ins_cmd.c
 * @brief   UART line command parser for INS debug control.
 */
#include "app_ins_cmd.h"
#include "app_ins.h"
#include "app_motion.h"
#include "dev_uart_rx.h"
#include "port_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define INS_CMD_TASK_PERIOD_MS  10U
#define INS_CMD_LINE_MAX        48U

static bool str_eq(const char *a, const char *b)
{
    return strcmp(a, b) == 0;
}

static bool str_prefix(const char *s, const char *prefix)
{
    return strncmp(s, prefix, strlen(prefix)) == 0;
}

static bool send_ins_cmd(INS_CommandType_t type)
{
    INS_Command_t cmd;
    cmd.type = type;

    if (g_insCmdQueue == NULL) {
        LOG_RAW("[INS_CMD] error: queue not ready\r\n");
        return false;
    }

    if (xQueueSend(g_insCmdQueue, &cmd, pdMS_TO_TICKS(20)) != pdTRUE) {
        LOG_RAW("[INS_CMD] error: queue full\r\n");
        return false;
    }

    return true;
}

static bool send_motion_cmd(Motion_CommandType_t type, float value)
{
    Motion_Command_t cmd;
    cmd.type = type;
    cmd.value = value;

    if (g_motionCmdQueue == NULL) {
        LOG_RAW("[MOTION_CMD] error: queue not ready\r\n");
        return false;
    }

    if (xQueueSend(g_motionCmdQueue, &cmd, pdMS_TO_TICKS(20)) != pdTRUE) {
        LOG_RAW("[MOTION_CMD] error: queue full\r\n");
        return false;
    }

    return true;
}

static bool parse_float_arg(const char *text, float *value)
{
    char *end = NULL;

    if (text == NULL || value == NULL) {
        return false;
    }

    while (*text == ' ' || *text == '\t') {
        text++;
    }

    *value = strtof(text, &end);
    return end != text;
}

static void parse_line(char *line)
{
    float value;

    while (*line == ' ' || *line == '\t') {
        line++;
    }

    if (str_eq(line, "ins help") || str_eq(line, "help")) {
        (void)send_ins_cmd(INS_CMD_HELP);
    } else if (str_eq(line, "ins status")) {
        (void)send_ins_cmd(INS_CMD_STATUS);
    } else if (str_eq(line, "ins reset")) {
        (void)send_ins_cmd(INS_CMD_RESET);
    } else if (str_eq(line, "ins log on")) {
        (void)send_ins_cmd(INS_CMD_LOG_ON);
    } else if (str_eq(line, "ins log off")) {
        (void)send_ins_cmd(INS_CMD_LOG_OFF);
    } else if (str_eq(line, "ins log print") || str_eq(line, "ins log dump")) {
        (void)send_ins_cmd(INS_CMD_LOG_PRINT);
    } else if (str_eq(line, "motion help")) {
        (void)send_motion_cmd(MOTION_CMD_HELP, 0.0f);
    } else if (str_eq(line, "motion status")) {
        (void)send_motion_cmd(MOTION_CMD_STATUS, 0.0f);
    } else if (str_eq(line, "motion stop")) {
        (void)send_motion_cmd(MOTION_CMD_STOP, 0.0f);
    } else if (str_prefix(line, "motion fwd ")) {
        if (parse_float_arg(line + strlen("motion fwd "), &value)) {
            (void)send_motion_cmd(MOTION_CMD_FWD, value);
        } else {
            LOG_RAW("[MOTION_CMD] error: usage motion fwd <meters>\r\n");
        }
    } else if (str_prefix(line, "motion back ")) {
        if (parse_float_arg(line + strlen("motion back "), &value)) {
            (void)send_motion_cmd(MOTION_CMD_BACK, value);
        } else {
            LOG_RAW("[MOTION_CMD] error: usage motion back <meters>\r\n");
        }
    } else if (str_prefix(line, "motion turn ")) {
        if (parse_float_arg(line + strlen("motion turn "), &value)) {
            (void)send_motion_cmd(MOTION_CMD_TURN, value);
        } else {
            LOG_RAW("[MOTION_CMD] error: usage motion turn <-180..180>\r\n");
        }
    } else if (str_prefix(line, "motion")) {
        LOG_RAW("[MOTION_CMD] unknown: %s\r\n", line);
        LOG_RAW("[MOTION_CMD] try: motion help\r\n");
    } else if (line[0] != '\0') {
        LOG_RAW("[INS_CMD] unknown: %s\r\n", line);
        LOG_RAW("[INS_CMD] try: ins help or motion help\r\n");
    }
}

void ins_cmd_task(void *pvParameters)
{
    DevUartRx *uart = GetUartRx();
    char line[INS_CMD_LINE_MAX];
    memset(line, 0, sizeof(line));
    uint32_t len = 0;

    (void)pvParameters;

    if (uart == NULL) {
        LOG_ERROR("[INS_CMD] UART RX handle NULL\r\n");
        vTaskDelete(NULL);
        return;
    }

    uart->init(uart);
    LOG_INFO("[INS_CMD] UART commands ready: ins help / motion help\r\n");

    while (1) {
        uint8_t ch;

        while (uart->readByte(uart, &ch)) {
            if (ch == '\r' || ch == '\n') {
                if (len > 0U) {
                    line[len] = '\0';
                    parse_line(line);
                    len = 0;
                }
            } else if (ch == '\b' || ch == 0x7FU) {
                if (len > 0U) {
                    len--;
                }
            } else if (ch >= 32U && ch <= 126U) {
                if (len < (INS_CMD_LINE_MAX - 1U)) {
                    line[len++] = (char)ch;
                } else {
                    len = 0;
                    LOG_RAW("[INS_CMD] error: line too long\r\n");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(INS_CMD_TASK_PERIOD_MS));
    }
}
