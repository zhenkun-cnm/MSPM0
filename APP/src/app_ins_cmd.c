/**
 * @file    app_ins_cmd.c
 * @brief   UART line command parser for INS debug control.
 */
#include "app_ins_cmd.h"
#include "app_ins.h"
#include "app_motion.h"
#include "app_nav.h"
#include "app_path.h"
#include "app_gray.h"
#include "app_gray_line.h"
#include "app_motor_encoder.h"
#include "app_tb6612.h"
#include "app_test1.h"
#include "dev_uart_rx.h"
#include "port_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define INS_CMD_TASK_PERIOD_MS  10U
#define INS_CMD_LINE_MAX        80U
#define SPEEDTEST_PRINT_MS      100U
#define SPEEDTEST_WHEEL_CIRCUM_UM 150796L

typedef struct {
    bool running;
    int16_t pwm;
    bool ref_ready;
    int64_t prev_left_total;
    int64_t prev_right_total;
    uint32_t prev_seq;
    TickType_t last_print_tick;
} SpeedTest_State_t;

static SpeedTest_State_t s_speedTest = {0};

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

static bool send_motion_cmd2(Motion_CommandType_t type, float value, float value2)
{
    Motion_Command_t cmd;
    cmd.type = type;
    cmd.value = value;
    cmd.value2 = value2;
    cmd.cmd_id = 0U;

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

static bool send_motion_cmd(Motion_CommandType_t type, float value)
{
    return send_motion_cmd2(type, value, 0.0f);
}

static bool send_nav_cmd(const Nav_Command_t *cmd)
{
    if (cmd == NULL) {
        return false;
    }

    if (g_navCmdQueue == NULL) {
        LOG_RAW("[NAV_CMD] error: queue not ready\r\n");
        return false;
    }

    if (xQueueSend(g_navCmdQueue, cmd, pdMS_TO_TICKS(20)) != pdTRUE) {
        LOG_RAW("[NAV_CMD] error: queue full\r\n");
        return false;
    }

    return true;
}

static bool send_path_cmd(Path_CommandType_t type)
{
    Path_Command_t cmd;
    cmd.type = type;

    if (g_pathCmdQueue == NULL) {
        LOG_RAW("[PATH] error: queue not ready\r\n");
        return false;
    }

    if (xQueueSend(g_pathCmdQueue, &cmd, pdMS_TO_TICKS(20)) != pdTRUE) {
        LOG_RAW("[PATH] error: queue full\r\n");
        return false;
    }

    return true;
}

static bool send_grayline_cmd(GrayLine_CommandType_t type)
{
    GrayLine_Command_t cmd;
    cmd.type = type;

    if (g_grayLineCmdQueue == NULL) {
        LOG_RAW("[GRAYLINE] error: queue not ready\r\n");
        return false;
    }

    if (xQueueSend(g_grayLineCmdQueue, &cmd, pdMS_TO_TICKS(20)) != pdTRUE) {
        LOG_RAW("[GRAYLINE] error: queue full\r\n");
        return false;
    }

    return true;
}

static void speedtest_send_motor_cmd(MotorCmdType type, int16_t val)
{
    MotorCmd cmd;

    if (g_motorCmdQueue == NULL) {
        LOG_RAW("[SPDTEST] error: motor queue not ready\r\n");
        return;
    }

    cmd.type = type;
    cmd.val = val;
    (void)xQueueSend(g_motorCmdQueue, &cmd, pdMS_TO_TICKS(20));
}

static int32_t speedtest_counts_to_mmps(int32_t counts,
                                        int32_t counts_per_rev,
                                        uint32_t dt_ms)
{
    int64_t num;

    if (dt_ms == 0U || counts_per_rev == 0) {
        return 0;
    }

    num = (int64_t)counts * (int64_t)SPEEDTEST_WHEEL_CIRCUM_UM;
    num /= (int64_t)counts_per_rev;
    num /= (int64_t)dt_ms;
    return (int32_t)num;
}

static int16_t clamp_pwm_arg(int16_t pwm)
{
    if (pwm < -100) {
        return -100;
    }
    if (pwm > 100) {
        return 100;
    }
    return pwm;
}

static void speedtest_print_help(void)
{
    LOG_RAW("[SPDTEST] commands:\r\n");
    LOG_RAW("[SPDTEST]   speedtest start <pwm 0..100>\r\n");
    LOG_RAW("[SPDTEST]   speedtest stop\r\n");
    LOG_RAW("[SPDTEST]   speedtest status\r\n");
}

static void speedtest_start(int16_t pwm)
{
    memset(&s_speedTest, 0, sizeof(s_speedTest));
    s_speedTest.running = true;
    s_speedTest.pwm = clamp_pwm_arg(pwm);
    if (s_speedTest.pwm < 0) {
        s_speedTest.pwm = (int16_t)-s_speedTest.pwm;
    }
    s_speedTest.last_print_tick = xTaskGetTickCount();

    speedtest_send_motor_cmd(MOTOR_CMD_DIR, (int16_t)TB6612_DIR_FORWARD);
    speedtest_send_motor_cmd(MOTOR_CMD_LEFT_SPEED, s_speedTest.pwm);
    speedtest_send_motor_cmd(MOTOR_CMD_RIGHT_SPEED, s_speedTest.pwm);
    speedtest_send_motor_cmd(MOTOR_CMD_ONOFF, 1);
    LOG_RAW("[SPDTEST] start pwm=%d print=%ums\r\n",
            (int)s_speedTest.pwm,
            (unsigned)SPEEDTEST_PRINT_MS);
}

static void speedtest_stop(void)
{
    speedtest_send_motor_cmd(MOTOR_CMD_LEFT_SPEED, 0);
    speedtest_send_motor_cmd(MOTOR_CMD_RIGHT_SPEED, 0);
    speedtest_send_motor_cmd(MOTOR_CMD_ONOFF, 0);
    memset(&s_speedTest, 0, sizeof(s_speedTest));
    LOG_RAW("[SPDTEST] stop\r\n");
}

static void speedtest_print_status(void)
{
    LOG_RAW("[SPDTEST] run=%u pwm=%d ref=%u\r\n",
            s_speedTest.running ? 1U : 0U,
            (int)s_speedTest.pwm,
            s_speedTest.ref_ready ? 1U : 0U);
}

static void speedtest_tick(void)
{
    MotorEncoderSnapshot_t enc;
    TickType_t now;
    uint32_t frame_count;
    uint32_t dt_ms;
    int32_t left_counts;
    int32_t right_counts;
    int32_t left_mmps;
    int32_t right_mmps;
    int32_t ratio_x100;

    if (!s_speedTest.running) {
        return;
    }

    now = xTaskGetTickCount();
    if ((uint32_t)((now - s_speedTest.last_print_tick) * portTICK_PERIOD_MS) <
        SPEEDTEST_PRINT_MS) {
        return;
    }
    s_speedTest.last_print_tick = now;

    if (!MotorEncoder_ReadSnapshot(&enc)) {
        LOG_RAW("[SPDTEST] encoder snapshot failed\r\n");
        return;
    }

    if (!s_speedTest.ref_ready || enc.seq == s_speedTest.prev_seq) {
        s_speedTest.prev_left_total = enc.left_total_counts;
        s_speedTest.prev_right_total = enc.right_total_counts;
        s_speedTest.prev_seq = enc.seq;
        s_speedTest.ref_ready = true;
        LOG_RAW("[SPDTEST] sync seq=%lu\r\n", (unsigned long)enc.seq);
        return;
    }

    frame_count = enc.seq - s_speedTest.prev_seq;
    dt_ms = frame_count * (uint32_t)MOTOR_ENC_PERIOD_MS;
    left_counts = (int32_t)(enc.left_total_counts - s_speedTest.prev_left_total);
    right_counts = (int32_t)(enc.right_total_counts - s_speedTest.prev_right_total);
    left_mmps = speedtest_counts_to_mmps(left_counts,
                                         (int32_t)MOTOR1_COUNTS_PER_OUTPUT_REV_CAL,
                                         dt_ms);
    right_mmps = speedtest_counts_to_mmps(right_counts,
                                          (int32_t)MOTOR2_COUNTS_PER_OUTPUT_REV_CAL,
                                          dt_ms);
    ratio_x100 = (left_mmps != 0) ?
                 (int32_t)(((int64_t)right_mmps * 100LL) / (int64_t)left_mmps) :
                 0;

    LOG_RAW("[SPDTEST] pwm=%d dt_ms=%lu lc=%ld rc=%ld lmmps=%ld rmmps=%ld ratio_x100=%ld\r\n",
            (int)s_speedTest.pwm,
            (unsigned long)dt_ms,
            (long)left_counts,
            (long)right_counts,
            (long)left_mmps,
            (long)right_mmps,
            (long)ratio_x100);

    s_speedTest.prev_left_total = enc.left_total_counts;
    s_speedTest.prev_right_total = enc.right_total_counts;
    s_speedTest.prev_seq = enc.seq;
}

#if APP_TEST1_ENABLE
static const char *test1_cmd_name(Test1_CommandType_t type)
{
    switch (type) {
        case TEST1_CMD_HELP:   return "help";
        case TEST1_CMD_START:  return "start";
        case TEST1_CMD_STATUS: return "status";
        case TEST1_CMD_STOP:   return "stop";
        default:               return "?";
    }
}

static bool send_test1_cmd(Test1_CommandType_t type)
{
    Test1_Command_t cmd;
    cmd.type = type;

    if (g_test1CmdQueue == NULL) {
        log_printf_internal("[TEST1_CMD] error: queue not ready\r\n");
        return false;
    }

    if (xQueueSend(g_test1CmdQueue, &cmd, pdMS_TO_TICKS(20)) != pdTRUE) {
        log_printf_internal("[TEST1_CMD] error: queue full\r\n");
        return false;
    }

    log_printf_internal("[TEST1_CMD] queued %s\r\n", test1_cmd_name(type));
    return true;
}
#endif

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

static bool parse_three_float_args(const char *text, float *a, float *b, float *c)
{
    char *end = NULL;

    if (text == NULL || a == NULL || b == NULL || c == NULL) {
        return false;
    }

    while (*text == ' ' || *text == '\t') {
        text++;
    }
    *a = strtof(text, &end);
    if (end == text) {
        return false;
    }

    text = end;
    while (*text == ' ' || *text == '\t') {
        text++;
    }
    *b = strtof(text, &end);
    if (end == text) {
        return false;
    }

    text = end;
    while (*text == ' ' || *text == '\t') {
        text++;
    }
    *c = strtof(text, &end);
    return end != text;
}

static bool parse_two_float_args(const char *text, float *a, float *b)
{
    char *end = NULL;

    if (text == NULL || a == NULL || b == NULL) {
        return false;
    }

    while (*text == ' ' || *text == '\t') {
        text++;
    }
    *a = strtof(text, &end);
    if (end == text) {
        return false;
    }

    text = end;
    while (*text == ' ' || *text == '\t') {
        text++;
    }
    *b = strtof(text, &end);
    return end != text;
}

static void parse_line(char *line)
{
    float value;
    float value2;
    float x_m;
    float y_m;
    float yaw_deg;
    Nav_Command_t navCmd;

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
    } else if (str_prefix(line, "motion arc ")) {
        if (parse_two_float_args(line + strlen("motion arc "), &value, &value2)) {
            (void)send_motion_cmd2(MOTION_CMD_ARC, value, value2);
        } else {
            LOG_RAW("[MOTION_CMD] error: usage motion arc <radius_m> <-180..180_deg>\r\n");
        }
    } else if (str_eq(line, "nav help")) {
        memset(&navCmd, 0, sizeof(navCmd));
        navCmd.type = NAV_CMD_HELP;
        (void)send_nav_cmd(&navCmd);
    } else if (str_eq(line, "nav status")) {
        memset(&navCmd, 0, sizeof(navCmd));
        navCmd.type = NAV_CMD_STATUS;
        (void)send_nav_cmd(&navCmd);
    } else if (str_eq(line, "nav stop")) {
        memset(&navCmd, 0, sizeof(navCmd));
        navCmd.type = NAV_CMD_STOP;
        (void)send_nav_cmd(&navCmd);
    } else if (str_prefix(line, "nav goto ")) {
        if (parse_three_float_args(line + strlen("nav goto "), &x_m, &y_m, &yaw_deg)) {
            memset(&navCmd, 0, sizeof(navCmd));
            navCmd.type = NAV_CMD_GOTO;
            navCmd.x_m = x_m;
            navCmd.y_m = y_m;
            navCmd.yaw_deg = yaw_deg;
            (void)send_nav_cmd(&navCmd);
        } else {
            LOG_RAW("[NAV_CMD] error: usage nav goto <x_m> <y_m> <yaw_deg>\r\n");
        }
    } else if (str_prefix(line, "nav square ")) {
        if (parse_float_arg(line + strlen("nav square "), &value)) {
            memset(&navCmd, 0, sizeof(navCmd));
            navCmd.type = NAV_CMD_SQUARE;
            navCmd.side_m = value;
            (void)send_nav_cmd(&navCmd);
        } else {
            LOG_RAW("[NAV_CMD] error: usage nav square <side_m>\r\n");
        }
    } else if (str_eq(line, "path help")) {
        (void)send_path_cmd(PATH_CMD_HELP);
    } else if (str_eq(line, "path status")) {
        (void)send_path_cmd(PATH_CMD_STATUS);
    } else if (str_eq(line, "path clear")) {
        (void)send_path_cmd(PATH_CMD_CLEAR);
    } else if (str_eq(line, "path record start")) {
        (void)send_path_cmd(PATH_CMD_RECORD_START);
    } else if (str_eq(line, "path record stop")) {
        (void)send_path_cmd(PATH_CMD_RECORD_STOP);
    } else if (str_eq(line, "path print")) {
        (void)send_path_cmd(PATH_CMD_PRINT);
    } else if (str_eq(line, "path replay")) {
        (void)send_path_cmd(PATH_CMD_REPLAY);
    } else if (str_eq(line, "path stop")) {
        (void)send_path_cmd(PATH_CMD_STOP);
    } else if (str_eq(line, "path save") || str_eq(line, "path flash save")) {
        (void)send_path_cmd(PATH_CMD_SAVE);
    } else if (str_eq(line, "path load") || str_eq(line, "path flash load")) {
        (void)send_path_cmd(PATH_CMD_LOAD);
    } else if (str_eq(line, "gray help")) {
        Gray_PrintHelp();
    } else if (str_eq(line, "gray status")) {
        Gray_PrintStatus();
    } else if (str_eq(line, "gray polarity high")) {
        Gray_SetPolarity(GRAY_POLARITY_ACTIVE_HIGH);
        LOG_RAW("[GRAY] polarity=HIGH\r\n");
    } else if (str_eq(line, "gray polarity low")) {
        Gray_SetPolarity(GRAY_POLARITY_ACTIVE_LOW);
        LOG_RAW("[GRAY] polarity=LOW\r\n");
    } else if (str_eq(line, "grayline help")) {
        (void)send_grayline_cmd(GRAYLINE_CMD_HELP);
    } else if (str_eq(line, "grayline status")) {
        (void)send_grayline_cmd(GRAYLINE_CMD_STATUS);
    } else if (str_eq(line, "grayline start")) {
        (void)send_grayline_cmd(GRAYLINE_CMD_START);
    } else if (str_eq(line, "grayline stop")) {
        (void)send_grayline_cmd(GRAYLINE_CMD_STOP);
    } else if (str_eq(line, "grayline pid")) {
        (void)send_grayline_cmd(GRAYLINE_CMD_PID);
    } else if (str_eq(line, "speedtest help")) {
        speedtest_print_help();
    } else if (str_eq(line, "speedtest status")) {
        speedtest_print_status();
    } else if (str_eq(line, "speedtest stop")) {
        speedtest_stop();
    } else if (str_prefix(line, "speedtest start ")) {
        char *end = NULL;
        long pwm = strtol(line + strlen("speedtest start "), &end, 10);
        if (end != (line + strlen("speedtest start "))) {
            speedtest_start((int16_t)pwm);
        } else {
            LOG_RAW("[SPDTEST] error: usage speedtest start <pwm 0..100>\r\n");
        }
#if APP_TEST1_ENABLE
    } else if (str_eq(line, "test1 help")) {
        (void)send_test1_cmd(TEST1_CMD_HELP);
    } else if (str_eq(line, "test1 start") || str_eq(line, "Test1")) {
        (void)send_test1_cmd(TEST1_CMD_START);
    } else if (str_eq(line, "test1 status")) {
        (void)send_test1_cmd(TEST1_CMD_STATUS);
    } else if (str_eq(line, "test1 stop")) {
        (void)send_test1_cmd(TEST1_CMD_STOP);
#else
    } else if (str_prefix(line, "test1") || str_eq(line, "Test1")) {
        LOG_RAW("[TEST1] disabled: set APP_TEST1_ENABLE=1\r\n");
#endif
    } else if (str_prefix(line, "motion")) {
        LOG_RAW("[MOTION_CMD] unknown: %s\r\n", line);
        LOG_RAW("[MOTION_CMD] try: motion help\r\n");
    } else if (str_prefix(line, "nav")) {
        LOG_RAW("[NAV_CMD] unknown: %s\r\n", line);
        LOG_RAW("[NAV_CMD] try: nav help\r\n");
    } else if (str_prefix(line, "path")) {
        LOG_RAW("[PATH] unknown: %s\r\n", line);
        LOG_RAW("[PATH] try: path help\r\n");
    } else if (str_prefix(line, "grayline")) {
        LOG_RAW("[GRAYLINE] unknown: %s\r\n", line);
        LOG_RAW("[GRAYLINE] try: grayline help\r\n");
    } else if (str_prefix(line, "gray")) {
        LOG_RAW("[GRAY] unknown: %s\r\n", line);
        LOG_RAW("[GRAY] try: gray help or grayline help\r\n");
    } else if (str_prefix(line, "speedtest")) {
        LOG_RAW("[SPDTEST] unknown: %s\r\n", line);
        LOG_RAW("[SPDTEST] try: speedtest help\r\n");
    } else if (str_prefix(line, "test1")) {
        LOG_RAW("[TEST1] unknown: %s\r\n", line);
        LOG_RAW("[TEST1] try: test1 help\r\n");
    } else if (line[0] != '\0') {
        LOG_RAW("[INS_CMD] unknown: %s\r\n", line);
        LOG_RAW("[INS_CMD] try: ins help, motion help, nav help, path help, gray help, grayline help, or test1 help\r\n");
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
    LOG_INFO("[INS_CMD] UART commands ready: ins help / motion help / nav help / path help / gray help / grayline help / speedtest help / test1 help\r\n");

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

        speedtest_tick();
        vTaskDelay(pdMS_TO_TICKS(INS_CMD_TASK_PERIOD_MS));
    }
}
