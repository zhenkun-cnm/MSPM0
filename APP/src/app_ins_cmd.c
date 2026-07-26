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
#include "app_drive_mode.h"
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

typedef enum {
    SPEEDTEST_MODE_FORWARD = 0,
    SPEEDTEST_MODE_REVERSE,
    SPEEDTEST_MODE_SIGNED
} SpeedTest_Mode_t;

typedef struct {
    bool running;
    SpeedTest_Mode_t mode;
    int16_t left_pwm;
    int16_t right_pwm;
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

static void log_print_help(void)
{
    /* One record: a busy-drop logger cannot reliably emit a help page line by line. */
    LOGI_RELIABLE(LOG_MOD_CLI,
         "log: status|level error|warn|info|debug|module <name|all> on|off|telemetry motion|grayline|off\r\n");
}

static void log_print_status(void)
{
    LOGI_RELIABLE(LOG_MOD_CLI, "level=%s modules=0x%08lX telemetry=%s dropped=%lu\r\n",
          LOG_LevelName(LOG_GetLevel()),
          (unsigned long)LOG_GetModuleMask(),
          LOG_TelemetryName(LOG_GetTelemetry()),
          (unsigned long)LOG_GetDroppedCount());
}

static bool log_parse_two_words(char *text, char **first, char **second)
{
    char *cursor = text;

    while (*cursor == ' ' || *cursor == '\t') {
        cursor++;
    }
    if (*cursor == '\0') {
        return false;
    }
    *first = cursor;
    while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t') {
        cursor++;
    }
    if (*cursor == '\0') {
        return false;
    }
    *cursor++ = '\0';
    while (*cursor == ' ' || *cursor == '\t') {
        cursor++;
    }
    if (*cursor == '\0') {
        return false;
    }
    *second = cursor;
    while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t') {
        cursor++;
    }
    while (*cursor == ' ' || *cursor == '\t') {
        cursor++;
    }
    return *cursor == '\0';
}

static bool handle_log_command(char *line)
{
    LOG_Level_t level;
    LOG_Module_t module;
    LOG_Telemetry_t telemetry;
    char *name;
    char *state;

    if (str_eq(line, "log help")) {
        log_print_help();
    } else if (str_eq(line, "log status")) {
        log_print_status();
    } else if (str_prefix(line, "log level ")) {
        name = line + strlen("log level ");
        if (LOG_LevelFromName(name, &level)) {
            LOG_SetLevel(level);
            LOGI_RELIABLE(LOG_MOD_CLI, "level=%s\r\n", LOG_LevelName(level));
        } else {
            LOGI_RELIABLE(LOG_MOD_CLI, "usage: log level error|warn|info|debug\r\n");
        }
    } else if (str_prefix(line, "log module ") &&
               log_parse_two_words(line + strlen("log module "), &name, &state)) {
        if (strcmp(name, "all") == 0 && strcmp(state, "on") == 0) {
            LOG_SetModuleMask(LOG_MOD_ALL);
            LOGI_RELIABLE(LOG_MOD_CLI, "module all on\r\n");
        } else if (strcmp(name, "all") == 0 && strcmp(state, "off") == 0) {
            LOG_SetModuleMask(0U);
            LOGI_RELIABLE(LOG_MOD_CLI, "module all off\r\n");
        } else if (LOG_ModuleFromName(name, &module) &&
                   (strcmp(state, "on") == 0 || strcmp(state, "off") == 0)) {
            LOG_SetModuleEnabled(module, strcmp(state, "on") == 0);
            LOGI_RELIABLE(LOG_MOD_CLI, "module %s %s\r\n", name, state);
        } else {
            LOGI_RELIABLE(LOG_MOD_CLI, "usage: log module <name>|all on|off\r\n");
        }
    } else if (str_prefix(line, "log telemetry ")) {
        name = line + strlen("log telemetry ");
        if (!LOG_TelemetryFromName(name, &telemetry)) {
            LOGI_RELIABLE(LOG_MOD_CLI, "usage: log telemetry motion|grayline|off\r\n");
        } else if (telemetry == LOG_TELEMETRY_OFF) {
            LOG_SetTelemetry(telemetry);
            LOGI_RELIABLE(LOG_MOD_CLI, "telemetry=off\r\n");
        } else {
            LOGI_RELIABLE(LOG_MOD_CLI, "telemetry=%s (text muted)\r\n",
                 LOG_TelemetryName(telemetry));
            LOG_SetTelemetry(telemetry);
        }
    } else {
        LOGI_RELIABLE(LOG_MOD_CLI, "try: log help\r\n");
    }
    return true;
}

static bool send_ins_cmd(INS_CommandType_t type)
{
    INS_Command_t cmd;
    cmd.type = type;

    if (g_insCmdQueue == NULL) {
        LOGI_RELIABLE(LOG_MOD_CLI, "error: queue not ready\r\n");
        return false;
    }

    if (xQueueSend(g_insCmdQueue, &cmd, pdMS_TO_TICKS(20)) != pdTRUE) {
        LOGI_RELIABLE(LOG_MOD_CLI, "error: queue full\r\n");
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
        LOGI_RELIABLE(LOG_MOD_CLI, "error: queue not ready\r\n");
        return false;
    }

    if (xQueueSend(g_motionCmdQueue, &cmd, pdMS_TO_TICKS(20)) != pdTRUE) {
        LOGI_RELIABLE(LOG_MOD_CLI, "error: queue full\r\n");
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
        LOGI_RELIABLE(LOG_MOD_CLI, "error: queue not ready\r\n");
        return false;
    }

    if (xQueueSend(g_navCmdQueue, cmd, pdMS_TO_TICKS(20)) != pdTRUE) {
        LOGI_RELIABLE(LOG_MOD_CLI, "error: queue full\r\n");
        return false;
    }

    return true;
}

static bool send_path_cmd(Path_CommandType_t type)
{
    Path_Command_t cmd;
    cmd.type = type;

    if (g_pathCmdQueue == NULL) {
        LOGI_RELIABLE(LOG_MOD_CLI, "error: queue not ready\r\n");
        return false;
    }

    if (xQueueSend(g_pathCmdQueue, &cmd, pdMS_TO_TICKS(20)) != pdTRUE) {
        LOGI_RELIABLE(LOG_MOD_CLI, "error: queue full\r\n");
        return false;
    }

    return true;
}

static bool send_grayline_cmd(GrayLine_CommandType_t type)
{
    GrayLine_Command_t cmd;
    cmd.type = type;
    cmd.rate_loop_enabled = false;
    cmd.path_assist_enabled = false;

    if (g_grayLineCmdQueue == NULL) {
        LOGI_RELIABLE(LOG_MOD_CLI, "error: queue not ready\r\n");
        return false;
    }

    if (xQueueSend(g_grayLineCmdQueue, &cmd, pdMS_TO_TICKS(20)) != pdTRUE) {
        LOGI_RELIABLE(LOG_MOD_CLI, "error: queue full\r\n");
        return false;
    }

    return true;
}

static void speedtest_send_motor_cmd(MotorCmdType type, int16_t val)
{
    MotorCmd cmd;

    if (g_motorCmdQueue == NULL) {
        LOGI_RELIABLE(LOG_MOD_CLI, "error: motor queue not ready\r\n");
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

static const char *speedtest_mode_name(SpeedTest_Mode_t mode)
{
    switch (mode) {
        case SPEEDTEST_MODE_FORWARD: return "forward";
        case SPEEDTEST_MODE_REVERSE: return "reverse";
        case SPEEDTEST_MODE_SIGNED:  return "signed";
        default:                     return "unknown";
    }
}

static void speedtest_print_help(void)
{
    LOGI_RELIABLE(LOG_MOD_CLI, "commands:\r\n");
    LOGI_RELIABLE(LOG_MOD_CLI, "speedtest start <pwm 0..100>\r\n");
    LOGI_RELIABLE(LOG_MOD_CLI, "speedtest reverse <left_pwm 0..100> <right_pwm 0..100>\r\n");
    LOGI_RELIABLE(LOG_MOD_CLI, "speedtest signed <left_pwm -100..100> <right_pwm -100..100>\r\n");
    LOGI_RELIABLE(LOG_MOD_CLI, "speedtest stop\r\n");
    LOGI_RELIABLE(LOG_MOD_CLI, "speedtest status\r\n");
}

static void speedtest_start_regular(bool reverse, int16_t left_pwm, int16_t right_pwm)
{
    if (DriveMode_IsActive()) {
        LOGI_RELIABLE(LOG_MOD_CLI, "error: stop drive mode before speedtest\r\n");
        return;
    }
    memset(&s_speedTest, 0, sizeof(s_speedTest));
    s_speedTest.running = true;
    s_speedTest.mode = reverse ? SPEEDTEST_MODE_REVERSE : SPEEDTEST_MODE_FORWARD;
    s_speedTest.left_pwm = left_pwm;
    s_speedTest.right_pwm = right_pwm;
    s_speedTest.last_print_tick = xTaskGetTickCount();

    speedtest_send_motor_cmd(MOTOR_CMD_DIR,
                             (int16_t)(reverse ? TB6612_DIR_REVERSE : TB6612_DIR_FORWARD));
    speedtest_send_motor_cmd(MOTOR_CMD_LEFT_SPEED, s_speedTest.left_pwm);
    speedtest_send_motor_cmd(MOTOR_CMD_RIGHT_SPEED, s_speedTest.right_pwm);
    speedtest_send_motor_cmd(MOTOR_CMD_ONOFF, 1);
    LOGI_RELIABLE(LOG_MOD_CLI, "start mode=%s pwm=%d/%d print=%ums\r\n",
            speedtest_mode_name(s_speedTest.mode),
            (int)s_speedTest.left_pwm,
            (int)s_speedTest.right_pwm,
            (unsigned)SPEEDTEST_PRINT_MS);
}

static void speedtest_start_signed(int16_t left_pwm, int16_t right_pwm)
{
    if (DriveMode_IsActive()) {
        LOGI_RELIABLE(LOG_MOD_CLI, "error: stop drive mode before speedtest\r\n");
        return;
    }
    memset(&s_speedTest, 0, sizeof(s_speedTest));
    s_speedTest.running = true;
    s_speedTest.mode = SPEEDTEST_MODE_SIGNED;
    s_speedTest.left_pwm = clamp_pwm_arg(left_pwm);
    s_speedTest.right_pwm = clamp_pwm_arg(right_pwm);
    s_speedTest.last_print_tick = xTaskGetTickCount();

    speedtest_send_motor_cmd(MOTOR_CMD_LEFT_SIGNED_SPEED, s_speedTest.left_pwm);
    speedtest_send_motor_cmd(MOTOR_CMD_RIGHT_SIGNED_SPEED, s_speedTest.right_pwm);
    speedtest_send_motor_cmd(MOTOR_CMD_ONOFF, 1);
    LOGI_RELIABLE(LOG_MOD_CLI, "start mode=%s pwm=%d/%d print=%ums\r\n",
            speedtest_mode_name(s_speedTest.mode),
            (int)s_speedTest.left_pwm,
            (int)s_speedTest.right_pwm,
            (unsigned)SPEEDTEST_PRINT_MS);
}

static void speedtest_start(int16_t pwm)
{
    pwm = clamp_pwm_arg(pwm);
    if (pwm < 0) {
        pwm = (int16_t)-pwm;
    }
    speedtest_start_regular(false, pwm, pwm);
}

static void speedtest_stop(void)
{
    speedtest_send_motor_cmd(MOTOR_CMD_LEFT_SPEED, 0);
    speedtest_send_motor_cmd(MOTOR_CMD_RIGHT_SPEED, 0);
    speedtest_send_motor_cmd(MOTOR_CMD_LEFT_SIGNED_SPEED, 0);
    speedtest_send_motor_cmd(MOTOR_CMD_RIGHT_SIGNED_SPEED, 0);
    speedtest_send_motor_cmd(MOTOR_CMD_ONOFF, 0);
    memset(&s_speedTest, 0, sizeof(s_speedTest));
    LOGI_RELIABLE(LOG_MOD_CLI, "stop\r\n");
}

static void speedtest_print_status(void)
{
    UBaseType_t stack_free_words = uxTaskGetStackHighWaterMark(NULL);

    LOGI_RELIABLE(LOG_MOD_CLI, "run=%u mode=%s pwm=%d/%d ref=%u stack_min_free=%lu words\r\n",
            s_speedTest.running ? 1U : 0U,
            speedtest_mode_name(s_speedTest.mode),
            (int)s_speedTest.left_pwm,
            (int)s_speedTest.right_pwm,
            s_speedTest.ref_ready ? 1U : 0U,
            (unsigned long)stack_free_words);
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

    if (DriveMode_IsActive()) {
        s_speedTest.running = false;
        return;
    }
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
        LOGE(LOG_MOD_MOTOR, "encoder snapshot failed\r\n");
        return;
    }

    if (!s_speedTest.ref_ready || enc.seq == s_speedTest.prev_seq) {
        s_speedTest.prev_left_total = enc.left_total_counts;
        s_speedTest.prev_right_total = enc.right_total_counts;
        s_speedTest.prev_seq = enc.seq;
        s_speedTest.ref_ready = true;
        LOGD(LOG_MOD_MOTOR, "sync seq=%lu\r\n", (unsigned long)enc.seq);
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

    LOGD(LOG_MOD_MOTOR, "mode=%s pwm=%d/%d dt_ms=%lu lc=%ld rc=%ld lmmps=%ld rmmps=%ld ratio_x100=%ld\r\n",
            speedtest_mode_name(s_speedTest.mode),
            (int)s_speedTest.left_pwm,
            (int)s_speedTest.right_pwm,
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
        LOGD(LOG_MOD_CLI, "error: queue not ready\r\n");
        return false;
    }

    if (xQueueSend(g_test1CmdQueue, &cmd, pdMS_TO_TICKS(20)) != pdTRUE) {
        LOGD(LOG_MOD_CLI, "error: queue full\r\n");
        return false;
    }

    LOGD(LOG_MOD_CLI, "queued %s\r\n", test1_cmd_name(type));
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

static bool parse_two_long_args(const char *text, long *a, long *b)
{
    char *end = NULL;

    if (text == NULL || a == NULL || b == NULL) {
        return false;
    }

    while (*text == ' ' || *text == '\t') {
        text++;
    }
    *a = strtol(text, &end, 10);
    if (end == text) {
        return false;
    }

    text = end;
    while (*text == ' ' || *text == '\t') {
        text++;
    }
    *b = strtol(text, &end, 10);
    if (end == text) {
        return false;
    }

    while (*end == ' ' || *end == '\t') {
        end++;
    }
    return *end == '\0';
}

static void parse_line(char *line)
{
    float value;
    float value2;
    float x_m;
    float y_m;
    float yaw_deg;
    long left_pwm;
    long right_pwm;
    Nav_Command_t navCmd;

    while (*line == ' ' || *line == '\t') {
        line++;
    }

    if (str_prefix(line, "log")) {
        (void)handle_log_command(line);
    } else if (str_eq(line, "ins help") || str_eq(line, "help")) {
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
    } else if (str_eq(line, "ins flip")) {
        (void)send_ins_cmd(INS_CMD_FLIP);
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
            LOGI_RELIABLE(LOG_MOD_CLI, "error: usage motion fwd <meters>\r\n");
        }
    } else if (str_prefix(line, "motion back ")) {
        if (parse_float_arg(line + strlen("motion back "), &value)) {
            (void)send_motion_cmd(MOTION_CMD_BACK, value);
        } else {
            LOGI_RELIABLE(LOG_MOD_CLI, "error: usage motion back <meters>\r\n");
        }
    } else if (str_prefix(line, "motion turn ")) {
        if (parse_float_arg(line + strlen("motion turn "), &value)) {
            (void)send_motion_cmd(MOTION_CMD_TURN, value);
        } else {
            LOGI_RELIABLE(LOG_MOD_CLI, "error: usage motion turn <-180..180>\r\n");
        }
    } else if (str_prefix(line, "motion arc ")) {
        if (parse_two_float_args(line + strlen("motion arc "), &value, &value2)) {
            (void)send_motion_cmd2(MOTION_CMD_ARC, value, value2);
        } else {
            LOGI_RELIABLE(LOG_MOD_CLI, "error: usage motion arc <radius_m> <-180..180_deg>\r\n");
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
            LOGI_RELIABLE(LOG_MOD_CLI, "error: usage nav goto <x_m> <y_m> <yaw_deg>\r\n");
        }
    } else if (str_prefix(line, "nav square ")) {
        if (parse_float_arg(line + strlen("nav square "), &value)) {
            memset(&navCmd, 0, sizeof(navCmd));
            navCmd.type = NAV_CMD_SQUARE;
            navCmd.side_m = value;
            (void)send_nav_cmd(&navCmd);
        } else {
            LOGI_RELIABLE(LOG_MOD_CLI, "error: usage nav square <side_m>\r\n");
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
        (void)DriveMode_Request(DRIVE_MODE_CMD_START_PATH, pdMS_TO_TICKS(20));
    } else if (str_eq(line, "path fusion start")) {
        (void)DriveMode_Request(DRIVE_MODE_CMD_START_PATH_FUSION, pdMS_TO_TICKS(20));
    } else if (str_eq(line, "path fusion stop")) {
        (void)DriveMode_Request(DRIVE_MODE_CMD_STOP, pdMS_TO_TICKS(20));
    } else if (str_eq(line, "path fusion status")) {
        (void)DriveMode_Request(DRIVE_MODE_CMD_STATUS, pdMS_TO_TICKS(20));
        (void)send_path_cmd(PATH_CMD_STATUS);
    } else if (str_eq(line, "path stop")) {
        (void)DriveMode_Request(DRIVE_MODE_CMD_STOP, pdMS_TO_TICKS(20));
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
        LOGI_RELIABLE(LOG_MOD_CLI, "polarity=HIGH\r\n");
    } else if (str_eq(line, "gray polarity low")) {
        Gray_SetPolarity(GRAY_POLARITY_ACTIVE_LOW);
        LOGI_RELIABLE(LOG_MOD_CLI, "polarity=LOW\r\n");
    } else if (str_eq(line, "grayline help")) {
        (void)send_grayline_cmd(GRAYLINE_CMD_HELP);
    } else if (str_eq(line, "grayline status")) {
        (void)send_grayline_cmd(GRAYLINE_CMD_STATUS);
    } else if (str_eq(line, "grayline start")) {
        (void)DriveMode_Request(DRIVE_MODE_CMD_START_GRAYLINE, pdMS_TO_TICKS(20));
    } else if (str_eq(line, "grayline stop")) {
        (void)DriveMode_Request(DRIVE_MODE_CMD_STOP, pdMS_TO_TICKS(20));
    } else if (str_eq(line, "grayline pid")) {
        (void)send_grayline_cmd(GRAYLINE_CMD_PID);
    } else if (str_eq(line, "speedtest help")) {
        speedtest_print_help();
    } else if (str_eq(line, "speedtest status")) {
        speedtest_print_status();
    } else if (str_eq(line, "speedtest stop")) {
        speedtest_stop();
    } else if (str_prefix(line, "speedtest reverse ")) {
        if (parse_two_long_args(line + strlen("speedtest reverse "), &left_pwm, &right_pwm) &&
            left_pwm >= 0L && left_pwm <= 100L &&
            right_pwm >= 0L && right_pwm <= 100L) {
            speedtest_start_regular(true, (int16_t)left_pwm, (int16_t)right_pwm);
        } else {
            LOGI_RELIABLE(LOG_MOD_CLI, "error: usage speedtest reverse <left_pwm 0..100> <right_pwm 0..100>\r\n");
        }
    } else if (str_prefix(line, "speedtest signed ")) {
        if (parse_two_long_args(line + strlen("speedtest signed "), &left_pwm, &right_pwm) &&
            left_pwm >= -100L && left_pwm <= 100L &&
            right_pwm >= -100L && right_pwm <= 100L) {
            speedtest_start_signed((int16_t)left_pwm, (int16_t)right_pwm);
        } else {
            LOGI_RELIABLE(LOG_MOD_CLI, "error: usage speedtest signed <left_pwm -100..100> <right_pwm -100..100>\r\n");
        }
    } else if (str_prefix(line, "speedtest start ")) {
        char *end = NULL;
        long pwm = strtol(line + strlen("speedtest start "), &end, 10);
        if (end != (line + strlen("speedtest start "))) {
            speedtest_start((int16_t)pwm);
        } else {
            LOGI_RELIABLE(LOG_MOD_CLI, "error: usage speedtest start <pwm 0..100>\r\n");
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
        LOGI_RELIABLE(LOG_MOD_CLI, "disabled: set APP_TEST1_ENABLE=1\r\n");
#endif
    } else if (str_prefix(line, "motion")) {
        LOGI_RELIABLE(LOG_MOD_CLI, "unknown: %s\r\n", line);
        LOGI_RELIABLE(LOG_MOD_CLI, "try: motion help\r\n");
    } else if (str_prefix(line, "nav")) {
        LOGI_RELIABLE(LOG_MOD_CLI, "unknown: %s\r\n", line);
        LOGI_RELIABLE(LOG_MOD_CLI, "try: nav help\r\n");
    } else if (str_prefix(line, "path")) {
        LOGI_RELIABLE(LOG_MOD_CLI, "unknown: %s\r\n", line);
        LOGI_RELIABLE(LOG_MOD_CLI, "try: path help\r\n");
    } else if (str_prefix(line, "grayline")) {
        LOGI_RELIABLE(LOG_MOD_CLI, "unknown: %s\r\n", line);
        LOGI_RELIABLE(LOG_MOD_CLI, "try: grayline help\r\n");
    } else if (str_prefix(line, "gray")) {
        LOGI_RELIABLE(LOG_MOD_CLI, "unknown: %s\r\n", line);
        LOGI_RELIABLE(LOG_MOD_CLI, "try: gray help or grayline help\r\n");
    } else if (str_prefix(line, "speedtest")) {
        LOGI_RELIABLE(LOG_MOD_CLI, "unknown: %s\r\n", line);
        LOGI_RELIABLE(LOG_MOD_CLI, "try: speedtest help\r\n");
    } else if (str_prefix(line, "test1")) {
        LOGI_RELIABLE(LOG_MOD_CLI, "unknown: %s\r\n", line);
        LOGI_RELIABLE(LOG_MOD_CLI, "try: test1 help\r\n");
    } else if (line[0] != '\0') {
        LOGI_RELIABLE(LOG_MOD_CLI, "unknown: %s\r\n", line);
        LOGI_RELIABLE(LOG_MOD_CLI, "try: ins help, motion help, nav help, path help, gray help, grayline help, or test1 help\r\n");
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
        LOGE(LOG_MOD_CLI, "UART RX handle NULL\r\n");
        vTaskDelete(NULL);
        return;
    }

    uart->init(uart);
    LOGI_INIT(LOG_MOD_CLI, "UART commands ready: ins help / motion help / nav help / path help / gray help / grayline help / speedtest help / test1 help\r\n");

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
                    LOGI_RELIABLE(LOG_MOD_CLI, "error: line too long\r\n");
                }
            }
        }

        speedtest_tick();
        vTaskDelay(pdMS_TO_TICKS(INS_CMD_TASK_PERIOD_MS));
    }
}
