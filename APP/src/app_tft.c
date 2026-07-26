/**
 * @file    app_tft.c
 * @brief   App layer TFT multi-level menu display task
 * @note    4-layer decoupled architecture - App layer
 *          Renders ST7735 TFT menu driven by rotary encoder events.
 *          Encoder events arrive via g_menuEvtQueue across tasks:
 *            CW/CCW    -> Menu_Rotate (edit mode: change value)
 *            SHORT     -> Menu_Enter (enter submenu / leaf)
 *            LONG      -> Menu_Back (return to parent)
 *            DOUBLE    -> Menu_ToggleEdit (toggle edit mode for VALUE items)
 */

#include "app_tft.h"
#include "app_menu.h"
#include "app_tb6612.h"
#include "app_motion.h"
#include "app_gray_line.h"
#include "app_drive_mode.h"
#include "app_imu.h"
#include "app_ins.h"
#include "app_path.h"
#include "dev_tft.h"
#include "dev_menu.h"
#include "dev_encoder.h"
#include "dev_tb6612.h"
#include "port_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* ================================================================
 *  Menu content tables (static tree, add menus here only)
 * ================================================================ */

/* TB6612 motor control runtime variables */
static int16_t g_motorOn       = 0;
static int16_t g_motorSpeed    = 0;
static int16_t g_motorDir      = 0;
static int16_t g_leftSpeed     = 0;
static int16_t g_rightSpeed    = 0;

/* Helper: send motor command to tb6612_task queue */
static void motorCmdSend(MotorCmdType type, int16_t val)
{
    if (DriveMode_IsActive()) return;
    if (g_motorCmdQueue == NULL) return;
    MotorCmd cmd;
    cmd.type = type;
    cmd.val  = val;
    xQueueSend(g_motorCmdQueue, &cmd, 0);
}

static bool insCmdSend(INS_CommandType_t type)
{
    INS_Command_t cmd;

    if (g_insCmdQueue == NULL) return false;
    cmd.type = type;
    return xQueueSend(g_insCmdQueue, &cmd, 0) == pdTRUE;
}

static bool pathCmdSend(Path_CommandType_t type)
{
    Path_Command_t cmd;

    if (g_pathCmdQueue == NULL) return false;
    cmd.type = type;
    return xQueueSend(g_pathCmdQueue, &cmd, 0) == pdTRUE;
}

static bool graylineCmdSend(GrayLine_CommandType_t type, bool rateLoopEnabled)
{
    GrayLine_Command_t cmd;

    if (g_grayLineCmdQueue == NULL) return false;
    cmd.type = type;
    cmd.rate_loop_enabled = rateLoopEnabled;
    cmd.path_assist_enabled = false;
    return xQueueSend(g_grayLineCmdQueue, &cmd, 0) == pdTRUE;
}

/* TB6612 direction LEAF items: enter = send command, auto-return */
static const MenuItem dirLeafItems[] = {
    {"Forward",    MENU_LEAF, NULL, 0, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"Reverse",    MENU_LEAF, NULL, 0, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"Brake",      MENU_LEAF, NULL, 0, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"Coast",      MENU_LEAF, NULL, 0, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
};

/* Motor submenu: 7 items (scrollable beyond 4 visible lines) */
static const MenuItem motorItems[] = {
    {"Motor On/Off", MENU_VALUE,   NULL,         0, MENU_VAL_INT, {.i = &g_motorOn},   {.i = 0}, {.i = 1},   {.i = 1}},
    {"Speed",        MENU_VALUE,   NULL,         0, MENU_VAL_INT, {.i = &g_motorSpeed}, {.i = 0}, {.i = 100}, {.i = 5}},
    {"Direction",    MENU_SUBMENU, dirLeafItems, 4, MENU_VAL_INT, {.i = NULL},          {.i = 0}, {.i = 0},   {.i = 0}},
    {"Left Only",    MENU_LEAF,    NULL,         0, MENU_VAL_INT, {.i = NULL},          {.i = 0}, {.i = 0},   {.i = 0}},
    {"Right Only",   MENU_LEAF,    NULL,         0, MENU_VAL_INT, {.i = NULL},          {.i = 0}, {.i = 0},   {.i = 0}},
    {"Left Speed",   MENU_VALUE,   NULL,         0, MENU_VAL_INT, {.i = &g_leftSpeed},  {.i = 0}, {.i = 100}, {.i = 5}},
    {"Right Speed",  MENU_VALUE,   NULL,         0, MENU_VAL_INT, {.i = &g_rightSpeed}, {.i = 0}, {.i = 100}, {.i = 5}},
};
#define MOTOR_ITEM_COUNT  (sizeof(motorItems) / sizeof(motorItems[0]))

static const MenuItem straightYawPidItems[] = {
    {"Kp",      MENU_VALUE, NULL, 0, MENU_VAL_FLOAT, {.f = &g_motionPid.straight_yaw.kp},       {.f = 0.0f}, {.f = 5.0f},  {.f = 0.05f}},
    {"Ki",      MENU_VALUE, NULL, 0, MENU_VAL_FLOAT, {.f = &g_motionPid.straight_yaw.ki},       {.f = 0.0f}, {.f = 1.0f},  {.f = 0.01f}},
    {"Kd",      MENU_VALUE, NULL, 0, MENU_VAL_FLOAT, {.f = &g_motionPid.straight_yaw.kd},       {.f = 0.0f}, {.f = 1.0f},  {.f = 0.01f}},
    {"TrimMax", MENU_VALUE, NULL, 0, MENU_VAL_FLOAT, {.f = &g_motionPid.straight_yaw.trim_max}, {.f = 0.0f}, {.f = 40.0f}, {.f = 1.0f}},
};
#define STRAIGHT_YAW_PID_ITEM_COUNT  (sizeof(straightYawPidItems) / sizeof(straightYawPidItems[0]))

static const MenuItem turnYawPidItems[] = {
    {"Kp",       MENU_VALUE, NULL, 0, MENU_VAL_FLOAT, {.f = &g_motionPid.turn_yaw.kp},           {.f = 0.0f}, {.f = 5.0f},  {.f = 0.05f}},
    {"Ki",       MENU_VALUE, NULL, 0, MENU_VAL_FLOAT, {.f = &g_motionPid.turn_yaw.ki},           {.f = 0.0f}, {.f = 1.0f},  {.f = 0.01f}},
    {"Kd",       MENU_VALUE, NULL, 0, MENU_VAL_FLOAT, {.f = &g_motionPid.turn_yaw.kd},           {.f = 0.0f}, {.f = 1.0f},  {.f = 0.01f}},
    {"Deadband", MENU_VALUE, NULL, 0, MENU_VAL_FLOAT, {.f = &g_motionPid.turn_yaw.deadband_deg}, {.f = 0.5f}, {.f = 10.0f}, {.f = 0.5f}},
    {"PwmMin",   MENU_VALUE, NULL, 0, MENU_VAL_FLOAT, {.f = &g_motionPid.turn_yaw.pwm_min},      {.f = 0.0f}, {.f = 40.0f}, {.f = 1.0f}},
    {"PwmMax",   MENU_VALUE, NULL, 0, MENU_VAL_FLOAT, {.f = &g_motionPid.turn_yaw.pwm_max},      {.f = 5.0f}, {.f = 60.0f}, {.f = 1.0f}},
};
#define TURN_YAW_PID_ITEM_COUNT  (sizeof(turnYawPidItems) / sizeof(turnYawPidItems[0]))

static const MenuItem wheelSpeedPidItems[] = {
    {"Kp",      MENU_VALUE, NULL, 0, MENU_VAL_FLOAT, {.f = &g_motionPid.wheel_speed.kp},           {.f = 0.0f}, {.f = 200.0f}, {.f = 5.0f}},
    {"Ki",      MENU_VALUE, NULL, 0, MENU_VAL_FLOAT, {.f = &g_motionPid.wheel_speed.ki},           {.f = 0.0f}, {.f = 50.0f},  {.f = 0.5f}},
    {"Kd",      MENU_VALUE, NULL, 0, MENU_VAL_FLOAT, {.f = &g_motionPid.wheel_speed.kd},           {.f = 0.0f}, {.f = 50.0f},  {.f = 0.5f}},
    {"PwmTrim", MENU_VALUE, NULL, 0, MENU_VAL_FLOAT, {.f = &g_motionPid.wheel_speed.pwm_trim_max}, {.f = 0.0f}, {.f = 30.0f},  {.f = 1.0f}},
};
#define WHEEL_SPEED_PID_ITEM_COUNT  (sizeof(wheelSpeedPidItems) / sizeof(wheelSpeedPidItems[0]))

static const MenuItem arcPidItems[] = {
    {"WheelSpd",   MENU_SUBMENU, wheelSpeedPidItems, (uint8_t)WHEEL_SPEED_PID_ITEM_COUNT, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"Arc Status", MENU_LEAF,    NULL,               0,                                  MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
};
#define ARC_PID_ITEM_COUNT     (sizeof(arcPidItems) / sizeof(arcPidItems[0]))
#define ARC_PID_STATUS_INDEX   1U

static const MenuItem grayLinePidItems[] = {
    {"Kp",      MENU_VALUE, NULL, 0, MENU_VAL_FLOAT, {.f = &g_grayLinePid.kp},              {.f = 0.0f},  {.f = 0.500f}, {.f = 0.001f}},
    {"Ki",      MENU_VALUE, NULL, 0, MENU_VAL_FLOAT, {.f = &g_grayLinePid.ki},              {.f = 0.0f},  {.f = 0.100f}, {.f = 0.001f}},
    {"Kd",      MENU_VALUE, NULL, 0, MENU_VAL_FLOAT, {.f = &g_grayLinePid.kd},              {.f = 0.0f},  {.f = 0.100f}, {.f = 0.001f}},
    {"TurnMax", MENU_VALUE, NULL, 0, MENU_VAL_FLOAT, {.f = &g_grayLinePid.turn_mps_max},    {.f = 0.0f},  {.f = 0.300f}, {.f = 0.005f}},
    {"BaseMps", MENU_VALUE, NULL, 0, MENU_VAL_FLOAT, {.f = &g_grayLinePid.base_mps},        {.f = 0.02f}, {.f = 0.250f}, {.f = 0.005f}},
    {"LostMs",  MENU_VALUE, NULL, 0, MENU_VAL_FLOAT, {.f = &g_grayLinePid.lost_timeout_ms}, {.f = 50.0f}, {.f = 1000.0f}, {.f = 50.0f}},
    {"Slew",    MENU_VALUE, NULL, 0, MENU_VAL_FLOAT, {.f = &g_grayLinePid.pwm_slew},        {.f = 0.5f},  {.f = 10.0f},  {.f = 0.5f}},
    {"RevMax",  MENU_VALUE, NULL, 0, MENU_VAL_FLOAT, {.f = &g_grayLinePid.reverse_mps_max}, {.f = 0.0f},  {.f = 0.150f}, {.f = 0.005f}},
    {"RateKp",  MENU_VALUE, NULL, 0, MENU_VAL_FLOAT, {.f = &g_grayLinePid.rate_kp},         {.f = 0.0f},  {.f = 0.0200f}, {.f = 0.0001f}},
    {"RateKi",  MENU_VALUE, NULL, 0, MENU_VAL_FLOAT, {.f = &g_grayLinePid.rate_ki},         {.f = 0.0f},  {.f = 0.0200f}, {.f = 0.0001f}},
    {"RateKd",  MENU_VALUE, NULL, 0, MENU_VAL_FLOAT, {.f = &g_grayLinePid.rate_kd},         {.f = 0.0f},  {.f = 0.0200f}, {.f = 0.0001f}},
    {"RateMax", MENU_VALUE, NULL, 0, MENU_VAL_FLOAT, {.f = &g_grayLinePid.rate_dps_max},    {.f = 10.0f}, {.f = 250.0f},  {.f = 5.0f}},
    {"RateTrim",MENU_VALUE, NULL, 0, MENU_VAL_FLOAT, {.f = &g_grayLinePid.rate_trim_mps_max},{.f = 0.0f},  {.f = 0.150f}, {.f = 0.005f}},
};
#define GRAYLINE_PID_ITEM_COUNT  (sizeof(grayLinePidItems) / sizeof(grayLinePidItems[0]))

static const MenuItem pathFusionPidItems[] = {
    {"GrayWt",   MENU_VALUE, NULL, 0, MENU_VAL_FLOAT, {.f = &g_pathFusionConfig.gray_weight},            {.f = 0.0f}, {.f = 1.0f},  {.f = 0.05f}},
    {"GrayGain", MENU_VALUE, NULL, 0, MENU_VAL_FLOAT, {.f = &g_pathFusionConfig.gray_turn_to_pwm},       {.f = 0.0f}, {.f = 100.0f},{.f = 1.0f}},
    {"GrayMax",  MENU_VALUE, NULL, 0, MENU_VAL_FLOAT, {.f = &g_pathFusionConfig.gray_trim_max_pwm},      {.f = 0.0f}, {.f = 8.0f},  {.f = 0.5f}},
};
#define PATH_FUSION_PID_ITEM_COUNT (sizeof(pathFusionPidItems) / sizeof(pathFusionPidItems[0]))

static const MenuItem pidItems[] = {
    {"Straight",  MENU_SUBMENU, straightYawPidItems, (uint8_t)STRAIGHT_YAW_PID_ITEM_COUNT, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"TurnYaw",   MENU_SUBMENU, turnYawPidItems,     (uint8_t)TURN_YAW_PID_ITEM_COUNT,     MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"WheelSpd",  MENU_SUBMENU, wheelSpeedPidItems,  (uint8_t)WHEEL_SPEED_PID_ITEM_COUNT,  MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"Arc",       MENU_SUBMENU, arcPidItems,         (uint8_t)ARC_PID_ITEM_COUNT,          MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"GrayLine",  MENU_SUBMENU, grayLinePidItems,    (uint8_t)GRAYLINE_PID_ITEM_COUNT,     MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"PathFusion",MENU_SUBMENU, pathFusionPidItems,  (uint8_t)PATH_FUSION_PID_ITEM_COUNT,  MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"Yaw Status", MENU_LEAF,    NULL,                0,                                   MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
};
#define PID_ITEM_COUNT  (sizeof(pidItems) / sizeof(pidItems[0]))
#define PID_YAW_STATUS_INDEX 6U

static const MenuItem testItems[] = {
    {"Motor_Test", MENU_SUBMENU, motorItems, MOTOR_ITEM_COUNT, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
};
#define TEST_ITEM_COUNT  (sizeof(testItems) / sizeof(testItems[0]))

static const MenuItem insViewItems[] = {
    {"View",      MENU_LEAF, NULL, 0, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"Reset",     MENU_LEAF, NULL, 0, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"Rec Start", MENU_LEAF, NULL, 0, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"Rec Stop",  MENU_LEAF, NULL, 0, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"Path Save", MENU_LEAF, NULL, 0, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"Path Load", MENU_LEAF, NULL, 0, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"Path Replay", MENU_LEAF, NULL, 0, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"Fusion Start", MENU_LEAF, NULL, 0, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"Fusion Stop", MENU_LEAF, NULL, 0, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"Path Stop", MENU_LEAF, NULL, 0, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"M2 RevFix", MENU_LEAF, NULL, 0, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
};
#define INS_VIEW_ITEM_COUNT       (sizeof(insViewItems) / sizeof(insViewItems[0]))
#define INS_VIEW_PAGE_INDEX       0U
#define INS_RESET_INDEX           1U
#define INS_REC_START_INDEX       2U
#define INS_REC_STOP_INDEX        3U
#define INS_PATH_SAVE_INDEX       4U
#define INS_PATH_LOAD_INDEX       5U
#define INS_PATH_REPLAY_INDEX     6U
#define INS_FUSION_START_INDEX    7U
#define INS_FUSION_STOP_INDEX     8U
#define INS_PATH_STOP_INDEX       9U
#define INS_VIEW_M2_REVFIX_INDEX  10U

static const MenuItem grayLineItems[] = {
    {"Start",     MENU_LEAF, NULL, 0, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"Stop",      MENU_LEAF, NULL, 0, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"Rate Loop", MENU_LEAF, NULL, 0, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"Status",    MENU_LEAF, NULL, 0, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
};
#define GRAYLINE_ITEM_COUNT       (sizeof(grayLineItems) / sizeof(grayLineItems[0]))
#define GRAYLINE_START_INDEX      0U
#define GRAYLINE_STOP_INDEX       1U
#define GRAYLINE_RATE_LOOP_INDEX  2U
#define GRAYLINE_STATUS_INDEX     3U

static const MenuItem rootItems[] = {
    {"Test",       MENU_SUBMENU, testItems,    (uint8_t)TEST_ITEM_COUNT, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"PID",        MENU_SUBMENU, pidItems,     (uint8_t)PID_ITEM_COUNT, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"INS",        MENU_SUBMENU, insViewItems,  (uint8_t)INS_VIEW_ITEM_COUNT, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"GrayLine",   MENU_SUBMENU, grayLineItems, (uint8_t)GRAYLINE_ITEM_COUNT, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
};

#define ROOT_ITEM_COUNT     (sizeof(rootItems) / sizeof(rootItems[0]))

/* ================================================================
 *  Render parameters (1.8" 160x128: 8 rows total, 7 visible + 1 status)
 * ================================================================ */
#define MENU_ROW_H          16U
#define MENU_MAX_ROWS       (TFT_HEIGHT / MENU_ROW_H)        /* =8 */

/* Visible rows: full screen minus 1 status line = 7 */
#define MENU_VISIBLE_ROWS   (MENU_MAX_ROWS - 1)

#define KEY_STATUS_Y        (TFT_HEIGHT - MENU_ROW_H)
#define MENU_REFRESH_MS     200U

/* Scroll offset (module-level, reset on menu layer change via Menu_Enter / Menu_Back) */
static uint8_t g_scrollOfs = 0;
static const char *g_lastKeyMsg = "Key:--";
static bool g_insViewFirstRender = true;
static bool g_yawStatusFirstRender = true;
static bool g_arcStatusFirstRender = true;
static bool g_grayLineStatusFirstRender = true;

typedef enum {
    MENU_FEEDBACK_NONE = 0,
    MENU_FEEDBACK_INS,
    MENU_FEEDBACK_PATH
} MenuFeedbackSource_t;

typedef struct {
    MenuFeedbackSource_t source;
    uint32_t             sequence;
    uint8_t              command;
} MenuFeedback_t;

static MenuFeedback_t g_menuFeedback = {MENU_FEEDBACK_NONE, 0U, 0U};

static const char *path_cmd_name(Path_CommandType_t command)
{
    switch (command) {
        case PATH_CMD_RECORD_START: return "REC START";
        case PATH_CMD_RECORD_STOP:  return "REC STOP";
        case PATH_CMD_SAVE:         return "SAVE";
        case PATH_CMD_LOAD:         return "LOAD";
        case PATH_CMD_REPLAY:       return "REPLAY";
        case PATH_CMD_FUSION_START: return "FUSION";
        case PATH_CMD_STOP:         return "STOP";
        default:                    return "PATH";
    }
}

static const char *path_error_name(Path_Error_t error)
{
    switch (error) {
        case PATH_ERROR_INS_NOT_READY: return "INS";
        case PATH_ERROR_ACTIVE:        return "ACTIVE";
        case PATH_ERROR_TOO_SHORT:     return "POINTS";
        case PATH_ERROR_FLASH:         return "FLASH";
        case PATH_ERROR_CHECKSUM:      return "CRC";
        case PATH_ERROR_MOTION_BUSY:   return "MOTION";
        case PATH_ERROR_MOTION:        return "MOTION";
        default:                       return "FAIL";
    }
}

static void menu_watch_ins(INS_CommandType_t command)
{
    INS_ControlStatus_t status;

    g_menuFeedback.source = MENU_FEEDBACK_INS;
    g_menuFeedback.command = (uint8_t)command;
    g_menuFeedback.sequence = INS_ControlStatus_Read(&status) ? status.sequence : 0U;
    g_lastKeyMsg = "INS QUEUED";
}

static void menu_watch_path(Path_CommandType_t command)
{
    Path_RuntimeStatus_t status;

    g_menuFeedback.source = MENU_FEEDBACK_PATH;
    g_menuFeedback.command = (uint8_t)command;
    g_menuFeedback.sequence = Path_Status_Read(&status) ? status.sequence : 0U;
    g_lastKeyMsg = "PATH QUEUED";
}

static void menu_queue_ins(INS_CommandType_t command)
{
    menu_watch_ins(command);
    if (!insCmdSend(command)) {
        g_menuFeedback.source = MENU_FEEDBACK_NONE;
        g_lastKeyMsg = "INS QUEUE ERR";
    }
}

static void menu_queue_path(Path_CommandType_t command)
{
    menu_watch_path(command);
    if (!pathCmdSend(command)) {
        g_menuFeedback.source = MENU_FEEDBACK_NONE;
        g_lastKeyMsg = "PATH QUEUE ERR";
    }
}

/* ================================================================
 *  Helper: decimals from float step
 * ================================================================ */
static uint8_t decimals_from_step(float step)
{
    if (step < 0.0f) step = -step;
    if (step >= 1.0f)  return 0;
    if (step >= 0.1f)  return 1;
    if (step >= 0.01f) return 2;
    if (step >= 0.001f) return 3;
    return 4;
}

/* ================================================================
 *  Scroll helper: ensure ctx->index is visible
 * ================================================================ */
static bool scroll_update(const MenuCtx *ctx)
{
    uint8_t oldOfs = g_scrollOfs;
    uint8_t maxVis = MENU_VISIBLE_ROWS;
    if (ctx->count <= maxVis) {
        g_scrollOfs = 0;
        return (oldOfs != g_scrollOfs);
    }
    if (ctx->index >= (uint8_t)(g_scrollOfs + maxVis)) {
        g_scrollOfs = (uint8_t)(ctx->index - maxVis + 1U);
    }
    if (ctx->index < g_scrollOfs) {
        g_scrollOfs = ctx->index;
    }
    return (oldOfs != g_scrollOfs);
}

/* ================================================================
 *  Menu render (with scroll support)
 * ================================================================ */
static uint8_t menu_visible_rows(const MenuCtx *ctx)
{
    uint8_t maxVis = MENU_VISIBLE_ROWS;
    if (ctx->count < maxVis) maxVis = ctx->count;
    return maxVis;
}

static bool menu_index_to_screen_row(const MenuCtx *ctx, uint8_t logicalIndex, uint8_t *screenRow)
{
    uint8_t maxVis = menu_visible_rows(ctx);

    if (logicalIndex < g_scrollOfs ||
        logicalIndex >= (uint8_t)(g_scrollOfs + maxVis) ||
        logicalIndex >= ctx->count) {
        return false;
    }

    *screenRow = (uint8_t)(logicalIndex - g_scrollOfs);
    return true;
}

static void menu_draw_item_at_row(DevTFT *tft, const MenuCtx *ctx,
                                  uint8_t logicalIndex, uint8_t screenRow)
{
    const MenuItem *it = &ctx->current[logicalIndex];
    bool selected = (logicalIndex == ctx->index);
    uint16_t fg = TFT_WHITE;
    uint16_t bg = TFT_BLACK;
    uint16_t y = (uint16_t)(screenRow * MENU_ROW_H);

    if (selected) {
        if (ctx->editing && it->type == MENU_VALUE) {
            fg = TFT_WHITE;
            bg = TFT_ORANGE;
        } else {
            fg = TFT_WHITE;
            bg = TFT_BLUE;
        }
    }

    if (it->type == MENU_VALUE && it->valuePtr.i != NULL) {
        if (it->valType == MENU_VAL_FLOAT) {
            uint8_t dec = decimals_from_step(it->valStep.f);
            tft->printf(tft, 0, y, fg, bg, "%-9s %.*f",
                        it->title, dec, (double)(*it->valuePtr.f));
        } else {
            tft->printf(tft, 0, y, fg, bg, "%-9s %d",
                        it->title, (int)(*it->valuePtr.i));
        }
    } else {
        tft->printString(tft, 0, y, it->title, fg, bg);
    }
}

static void menu_render_item_line(DevTFT *tft, const MenuCtx *ctx, uint8_t logicalIndex)
{
    uint8_t row;

    if (!menu_index_to_screen_row(ctx, logicalIndex, &row)) {
        return;
    }

    tft->fillRect(tft, 0, (uint16_t)(row * MENU_ROW_H), TFT_WIDTH, MENU_ROW_H, TFT_BLACK);
    menu_draw_item_at_row(tft, ctx, logicalIndex, row);
}

static void menu_render_status(DevTFT *tft)
{
    tft->fillRect(tft, 0, KEY_STATUS_Y, TFT_WIDTH, MENU_ROW_H, TFT_BLACK);

    if (g_menuFeedback.source == MENU_FEEDBACK_INS) {
        INS_ControlStatus_t status;
        if (INS_ControlStatus_Read(&status) &&
            status.sequence > g_menuFeedback.sequence &&
            status.last_command == (INS_CommandType_t)g_menuFeedback.command) {
            tft->printString(tft, 0, KEY_STATUS_Y,
                (status.result == INS_CONTROL_RESULT_OK) ? "INS OK" : "INS ERR",
                (status.result == INS_CONTROL_RESULT_OK) ? TFT_GREEN : TFT_RED,
                TFT_BLACK);
            return;
        }
    } else if (g_menuFeedback.source == MENU_FEEDBACK_PATH) {
        Path_RuntimeStatus_t status;
        if (Path_Status_Read(&status) &&
            status.sequence > g_menuFeedback.sequence &&
            status.last_command == (Path_CommandType_t)g_menuFeedback.command) {
            if (status.result == PATH_RESULT_ERROR) {
                tft->printf(tft, 0, KEY_STATUS_Y, TFT_RED, TFT_BLACK, "%s ERR %s",
                            path_cmd_name(status.last_command),
                            path_error_name(status.error));
            } else if (status.result == PATH_RESULT_RUNNING) {
                tft->printf(tft, 0, KEY_STATUS_Y, TFT_YELLOW, TFT_BLACK,
                            "REPLAY %u/%u", (unsigned)status.replay_index,
                            (unsigned)status.replay_total);
            } else if (status.result == PATH_RESULT_DONE) {
                tft->printString(tft, 0, KEY_STATUS_Y, "REPLAY DONE",
                                 TFT_GREEN, TFT_BLACK);
            } else {
                tft->printf(tft, 0, KEY_STATUS_Y, TFT_GREEN, TFT_BLACK, "%s OK %u",
                            path_cmd_name(status.last_command),
                            (unsigned)status.point_count);
            }
            return;
        }
    }

    tft->printString(tft, 0, KEY_STATUS_Y, g_lastKeyMsg, TFT_GREEN, TFT_BLACK);
}

static void menu_render_full(DevTFT *tft, const MenuCtx *ctx)
{
    uint8_t maxVis;

    tft->fillScreen(tft, TFT_BLACK);
    (void)scroll_update(ctx);

    maxVis = menu_visible_rows(ctx);
    for (uint8_t i = 0; i < maxVis; i++) {
        uint8_t idx = (uint8_t)(g_scrollOfs + i);
        if (idx >= ctx->count) break;
        menu_draw_item_at_row(tft, ctx, idx, i);
    }

    menu_render_status(tft);
}

/* ================================================================
 *  Leaf placeholder page
 * ================================================================ */
static void leaf_render(DevTFT *tft, const MenuItem *it)
{
    tft->fillScreen(tft, TFT_BLACK);
    tft->printString(tft, 0, 0, it->title, TFT_YELLOW, TFT_BLACK);
    tft->printString(tft, 0, MENU_ROW_H, "(leaf page)", TFT_WHITE, TFT_BLACK);
    tft->printString(tft, 0, 2 * MENU_ROW_H, "Long=Back", TFT_GRAY, TFT_BLACK);
}

static void grayline_status_render(DevTFT *tft)
{
    static int8_t previous = -1;
    GrayLine_Status_t status;
    int8_t display;

    if (g_grayLineStatusFirstRender) {
        g_grayLineStatusFirstRender = false;
        previous = -1;
        tft->fillScreen(tft, TFT_BLACK);
        tft->printString(tft, 0, 0, "-- GrayLine --", TFT_YELLOW, TFT_BLACK);
        tft->printString(tft, 0, KEY_STATUS_Y, "Long=Back", TFT_GRAY, TFT_BLACK);
    }

    if (!GrayLine_ReadStatus(&status)) {
        return;
    }

    if (!status.rate_loop_enabled) {
        display = 0;
    } else if (status.running && !status.rate_loop_active) {
        display = 1;
    } else {
        display = 2;
    }

    if (display == previous) {
        return;
    }

    tft->fillRect(tft, 0, 3 * MENU_ROW_H, TFT_WIDTH, MENU_ROW_H, TFT_BLACK);
    if (display == 0) {
        tft->printString(tft, 0, 3 * MENU_ROW_H, "Rate Loop: OFF", TFT_GRAY, TFT_BLACK);
    } else if (display == 1) {
        tft->printString(tft, 0, 3 * MENU_ROW_H, "Rate Loop: WAIT IMU", TFT_YELLOW, TFT_BLACK);
    } else {
        tft->printString(tft, 0, 3 * MENU_ROW_H, "Rate Loop: ON", TFT_GREEN, TFT_BLACK);
    }
    previous = display;
}

/* ================================================================
 *  Check if current menu context is inside Motor subtree
 * ================================================================ */
static bool is_in_motor_menu(const MenuCtx *ctx)
{
    return (ctx->current == motorItems || ctx->current == dirLeafItems);
}

/* ================================================================
 *  Motor command dispatch
 * ================================================================ */
static void motor_dispatch(MenuCtx *ctx, bool onLeafEnter, uint8_t leafIdx, const MenuItem *cur)
{
    (void)cur;

    if (onLeafEnter) {
        if (leafIdx < 4) {
            g_motorDir = (int16_t)leafIdx;
            motorCmdSend(MOTOR_CMD_DIR, (int16_t)leafIdx);
        }
        return;
    }

    const MenuItem *it = Menu_Current(ctx);
    if (it == NULL || it->type != MENU_VALUE || it->valuePtr.i == NULL) return;

    if (it->valuePtr.i == &g_motorOn) {
        motorCmdSend(MOTOR_CMD_ONOFF, g_motorOn);
    } else if (it->valuePtr.i == &g_motorSpeed) {
        motorCmdSend(MOTOR_CMD_SPEED, g_motorSpeed);
    } else if (it->valuePtr.i == &g_leftSpeed) {
        motorCmdSend(MOTOR_CMD_LEFT_SPEED, g_leftSpeed);
    } else if (it->valuePtr.i == &g_rightSpeed) {
        motorCmdSend(MOTOR_CMD_RIGHT_SPEED, g_rightSpeed);
    }
}

/* Quick-action leaf: send command + auto-return to parent */
static bool handle_quick_leaf(const MenuItem *cur, MenuCtx *ctx)
{
    if (cur >= dirLeafItems && cur < &dirLeafItems[4]) {
        uint8_t idx = (uint8_t)(cur - dirLeafItems);
        motor_dispatch(ctx, true, idx, cur);
        Menu_Back(ctx);
        g_scrollOfs = 0;
        return true;
    }
    if (cur == &motorItems[3]) {  /* Left Only */
        motorCmdSend(MOTOR_CMD_LEFT_ONLY, 0);
        Menu_Back(ctx);
        g_scrollOfs = 0;
        return true;
    }
    if (cur == &motorItems[4]) {  /* Right Only */
        motorCmdSend(MOTOR_CMD_RIGHT_ONLY, 0);
        Menu_Back(ctx);
        g_scrollOfs = 0;
        return true;
    }
    if (cur == &insViewItems[INS_RESET_INDEX]) {
        menu_queue_ins(INS_CMD_RESET);
    } else if (cur == &insViewItems[INS_REC_START_INDEX]) {
        menu_queue_path(PATH_CMD_RECORD_START);
    } else if (cur == &insViewItems[INS_REC_STOP_INDEX]) {
        menu_queue_path(PATH_CMD_RECORD_STOP);
    } else if (cur == &insViewItems[INS_PATH_SAVE_INDEX]) {
        menu_queue_path(PATH_CMD_SAVE);
    } else if (cur == &insViewItems[INS_PATH_LOAD_INDEX]) {
        menu_queue_path(PATH_CMD_LOAD);
    } else if (cur == &insViewItems[INS_PATH_REPLAY_INDEX]) {
        g_lastKeyMsg = DriveMode_Request(DRIVE_MODE_CMD_START_PATH, 0) ?
                       "PATH START" : "MODE QUEUE ERR";
    } else if (cur == &insViewItems[INS_FUSION_START_INDEX]) {
        g_lastKeyMsg = DriveMode_Request(DRIVE_MODE_CMD_START_PATH_FUSION, 0) ?
                       "FUSION START" : "MODE QUEUE ERR";
    } else if (cur == &insViewItems[INS_FUSION_STOP_INDEX]) {
        g_lastKeyMsg = DriveMode_Request(DRIVE_MODE_CMD_STOP, 0) ?
                       "FUSION STOP" : "MODE QUEUE ERR";
    } else if (cur == &insViewItems[INS_PATH_STOP_INDEX]) {
        g_lastKeyMsg = DriveMode_Request(DRIVE_MODE_CMD_STOP, 0) ?
                       "DRIVE STOP" : "MODE QUEUE ERR";
    } else if (cur == &insViewItems[INS_VIEW_M2_REVFIX_INDEX]) {
        menu_queue_ins(INS_CMD_FLIP);
    } else if (cur == &grayLineItems[GRAYLINE_START_INDEX]) {
        g_lastKeyMsg = DriveMode_Request(DRIVE_MODE_CMD_START_GRAYLINE, 0) ?
                       "GRAY START" : "GRAY QUEUE ERR";
    } else if (cur == &grayLineItems[GRAYLINE_STOP_INDEX]) {
        g_lastKeyMsg = DriveMode_Request(DRIVE_MODE_CMD_STOP, 0) ?
                       "GRAY STOP" : "GRAY QUEUE ERR";
    } else if (cur == &grayLineItems[GRAYLINE_RATE_LOOP_INDEX]) {
        GrayLine_Status_t status;
        bool enable = false;

        if (GrayLine_ReadStatus(&status)) {
            enable = !status.rate_loop_enabled;
        }
        g_lastKeyMsg = graylineCmdSend(GRAYLINE_CMD_SET_RATE_LOOP, enable) ?
                       (enable ? "RATE LOOP ON" : "RATE LOOP OFF") :
                       "GRAY QUEUE ERR";
    } else {
        return false;
    }

        Menu_Back(ctx);
        g_scrollOfs = 0;
        return true;
}

/* ================================================================
 *  INS View leaf page (200ms auto-refresh)
 * ================================================================ */
static void ins_view_render(DevTFT *tft)
{
    static float prevX = -999.0f, prevY = -999.0f, prevM1 = -999.0f, prevM2 = -999.0f, prevYaw = -999.0f;
    static uint32_t prevPathSequence = 0xFFFFFFFFU;
    static uint16_t prevPathPoints = 0xFFFFU;
    static uint16_t prevReplayIndex = 0xFFFFU;
    static Path_State_t prevPathState = (Path_State_t)(-1);
    static DriveMode_t prevDriveMode = (DriveMode_t)(-1);

    if (g_insViewFirstRender) {
        g_insViewFirstRender = false;
        prevX = prevY = prevM1 = prevM2 = prevYaw = -999.0f;
        prevPathSequence = 0xFFFFFFFFU;
        prevPathPoints = 0xFFFFU;
        prevReplayIndex = 0xFFFFU;
        prevPathState = (Path_State_t)(-1);
        prevDriveMode = (DriveMode_t)(-1);
        tft->fillScreen(tft, TFT_BLACK);
        tft->printString(tft, 0, 0, "--- INS ---", TFT_YELLOW, TFT_BLACK);
        tft->printString(tft, 0, KEY_STATUS_Y, "Long=Back", TFT_GRAY, TFT_BLACK);
    }

    INS_Pose_t pose;
    if (!INS_Pose_Read(&pose)) {
        if (prevX == -999.0f)
            tft->printString(tft, 0, 2 * MENU_ROW_H, "Waiting INS...", TFT_RED, TFT_BLACK);
        return;
    }

    if (pose.x_m != prevX) {
        tft->fillRect(tft, 0, MENU_ROW_H, TFT_WIDTH, MENU_ROW_H, TFT_BLACK);
        tft->printf(tft, 0, MENU_ROW_H, TFT_WHITE, TFT_BLACK, "X: %+.3fm", (double)pose.x_m);
        prevX = pose.x_m;
    }
    if (pose.y_m != prevY) {
        tft->fillRect(tft, 0, 2 * MENU_ROW_H, TFT_WIDTH, MENU_ROW_H, TFT_BLACK);
        tft->printf(tft, 0, 2 * MENU_ROW_H, TFT_WHITE, TFT_BLACK, "Y: %+.3fm", (double)pose.y_m);
        prevY = pose.y_m;
    }
    if (pose.left_m != prevM1) {
        tft->fillRect(tft, 0, 3 * MENU_ROW_H, TFT_WIDTH, MENU_ROW_H, TFT_BLACK);
        tft->printf(tft, 0, 3 * MENU_ROW_H, TFT_WHITE, TFT_BLACK, "M1: %+.3fm", (double)pose.left_m);
        prevM1 = pose.left_m;
    }
    if (pose.right_m != prevM2) {
        tft->fillRect(tft, 0, 4 * MENU_ROW_H, TFT_WIDTH, MENU_ROW_H, TFT_BLACK);
        tft->printf(tft, 0, 4 * MENU_ROW_H, TFT_WHITE, TFT_BLACK, "M2: %+.3fm", (double)pose.right_m);
        prevM2 = pose.right_m;
    }
    if (pose.yaw_deg != prevYaw) {
        tft->fillRect(tft, 0, 5 * MENU_ROW_H, TFT_WIDTH, MENU_ROW_H, TFT_BLACK);
        tft->printf(tft, 0, 5 * MENU_ROW_H, TFT_WHITE, TFT_BLACK, "YAW: %+.2f deg", (double)pose.yaw_deg);
        prevYaw = pose.yaw_deg;
    }

    {
        Path_RuntimeStatus_t path;
        DriveMode_Status_t drive;
        if (Path_Status_Read(&path) &&
            DriveMode_Status_Read(&drive) &&
            (path.sequence != prevPathSequence ||
             path.point_count != prevPathPoints ||
             path.replay_index != prevReplayIndex ||
             path.state != prevPathState || drive.mode != prevDriveMode)) {
            tft->fillRect(tft, 0, 6 * MENU_ROW_H, TFT_WIDTH, MENU_ROW_H, TFT_BLACK);
            if (path.result == PATH_RESULT_ERROR) {
                tft->printf(tft, 0, 6 * MENU_ROW_H, TFT_RED, TFT_BLACK,
                            "P ERR:%s", path_error_name(path.error));
            } else if (drive.mode == DRIVE_MODE_PATH_FUSION) {
                tft->printf(tft, 0, 6 * MENU_ROW_H, TFT_YELLOW, TFT_BLACK,
                            "F%u G%+.1f P%+.1f", (unsigned)path.fusion_mode,
                            (double)path.fusion_gray_trim_pwm,
                            (double)path.fusion_path_trim_pwm);
            } else if (drive.mode == DRIVE_MODE_GRAYLINE) {
                tft->printString(tft, 0, 6 * MENU_ROW_H, "GRAY RUN", TFT_YELLOW, TFT_BLACK);
            } else if (path.result == PATH_RESULT_RUNNING ||
                       path.state == PATH_STATE_REPLAY_TRACK) {
                tft->printf(tft, 0, 6 * MENU_ROW_H, TFT_YELLOW, TFT_BLACK,
                            "P RUN:%u/%u", (unsigned)path.replay_index,
                            (unsigned)path.replay_total);
            } else if (path.result == PATH_RESULT_DONE) {
                tft->printString(tft, 0, 6 * MENU_ROW_H, "P DONE", TFT_GREEN, TFT_BLACK);
            } else if (path.state == PATH_STATE_RECORDING) {
                tft->printf(tft, 0, 6 * MENU_ROW_H, TFT_YELLOW, TFT_BLACK,
                            "P REC:%u", (unsigned)path.point_count);
            } else {
                tft->printf(tft, 0, 6 * MENU_ROW_H, TFT_GREEN, TFT_BLACK,
                            "P OK:%u", (unsigned)path.point_count);
            }
            prevPathSequence = path.sequence;
            prevPathPoints = path.point_count;
            prevReplayIndex = path.replay_index;
            prevPathState = path.state;
            prevDriveMode = drive.mode;
        }
    }
}

/* ================================================================
 *  PID Yaw Status leaf page (200ms auto-refresh, diff per-line)
 * ================================================================ */
static const char *yaw_status_state_name(Motion_RtState_t s)
{
    switch (s) {
        case MOTION_RT_FWD:  return "FWD";
        case MOTION_RT_BACK: return "BACK";
        case MOTION_RT_TURN: return "TURN";
        case MOTION_RT_ARC:  return "ARC";
        default:             return "IDLE";
    }
}

static void yaw_status_render(DevTFT *tft)
{
    static Motion_RtState_t prevState = (Motion_RtState_t)(-1);
    static float prevTarget = -999.0f, prevActual = -999.0f;

    if (g_yawStatusFirstRender) {
        g_yawStatusFirstRender = false;
        prevState = (Motion_RtState_t)(-1);
        prevTarget = prevActual = -999.0f;
        tft->fillScreen(tft, TFT_BLACK);
        tft->printString(tft, 0, 0, "--- Yaw ---", TFT_YELLOW, TFT_BLACK);
        tft->printString(tft, 0, KEY_STATUS_Y, "Long=Back", TFT_GRAY, TFT_BLACK);
    }

    if (g_motionRtStatus.state != prevState) {
        tft->fillRect(tft, 0, MENU_ROW_H, TFT_WIDTH, MENU_ROW_H, TFT_BLACK);
        tft->printf(tft, 0, MENU_ROW_H, TFT_WHITE, TFT_BLACK,
                    "St: %s", yaw_status_state_name(g_motionRtStatus.state));
        prevState = g_motionRtStatus.state;
    }
    if (g_motionRtStatus.target_yaw_deg != prevTarget) {
        tft->fillRect(tft, 0, 2 * MENU_ROW_H, TFT_WIDTH, MENU_ROW_H, TFT_BLACK);
        tft->printf(tft, 0, 2 * MENU_ROW_H, TFT_WHITE, TFT_BLACK,
                    "Tgt: %+.2f deg", (double)g_motionRtStatus.target_yaw_deg);
        prevTarget = g_motionRtStatus.target_yaw_deg;
    }
    if (g_motionRtStatus.actual_yaw_deg != prevActual) {
        tft->fillRect(tft, 0, 3 * MENU_ROW_H, TFT_WIDTH, MENU_ROW_H, TFT_BLACK);
        tft->printf(tft, 0, 3 * MENU_ROW_H, TFT_WHITE, TFT_BLACK,
                    "Act: %+.2f deg", (double)g_motionRtStatus.actual_yaw_deg);
        prevActual = g_motionRtStatus.actual_yaw_deg;
    }
}

/* ================================================================
 *  PID Arc Status leaf page (200ms auto-refresh, diff per-line)
 * ================================================================ */
static void arc_status_render(DevTFT *tft)
{
    static Motion_RtState_t prevState = (Motion_RtState_t)(-1);
    static float prevTargetLeft = -999.0f, prevActualLeft = -999.0f;
    static float prevTargetRight = -999.0f, prevActualRight = -999.0f;
    static float prevTargetYaw = -999.0f, prevActualYaw = -999.0f;

    if (g_arcStatusFirstRender) {
        g_arcStatusFirstRender = false;
        prevState = (Motion_RtState_t)(-1);
        prevTargetLeft = prevActualLeft = -999.0f;
        prevTargetRight = prevActualRight = -999.0f;
        prevTargetYaw = prevActualYaw = -999.0f;
        tft->fillScreen(tft, TFT_BLACK);
        tft->printString(tft, 0, 0, "--- Arc ---", TFT_YELLOW, TFT_BLACK);
        tft->printString(tft, 0, KEY_STATUS_Y, "Long=Back", TFT_GRAY, TFT_BLACK);
    }

    if (g_motionRtStatus.state != prevState) {
        tft->fillRect(tft, 0, MENU_ROW_H, TFT_WIDTH, MENU_ROW_H, TFT_BLACK);
        tft->printf(tft, 0, MENU_ROW_H, TFT_WHITE, TFT_BLACK,
                    "St: %s", yaw_status_state_name(g_motionRtStatus.state));
        prevState = g_motionRtStatus.state;
    }
    if (g_motionRtStatus.target_left_mps != prevTargetLeft) {
        tft->fillRect(tft, 0, 2 * MENU_ROW_H, TFT_WIDTH, MENU_ROW_H, TFT_BLACK);
        tft->printf(tft, 0, 2 * MENU_ROW_H, TFT_WHITE, TFT_BLACK,
                    "LT: %+.3f m/s", (double)g_motionRtStatus.target_left_mps);
        prevTargetLeft = g_motionRtStatus.target_left_mps;
    }
    if (g_motionRtStatus.actual_left_mps != prevActualLeft) {
        tft->fillRect(tft, 0, 3 * MENU_ROW_H, TFT_WIDTH, MENU_ROW_H, TFT_BLACK);
        tft->printf(tft, 0, 3 * MENU_ROW_H, TFT_WHITE, TFT_BLACK,
                    "LA: %+.3f m/s", (double)g_motionRtStatus.actual_left_mps);
        prevActualLeft = g_motionRtStatus.actual_left_mps;
    }
    if (g_motionRtStatus.target_right_mps != prevTargetRight) {
        tft->fillRect(tft, 0, 4 * MENU_ROW_H, TFT_WIDTH, MENU_ROW_H, TFT_BLACK);
        tft->printf(tft, 0, 4 * MENU_ROW_H, TFT_WHITE, TFT_BLACK,
                    "RT: %+.3f m/s", (double)g_motionRtStatus.target_right_mps);
        prevTargetRight = g_motionRtStatus.target_right_mps;
    }
    if (g_motionRtStatus.actual_right_mps != prevActualRight) {
        tft->fillRect(tft, 0, 5 * MENU_ROW_H, TFT_WIDTH, MENU_ROW_H, TFT_BLACK);
        tft->printf(tft, 0, 5 * MENU_ROW_H, TFT_WHITE, TFT_BLACK,
                    "RA: %+.3f m/s", (double)g_motionRtStatus.actual_right_mps);
        prevActualRight = g_motionRtStatus.actual_right_mps;
    }
    if (g_motionRtStatus.target_yaw_deg != prevTargetYaw ||
        g_motionRtStatus.actual_yaw_deg != prevActualYaw) {
        tft->fillRect(tft, 0, 6 * MENU_ROW_H, TFT_WIDTH, MENU_ROW_H, TFT_BLACK);
        tft->printf(tft, 0, 6 * MENU_ROW_H, TFT_WHITE, TFT_BLACK,
                    "Y:%+.1f/%+.1f", (double)g_motionRtStatus.target_yaw_deg,
                    (double)g_motionRtStatus.actual_yaw_deg);
        prevTargetYaw = g_motionRtStatus.target_yaw_deg;
        prevActualYaw = g_motionRtStatus.actual_yaw_deg;
    }
}

/* ================================================================
 *  TFT menu task
 * ================================================================ */
static bool tft_wait_for_system_ready(DevTFT *tft)
{
    tft->fillScreen(tft, TFT_BLACK);
    tft->printString(tft, 0, 2 * MENU_ROW_H, "System", TFT_YELLOW, TFT_BLACK);
    tft->printString(tft, 0, 3 * MENU_ROW_H, "Initializing...", TFT_YELLOW, TFT_BLACK);
    tft->printString(tft, 0, 5 * MENU_ROW_H, "IMU calibration", TFT_GRAY, TFT_BLACK);

    while (1) {
        IMU_InitStatus_t status = IMU_InitStatus_Get();

        if (status == IMU_INIT_READY) {
            tft->fillScreen(tft, TFT_BLACK);
            tft->printString(tft, 0, 2 * MENU_ROW_H, "System Ready", TFT_GREEN, TFT_BLACK);
            tft->printString(tft, 0, 3 * MENU_ROW_H, "Initialization", TFT_GREEN, TFT_BLACK);
            tft->printString(tft, 0, 4 * MENU_ROW_H, "Complete", TFT_GREEN, TFT_BLACK);
            vTaskDelay(pdMS_TO_TICKS(2000));
            return true;
        }

        if (status == IMU_INIT_FAILED) {
            tft->fillScreen(tft, TFT_BLACK);
            tft->printString(tft, 0, 2 * MENU_ROW_H, "System Init Error", TFT_RED, TFT_BLACK);
            tft->printString(tft, 0, 3 * MENU_ROW_H, "IMU failed", TFT_RED, TFT_BLACK);
            return false;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void tft_task(void *pvParameters)
{
    (void)pvParameters;

    DevTFT *tft = GetTFT();
    if (tft == NULL) {
        LOGE(LOG_MOD_TFT, "Device handle is NULL!\r\n");
        vTaskDelete(NULL);
        return;
    }

    tft->init(tft);
    LOGI_INIT(LOG_MOD_TFT, "ST7735 initialized OK\r\n");

    if (!tft_wait_for_system_ready(tft)) {
        vTaskDelete(NULL);
        return;
    }

    static MenuCtx menuCtx;
    Menu_Init(&menuCtx, rootItems, (uint8_t)ROOT_ITEM_COUNT);
    g_scrollOfs = 0;

    bool inLeaf = false;
    bool insViewLeaf = false;
    bool yawStatusLeaf = false;
    bool arcStatusLeaf = false;
    bool grayLineStatusLeaf = false;
    menu_render_full(tft, &menuCtx);

    DevEncoder_Event_t evt;
    while (1)
    {
        if (xQueueReceive(g_menuEvtQueue, &evt, pdMS_TO_TICKS(MENU_REFRESH_MS)) != pdTRUE) {
            if (insViewLeaf) {
                ins_view_render(tft);
            } else if (yawStatusLeaf) {
                yaw_status_render(tft);
            } else if (arcStatusLeaf) {
                arc_status_render(tft);
            } else if (grayLineStatusLeaf) {
                grayline_status_render(tft);
            } else {
                menu_render_status(tft);
            }
            continue;
        }

        if (inLeaf) {
            if (evt == ENCODER_EVT_KEY_LONG) {
                g_lastKeyMsg = "Key:LONG";
                inLeaf = false;
                insViewLeaf = false;
                yawStatusLeaf = false;
                arcStatusLeaf = false;
                grayLineStatusLeaf = false;
                Menu_Back(&menuCtx);
                g_scrollOfs = 0;
                menu_render_full(tft, &menuCtx);
            }
            continue;
        }

        switch (evt) {
            case ENCODER_EVT_CW: {
                uint8_t oldIndex = menuCtx.index;
                bool wasEditing = menuCtx.editing;
                Menu_Rotate(&menuCtx, +1);
                if (is_in_motor_menu(&menuCtx)) {
                    motor_dispatch(&menuCtx, false, 0, NULL);
                }
                if (wasEditing) {
                    menu_render_item_line(tft, &menuCtx, menuCtx.index);
                } else if (scroll_update(&menuCtx)) {
                    menu_render_full(tft, &menuCtx);
                } else {
                    menu_render_item_line(tft, &menuCtx, oldIndex);
                    menu_render_item_line(tft, &menuCtx, menuCtx.index);
                }
                break;
            }

            case ENCODER_EVT_CCW: {
                uint8_t oldIndex = menuCtx.index;
                bool wasEditing = menuCtx.editing;
                Menu_Rotate(&menuCtx, -1);
                if (is_in_motor_menu(&menuCtx)) {
                    motor_dispatch(&menuCtx, false, 0, NULL);
                }
                if (wasEditing) {
                    menu_render_item_line(tft, &menuCtx, menuCtx.index);
                } else if (scroll_update(&menuCtx)) {
                    menu_render_full(tft, &menuCtx);
                } else {
                    menu_render_item_line(tft, &menuCtx, oldIndex);
                    menu_render_item_line(tft, &menuCtx, menuCtx.index);
                }
                break;
            }

            case ENCODER_EVT_KEY_SHORT:
                g_lastKeyMsg = "Key:SHORT";
                menu_render_status(tft);
                if (Menu_Enter(&menuCtx)) {
                    const MenuItem *cur = Menu_Current(&menuCtx);
                    if (handle_quick_leaf(cur, &menuCtx)) {
                        menu_render_full(tft, &menuCtx);
                    } else {
                        inLeaf = true;
                        if (cur == &insViewItems[INS_VIEW_PAGE_INDEX]) {
                            g_insViewFirstRender = true;
                            insViewLeaf = true;
                            ins_view_render(tft);
                        } else if (cur == &pidItems[PID_YAW_STATUS_INDEX]) {
                            g_yawStatusFirstRender = true;
                            yawStatusLeaf = true;
                            yaw_status_render(tft);
                        } else if (cur == &arcPidItems[ARC_PID_STATUS_INDEX]) {
                            g_arcStatusFirstRender = true;
                            arcStatusLeaf = true;
                            arc_status_render(tft);
                        } else if (cur == &grayLineItems[GRAYLINE_STATUS_INDEX]) {
                            g_grayLineStatusFirstRender = true;
                            grayLineStatusLeaf = true;
                            grayline_status_render(tft);
                        } else {
                            leaf_render(tft, cur);
                        }
                    }
                } else {
                    g_scrollOfs = 0;
                    menu_render_full(tft, &menuCtx);
                }
                break;

            case ENCODER_EVT_KEY_LONG:
                g_lastKeyMsg = "Key:LONG";
                menu_render_status(tft);
                Menu_Back(&menuCtx);
                g_scrollOfs = 0;
                menu_render_full(tft, &menuCtx);
                break;

            case ENCODER_EVT_KEY_DOUBLE:
                g_lastKeyMsg = "Key:DOUBLE";
                Menu_ToggleEdit(&menuCtx);
                menu_render_item_line(tft, &menuCtx, menuCtx.index);
                menu_render_status(tft);
                break;

            default:
                break;
        }
    }
}
