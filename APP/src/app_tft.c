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
#include "app_ins.h"
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

/* Runtime mutable values (non-const, editable in edit mode) */
static int16_t g_brightness = 50;
static int16_t g_contrast   = 50;
static float   g_temp       = 23.5f;
static float   g_testAuto   = 0.0f;

/* TB6612 motor control runtime variables */
static int16_t g_motorOn       = 0;
static int16_t g_motorSpeed    = 0;
static int16_t g_motorDir      = 0;
static int16_t g_leftSpeed     = 0;
static int16_t g_rightSpeed    = 0;

/* Helper: send motor command to tb6612_task queue */
static void motorCmdSend(MotorCmdType type, int16_t val)
{
    if (g_motorCmdQueue == NULL) return;
    MotorCmd cmd;
    cmd.type = type;
    cmd.val  = val;
    xQueueSend(g_motorCmdQueue, &cmd, 0);
}

static const MenuItem settingsItems[] = {
    {"Brightness", MENU_VALUE,   NULL, 0, MENU_VAL_INT,   {.i = &g_brightness}, {.i = 0}, {.i = 100}, {.i = 5}},
    {"Contrast",   MENU_VALUE,   NULL, 0, MENU_VAL_INT,   {.i = &g_contrast},   {.i = 0}, {.i = 100}, {.i = 5}},
    {"Temp",       MENU_VALUE,   NULL, 0, MENU_VAL_FLOAT, {.f = &g_temp},       {.f = 0.0f}, {.f = 50.0f}, {.f = 0.5f}},
    {"Back-Test",  MENU_LEAF,    NULL, 0, MENU_VAL_INT,   {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
};

static const MenuItem sensorItems[] = {
    {"IMU Data",   MENU_LEAF,    NULL, 0, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"Encoder",    MENU_LEAF,    NULL, 0, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
};

static const MenuItem aboutItems[] = {
    {"Version",    MENU_LEAF,    NULL, 0, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"Author",     MENU_LEAF,    NULL, 0, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
};

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

static const MenuItem pidItems[] = {
    {"Straight",  MENU_SUBMENU, straightYawPidItems, (uint8_t)STRAIGHT_YAW_PID_ITEM_COUNT, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"TurnYaw",   MENU_SUBMENU, turnYawPidItems,     (uint8_t)TURN_YAW_PID_ITEM_COUNT,     MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"WheelSpd",  MENU_SUBMENU, wheelSpeedPidItems,  (uint8_t)WHEEL_SPEED_PID_ITEM_COUNT,  MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"Arc",       MENU_SUBMENU, arcPidItems,         (uint8_t)ARC_PID_ITEM_COUNT,          MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"Yaw Status", MENU_LEAF,    NULL,                0,                                   MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
};
#define PID_ITEM_COUNT  (sizeof(pidItems) / sizeof(pidItems[0]))
#define PID_YAW_STATUS_INDEX 4U

static const MenuItem testItems[] = {
    {"LED Test",    MENU_LEAF,    NULL,        0, MENU_VAL_INT,   {.i = NULL},        {.i = 0},    {.i = 0},     {.i = 0}},
    {"Encoder",     MENU_LEAF,    NULL,        0, MENU_VAL_INT,   {.i = NULL},        {.i = 0},    {.i = 0},     {.i = 0}},
    {"Button",      MENU_LEAF,    NULL,        0, MENU_VAL_INT,   {.i = NULL},        {.i = 0},    {.i = 0},     {.i = 0}},
    {"Motor",       MENU_SUBMENU, motorItems,  MOTOR_ITEM_COUNT, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"Display",     MENU_LEAF,    NULL,        0, MENU_VAL_INT,   {.i = NULL},        {.i = 0},    {.i = 0},     {.i = 0}},
    {"IMU View",    MENU_LEAF,    NULL,        0, MENU_VAL_INT,   {.i = NULL},        {.i = 0},    {.i = 0},     {.i = 0}},
    {"Yaw View",    MENU_LEAF,    NULL,        0, MENU_VAL_INT,   {.i = NULL},        {.i = 0},    {.i = 0},     {.i = 0}},
    {"Odom View",   MENU_LEAF,    NULL,        0, MENU_VAL_INT,   {.i = NULL},        {.i = 0},    {.i = 0},     {.i = 0}},
    {"Flash View",  MENU_LEAF,    NULL,        0, MENU_VAL_INT,   {.i = NULL},        {.i = 0},    {.i = 0},     {.i = 0}},
    {"AutoTest",    MENU_VALUE,   NULL,        0, MENU_VAL_FLOAT, {.f = &g_testAuto}, {.f = 0.0f}, {.f = 999.9f}, {.f = 0.1f}},
};
#define TEST_AUTO_INDEX  9U
#define TEST_ITEM_COUNT  (sizeof(testItems) / sizeof(testItems[0]))

static const MenuItem rootItems[] = {
    {"Test",       MENU_SUBMENU, testItems,    (uint8_t)TEST_ITEM_COUNT, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"PID",        MENU_SUBMENU, pidItems,     (uint8_t)PID_ITEM_COUNT, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"Settings",   MENU_SUBMENU, settingsItems, 4, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"Sensors",    MENU_SUBMENU, sensorItems,   2, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"INS View",   MENU_LEAF,    NULL,          0, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"About",      MENU_SUBMENU, aboutItems,    3, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
};

#define ROOT_ITEM_COUNT     (sizeof(rootItems) / sizeof(rootItems[0]))
#define ROOT_INS_VIEW_INDEX 4U

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

/* ================================================================
 *  Helper: decimals from float step
 * ================================================================ */
static uint8_t decimals_from_step(float step)
{
    if (step < 0.0f) step = -step;
    if (step >= 1.0f)  return 0;
    if (step >= 0.1f)  return 1;
    return 2;
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
    return false;
}

static void auto_test_tick(DevTFT *tft, const MenuCtx *ctx, bool inLeaf)
{
    g_testAuto += 0.1f;
    if (g_testAuto > 999.9f) {
        g_testAuto = 0.0f;
    }

    if (!inLeaf && ctx->current == testItems) {
        menu_render_item_line(tft, ctx, TEST_AUTO_INDEX);
    }
}

/* ================================================================
 *  INS View leaf page (200ms auto-refresh)
 * ================================================================ */
static void ins_view_render(DevTFT *tft)
{
    static float prevX = -999.0f, prevY = -999.0f, prevM1 = -999.0f, prevM2 = -999.0f, prevYaw = -999.0f;

    if (g_insViewFirstRender) {
        g_insViewFirstRender = false;
        prevX = prevY = prevM1 = prevM2 = prevYaw = -999.0f;
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
void tft_task(void *pvParameters)
{
    (void)pvParameters;

    DevTFT *tft = GetTFT();
    if (tft == NULL) {
        LOG_ERROR("[TFT] Device handle is NULL!\r\n");
        vTaskDelete(NULL);
        return;
    }

    tft->init(tft);
    LOG_INFO("[TFT] ST7735 initialized OK\r\n");

    static MenuCtx menuCtx;
    Menu_Init(&menuCtx, rootItems, (uint8_t)ROOT_ITEM_COUNT);
    g_scrollOfs = 0;

    bool inLeaf = false;
    bool insViewLeaf = false;
    bool yawStatusLeaf = false;
    bool arcStatusLeaf = false;
    menu_render_full(tft, &menuCtx);

    DevEncoder_Event_t evt;
    while (1)
    {
        if (xQueueReceive(g_menuEvtQueue, &evt, pdMS_TO_TICKS(MENU_REFRESH_MS)) != pdTRUE) {
            auto_test_tick(tft, &menuCtx, inLeaf);
            if (insViewLeaf) {
                ins_view_render(tft);
            } else if (yawStatusLeaf) {
                yaw_status_render(tft);
            } else if (arcStatusLeaf) {
                arc_status_render(tft);
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
                        if (cur == &rootItems[ROOT_INS_VIEW_INDEX]) {
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
