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

static const MenuItem testItems[] = {
    {"LED Test",    MENU_LEAF,    NULL,        0, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"Encoder",     MENU_LEAF,    NULL,        0, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"Button",      MENU_LEAF,    NULL,        0, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"Motor",       MENU_SUBMENU, motorItems,  MOTOR_ITEM_COUNT, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
};

static const MenuItem rootItems[] = {
    {"Test",       MENU_SUBMENU, testItems,    4, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"Settings",   MENU_SUBMENU, settingsItems, 4, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"Sensors",    MENU_SUBMENU, sensorItems,   2, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"About",      MENU_SUBMENU, aboutItems,    3, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
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

/* Scroll offset (module-level, reset on menu layer change via Menu_Enter / Menu_Back) */
static uint8_t g_scrollOfs = 0;
static const char *g_lastKeyMsg = "Key:--";

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
static void scroll_update(const MenuCtx *ctx)
{
    uint8_t maxVis = MENU_VISIBLE_ROWS;
    if (ctx->count <= maxVis) {
        g_scrollOfs = 0;
        return;
    }
    if (ctx->index >= (uint8_t)(g_scrollOfs + maxVis)) {
        g_scrollOfs = (uint8_t)(ctx->index - maxVis + 1U);
    }
    if (ctx->index < g_scrollOfs) {
        g_scrollOfs = ctx->index;
    }
}

/* ================================================================
 *  Menu render (with scroll support)
 * ================================================================ */
static void menu_render(DevTFT *tft, const MenuCtx *ctx)
{
    tft->fillScreen(tft, TFT_BLACK);

    scroll_update(ctx);

    uint8_t maxVis = MENU_VISIBLE_ROWS;
    if (ctx->count < maxVis) maxVis = ctx->count;

    for (uint8_t i = 0; i < maxVis; i++) {
        uint8_t idx = (uint8_t)(g_scrollOfs + i);
        if (idx >= ctx->count) break;
        const MenuItem *it = &ctx->current[idx];
        bool selected = (idx == ctx->index);

        uint16_t fg = TFT_WHITE;
        uint16_t bg = TFT_BLACK;

        if (selected) {
            if (ctx->editing && it->type == MENU_VALUE) {
                fg = TFT_WHITE;
                bg = TFT_ORANGE;
            } else {
                fg = TFT_WHITE;
                bg = TFT_BLUE;
            }
        }

        uint16_t y = (uint16_t)(i * MENU_ROW_H);

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

    tft->printString(tft, 0, KEY_STATUS_Y, g_lastKeyMsg, TFT_GREEN, TFT_BLACK);
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
    menu_render(tft, &menuCtx);

    DevEncoder_Event_t evt;
    while (1)
    {
        if (xQueueReceive(g_menuEvtQueue, &evt, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (inLeaf) {
            if (evt == ENCODER_EVT_KEY_LONG) {
                g_lastKeyMsg = "Key:LONG";
                inLeaf = false;
                menu_render(tft, &menuCtx);
            }
            continue;
        }

        switch (evt) {
            case ENCODER_EVT_CW:
                Menu_Rotate(&menuCtx, +1);
                if (is_in_motor_menu(&menuCtx)) {
                    motor_dispatch(&menuCtx, false, 0, NULL);
                }
                menu_render(tft, &menuCtx);
                break;

            case ENCODER_EVT_CCW:
                Menu_Rotate(&menuCtx, -1);
                if (is_in_motor_menu(&menuCtx)) {
                    motor_dispatch(&menuCtx, false, 0, NULL);
                }
                menu_render(tft, &menuCtx);
                break;

            case ENCODER_EVT_KEY_SHORT:
                g_lastKeyMsg = "Key:SHORT";
                if (Menu_Enter(&menuCtx)) {
                    const MenuItem *cur = Menu_Current(&menuCtx);
                    if (handle_quick_leaf(cur, &menuCtx)) {
                        menu_render(tft, &menuCtx);
                    } else {
                        inLeaf = true;
                        leaf_render(tft, cur);
                    }
                } else {
                    g_scrollOfs = 0;
                    menu_render(tft, &menuCtx);
                }
                break;

            case ENCODER_EVT_KEY_LONG:
                g_lastKeyMsg = "Key:LONG";
                Menu_Back(&menuCtx);
                g_scrollOfs = 0;
                menu_render(tft, &menuCtx);
                break;

            case ENCODER_EVT_KEY_DOUBLE:
                g_lastKeyMsg = "Key:DOUBLE";
                Menu_ToggleEdit(&menuCtx);
                menu_render(tft, &menuCtx);
                break;

            default:
                break;
        }
    }
}