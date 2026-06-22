/**
 * @file    app_tft.c
 * @brief   App 层 TFT 多级菜单显示任务
 * @note    四层解耦架构 - App 层
 *          通过 Device 句柄操作 ST7735 彩屏，渲染由旋转编码器驱动的多级菜单。
 *          编码器事件经 g_menuEvtQueue 跨任务送达，映射：
 *            CW/CCW    → Menu_Rotate（编辑模式下改值）
 *            SHORT     → Menu_Enter（进子菜单 / 叶子页）
 *            LONG      → Menu_Back（返回上一级）
 *            DOUBLE    → Menu_ToggleEdit（可调值项进/出编辑）
 */

#include "app_tft.h"
#include "app_menu.h"
#include "dev_tft.h"
#include "dev_menu.h"
#include "dev_encoder.h"
#include "port_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* ================================================================
 *  菜单内容表（静态树状结构，加菜单只改此表）
 * ================================================================ */

/* 运行期可变值（非 const，供编辑模式修改） */
static int16_t g_brightness = 50;       /* 0..100 整数 */
static int16_t g_contrast   = 50;       /* 0..100 整数 */
static float   g_temp       = 23.5f;    /* 0.0..50.0 float，step 0.5 → 1 位小数 */

/* 字段顺序：title, type, children, childCount, valType, valuePtr, valMin, valMax, valStep
 * 非 VALUE 项（SUBMENU/LEAF）的值字段填 MENU_VAL_INT + 全零。 */
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

static const MenuItem rootItems[] = {
    {"Settings",   MENU_SUBMENU, settingsItems, 4, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"Sensors",    MENU_SUBMENU, sensorItems,   2, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
    {"About",      MENU_SUBMENU, aboutItems,    3, MENU_VAL_INT, {.i = NULL}, {.i = 0}, {.i = 0}, {.i = 0}},
};

#define ROOT_ITEM_COUNT     (sizeof(rootItems) / sizeof(rootItems[0]))

/* ================================================================
 *  渲染参数
 * ================================================================ */
#define MENU_ROW_H          16U     /* 行高（8x16 字体） */
#define MENU_MAX_ROWS       (TFT_HEIGHT / MENU_ROW_H)   /* 一屏最多行数 = 5 */

/* 按键测试状态行：屏幕最底行(y=64)显示最近一次按键事件 */
#define KEY_STATUS_Y        (TFT_HEIGHT - MENU_ROW_H)    /* = 64 */
static const char *g_lastKeyMsg = "Key:--";             /* 最近按键提示 */

/* ================================================================
 *  辅助：按 float 步进推导显示小数位（避免过多无意义小数）
 *    step >= 1   → 0 位
 *    step >= 0.1 → 1 位
 *    其余        → 2 位（封顶）
 * ================================================================ */
static uint8_t decimals_from_step(float step)
{
    if (step < 0.0f) step = -step;
    if (step >= 1.0f)  return 0;
    if (step >= 0.1f)  return 1;
    return 2;
}

/* ================================================================
 *  菜单渲染：逐项绘制，高亮/编辑用反色区分；底部显示按键测试状态行
 * ================================================================ */
static void menu_render(DevTFT *tft, const MenuCtx *ctx)
{
    tft->fillScreen(tft, TFT_BLACK);

    /* 菜单项最多占用底部状态行以上的区域 */
    uint8_t rows = ctx->count;
    uint8_t maxRows = MENU_MAX_ROWS - 1;   /* 留出最底行给按键状态 */
    if (rows > maxRows) rows = maxRows;

    for (uint8_t i = 0; i < rows; i++) {
        const MenuItem *it = &ctx->current[i];
        bool selected = (i == ctx->index);

        uint16_t fg = TFT_WHITE;
        uint16_t bg = TFT_BLACK;

        if (selected) {
            /* 编辑中的可调值项用醒目色，其余高亮用蓝底 */
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
            /* "标题   值"，左对齐标题 + 数值（按值类型选择格式） */
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

    /* 底部按键测试状态行（绿字）：验证 单击/双击/长按 识别 */
    tft->printString(tft, 0, KEY_STATUS_Y, g_lastKeyMsg, TFT_GREEN, TFT_BLACK);
}

/* ================================================================
 *  叶子占位页
 * ================================================================ */
static void leaf_render(DevTFT *tft, const MenuItem *it)
{
    tft->fillScreen(tft, TFT_BLACK);
    tft->printString(tft, 0, 0, it->title, TFT_YELLOW, TFT_BLACK);
    tft->printString(tft, 0, MENU_ROW_H, "(leaf page)", TFT_WHITE, TFT_BLACK);
    tft->printString(tft, 0, 2 * MENU_ROW_H, "Long=Back", TFT_GRAY, TFT_BLACK);
}

/* ================================================================
 *  TFT 菜单任务
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

    /* 初始化 TFT 硬件 */
    tft->init(tft);
    LOG_INFO("[TFT] ST7735 initialized OK\r\n");

    /* 初始化菜单到根层并首绘 */
    static MenuCtx menuCtx;
    Menu_Init(&menuCtx, rootItems, (uint8_t)ROOT_ITEM_COUNT);

    bool inLeaf = false;            /* 是否停留在叶子占位页 */
    menu_render(tft, &menuCtx);

    /* 主循环：阻塞等待编码器事件 */
    DevEncoder_Event_t evt;
    while (1)
    {
        if (xQueueReceive(g_menuEvtQueue, &evt, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (inLeaf) {
            /* 叶子页内：仅长按返回当前菜单层（叶子未压栈，直接重绘即可） */
            if (evt == ENCODER_EVT_KEY_LONG) {
                g_lastKeyMsg = "Key:LONG";      /* 长按测试提示 */
                inLeaf = false;
                menu_render(tft, &menuCtx);
            }
            continue;
        }

        switch (evt) {
            case ENCODER_EVT_CW:
                Menu_Rotate(&menuCtx, +1);
                menu_render(tft, &menuCtx);
                break;

            case ENCODER_EVT_CCW:
                Menu_Rotate(&menuCtx, -1);
                menu_render(tft, &menuCtx);
                break;

            case ENCODER_EVT_KEY_SHORT:
                g_lastKeyMsg = "Key:SHORT";     /* 单击测试提示 */
                if (Menu_Enter(&menuCtx)) {
                    /* 进入叶子项 → 占位页 */
                    inLeaf = true;
                    leaf_render(tft, Menu_Current(&menuCtx));
                } else {
                    menu_render(tft, &menuCtx);
                }
                break;

            case ENCODER_EVT_KEY_LONG:
                g_lastKeyMsg = "Key:LONG";      /* 长按测试提示 */
                Menu_Back(&menuCtx);
                menu_render(tft, &menuCtx);
                break;

            case ENCODER_EVT_KEY_DOUBLE:
                g_lastKeyMsg = "Key:DOUBLE";    /* 双击测试提示 */
                Menu_ToggleEdit(&menuCtx);
                menu_render(tft, &menuCtx);
                break;

            default:
                break;
        }
    }
}
