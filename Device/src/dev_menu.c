/**
 * @file    dev_menu.c
 * @brief   多级菜单引擎实现（纯逻辑，无硬件依赖）
 * @note    四层解耦架构 - Device 层
 *          实现 dev_menu.h 的菜单导航 / 编辑状态机。
 */

#include "dev_menu.h"
#include <stddef.h>     /* NULL */

/* ================================================================
 *  内部辅助
 * ================================================================ */

static int16_t clamp_i16(int16_t v, int16_t lo, int16_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float clamp_f(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ================================================================
 *  公有 API
 * ================================================================ */

void Menu_Init(MenuCtx *ctx, const MenuItem *root, uint8_t count)
{
    if (ctx == NULL) return;

    ctx->current = root;
    ctx->count   = count;
    ctx->index   = 0;
    ctx->editing = false;
    ctx->depth   = 0;
}

void Menu_Rotate(MenuCtx *ctx, int8_t dir)
{
    if (ctx == NULL || ctx->count == 0) return;

    /* dir 归一化为 ±1 */
    int8_t step = (dir >= 0) ? 1 : -1;

    if (ctx->editing) {
        /* 编辑模式：调当前 VALUE 项的值（按 INT / FLOAT 分流） */
        const MenuItem *it = &ctx->current[ctx->index];
        if (it->type == MENU_VALUE && it->valuePtr.i != NULL) {
            if (it->valType == MENU_VAL_FLOAT) {
                float nv = *it->valuePtr.f + (float)step * it->valStep.f;
                *it->valuePtr.f = clamp_f(nv, it->valMin.f, it->valMax.f);
            } else {
                int32_t nv = (int32_t)(*it->valuePtr.i) + (int32_t)step * it->valStep.i;
                *it->valuePtr.i = clamp_i16((int16_t)nv, it->valMin.i, it->valMax.i);
            }
        }
        return;
    }

    /* 非编辑：移动高亮（回绕） */
    if (step > 0) {
        ctx->index = (uint8_t)((ctx->index + 1U) % ctx->count);
    } else {
        ctx->index = (uint8_t)((ctx->index + ctx->count - 1U) % ctx->count);
    }
}

bool Menu_Enter(MenuCtx *ctx)
{
    if (ctx == NULL || ctx->count == 0 || ctx->editing) return false;

    const MenuItem *it = &ctx->current[ctx->index];

    if (it->type == MENU_SUBMENU &&
        it->children != NULL && it->childCount > 0) {

        if (ctx->depth >= MENU_MAX_DEPTH) {
            return false;   /* 超过最大深度，拒绝下钻 */
        }

        /* 压栈当前层 */
        ctx->stack[ctx->depth].arr = ctx->current;
        ctx->stack[ctx->depth].cnt = ctx->count;
        ctx->stack[ctx->depth].idx = ctx->index;
        ctx->depth++;

        /* 切到子菜单 */
        ctx->current = it->children;
        ctx->count   = it->childCount;
        ctx->index   = 0;
        return false;       /* 进入的是子菜单，非叶子 */
    }

    /* LEAF 或 VALUE 短按：视为"进入叶子页"，由 App 层处理 */
    return true;
}

bool Menu_Back(MenuCtx *ctx)
{
    if (ctx == NULL || ctx->editing) return false;

    if (ctx->depth == 0) {
        return false;       /* 已在顶层 */
    }

    ctx->depth--;
    ctx->current = ctx->stack[ctx->depth].arr;
    ctx->count   = ctx->stack[ctx->depth].cnt;
    ctx->index   = ctx->stack[ctx->depth].idx;
    return true;
}

bool Menu_ToggleEdit(MenuCtx *ctx)
{
    if (ctx == NULL || ctx->count == 0) return false;

    const MenuItem *it = &ctx->current[ctx->index];
    if (it->type != MENU_VALUE || it->valuePtr.i == NULL) {
        return false;       /* 当前项不可编辑 */
    }

    ctx->editing = !ctx->editing;
    return true;
}

const MenuItem* Menu_Current(const MenuCtx *ctx)
{
    if (ctx == NULL || ctx->count == 0) return NULL;
    return &ctx->current[ctx->index];
}
