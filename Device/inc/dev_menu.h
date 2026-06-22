/**
 * @file    dev_menu.h
 * @brief   多级菜单引擎 Device 层接口（纯逻辑，无硬件依赖）
 * @note    四层解耦架构 - Device 层
 *          以静态树状 MenuItem 表描述菜单层级，MenuCtx 维护导航/编辑状态。
 *          交互映射（由 App 层翻译编码器事件后调用）：
 *            旋转  → Menu_Rotate(±1)   （编辑模式下改值，否则移动高亮）
 *            短按  → Menu_Enter()       （进子菜单 / 触发叶子）
 *            长按  → Menu_Back()        （返回上一级）
 *            双击  → Menu_ToggleEdit()  （可调值项进/出编辑模式）
 */

#ifndef DEV_MENU_H
#define DEV_MENU_H

#include <stdint.h>
#include <stdbool.h>

/* 最大菜单层级深度（用于返回栈） */
#define MENU_MAX_DEPTH      4U

/* 菜单项类型 */
typedef enum {
    MENU_SUBMENU = 0,   /* 含子菜单：短按进入 children */
    MENU_LEAF,          /* 叶子动作：短按进入占位/执行页 */
    MENU_VALUE          /* 可调值：双击进入编辑，旋转增减 valuePtr */
} MenuType;

/* 可调值的数据类型（仅 MENU_VALUE 有效） */
typedef enum {
    MENU_VAL_INT = 0,   /* int16_t 整数：旋转步进 valStep.i，渲染 %d */
    MENU_VAL_FLOAT      /* float 浮点：旋转步进 valStep.f，渲染按步进定小数位 */
} MenuValType;

/*
 * 菜单项（静态 const 表项；MENU_VALUE 的 valuePtr 指向运行期变量）。
 *
 * MENU_VALUE 项的值/边界/步进通过 union 复用为 int16_t 或 float，由 valType 区分：
 *   - INT  : {.i=&var}, {.i=min}, {.i=max}, {.i=step}
 *   - FLOAT: {.f=&var}, {.f=min}, {.f=max}, {.f=step}
 * 非 MENU_VALUE 项（SUBMENU/LEAF）这些字段填零即可（valType 取默认 MENU_VAL_INT）。
 */
typedef struct MenuItem {
    const char            *title;       /* 显示文本 */
    MenuType               type;        /* 子菜单 / 叶子 / 可调值 */
    const struct MenuItem *children;    /* SUBMENU 时子数组首地址，否则 NULL */
    uint8_t                childCount;  /* 子项数量 */
    MenuValType            valType;     /* VALUE 项的数据类型：INT / FLOAT */
    union { int16_t *i; float *f; } valuePtr;   /* VALUE 时指向可变值，否则 NULL */
    union { int16_t i; float f; } valMin;       /* VALUE 下限 */
    union { int16_t i; float f; } valMax;       /* VALUE 上限 */
    union { int16_t i; float f; } valStep;      /* VALUE 旋转步进 */
} MenuItem;

/* 菜单运行时上下文 */
typedef struct {
    const MenuItem *current;    /* 当前层菜单数组 */
    uint8_t         count;      /* 当前层项数 */
    uint8_t         index;      /* 当前高亮项下标 */
    bool            editing;    /* true=编辑模式：旋转改值而非移动高亮 */
    uint8_t         depth;      /* 当前返回栈深度 */
    struct {
        const MenuItem *arr;
        uint8_t         cnt;
        uint8_t         idx;
    } stack[MENU_MAX_DEPTH];    /* 返回栈：记录各上级层 (数组,项数,高亮) */
} MenuCtx;

/**
 * @brief 初始化菜单上下文，定位到根菜单
 * @param ctx   菜单上下文
 * @param root  根菜单数组
 * @param count 根菜单项数
 */
void Menu_Init(MenuCtx *ctx, const MenuItem *root, uint8_t count);

/**
 * @brief 旋转处理：编辑模式下增减当前 VALUE 项并 clamp；否则移动高亮（回绕）
 * @param ctx  菜单上下文
 * @param dir  +1=顺时针(CW)，-1=逆时针(CCW)
 */
void Menu_Rotate(MenuCtx *ctx, int8_t dir);

/**
 * @brief 短按：SUBMENU 进入子菜单 / LEAF 触发；编辑模式下忽略
 * @return true=进入了叶子项（App 层据此渲染占位页）
 */
bool Menu_Enter(MenuCtx *ctx);

/**
 * @brief 长按：返回上一级；编辑模式或已在顶层时不动作
 * @return true=成功返回上级，false=已在顶层
 */
bool Menu_Back(MenuCtx *ctx);

/**
 * @brief 双击：仅当前项为 MENU_VALUE 时翻转编辑模式
 * @return true=切换了编辑状态，false=当前项不可编辑
 */
bool Menu_ToggleEdit(MenuCtx *ctx);

/**
 * @brief 取当前高亮项指针（便于 App 层渲染/取值）
 */
const MenuItem* Menu_Current(const MenuCtx *ctx);

#endif /* DEV_MENU_H */
