/**
 * @file    dev_button.h
 * @brief   独立按键 Device 层抽象接口
 * @note    四层解耦架构 - Device 层
 *          使用 struct + 函数指针模拟 OOP 契约
 *          支持两路独立按键（PA27 / PB27）的短按 / 长按 / 双击
 */

#ifndef DEV_BUTTON_H
#define DEV_BUTTON_H

#include <stdint.h>
#include <stdbool.h>

/* 按键事件类型 */
typedef enum {
    BUTTON_EVT_NONE   = 0,    /* 无事件 */
    BUTTON_EVT_SHORT  = 1,    /* 短按 */
    BUTTON_EVT_LONG   = 2,    /* 长按 */
    BUTTON_EVT_DOUBLE = 3     /* 双击 */
} DevButton_EvtType_t;

/* 按键 ID */
typedef enum {
    BUTTON_ID_1 = 1,          /* PA27 */
    BUTTON_ID_2 = 2           /* PB27 */
} DevButton_Id_t;

/* 带 ID 的按键事件结构体 */
typedef struct {
    uint8_t              id;   /* DevButton_Id_t；NONE 时为 0 */
    DevButton_EvtType_t  type;
} DevButton_Event_t;

/* 按键设备接口结构体（OOP 契约） */
typedef struct DevButton {
    /**
     * @brief 初始化按键设备
     * @param self  指向自身
     */
    void (*init)(struct DevButton *self);

    /**
     * @brief 获取下一个按键事件（非阻塞）
     * @param self  指向自身
     * @return DevButton_Event_t  队列为空时返回 {0, BUTTON_EVT_NONE}
     */
    DevButton_Event_t (*getEvent)(struct DevButton *self);
} DevButton;

/**
 * @brief 获取全局按键设备句柄
 * @return DevButton* 指向按键设备接口的指针
 */
DevButton* GetButton(void);

#endif /* DEV_BUTTON_H */
