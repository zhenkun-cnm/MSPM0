/**
 * @file    dev_encoder.h
 * @brief   Encoder Device 层抽象接口
 * @note    四层解耦架构 - Device 层
 *          使用 struct + 函数指针模拟 OOP 契约
 *          支持旋转方向检测和按键（短按/长按/双击）
 */

#ifndef DEV_ENCODER_H
#define DEV_ENCODER_H

#include <stdint.h>
#include <stdbool.h>

/* 编码器事件类型 */
typedef enum {
    ENCODER_EVT_NONE        = 0,    /* 无事件 */
    ENCODER_EVT_CW          = 1,    /* 顺时针旋转 1 格 */
    ENCODER_EVT_CCW         = 2,    /* 逆时针旋转 1 格 */
    ENCODER_EVT_KEY_SHORT   = 3,    /* 短按 */
    ENCODER_EVT_KEY_LONG    = 4,    /* 长按 */
    ENCODER_EVT_KEY_DOUBLE  = 5     /* 双击 */
} DevEncoder_Event_t;

/* 编码器设备接口结构体（OOP 契约） */
typedef struct DevEncoder {
    /**
     * @brief 初始化编码器硬件
     * @param self  指向自身
     */
    void (*init)(struct DevEncoder *self);

    /**
     * @brief 获取下一个编码器事件（非阻塞）
     * @param self  指向自身
     * @return DevEncoder_Event_t  若队列为空返回 ENCODER_EVT_NONE
     */
    DevEncoder_Event_t (*getEvent)(struct DevEncoder *self);

    /**
     * @brief 获取当前累积位置计数值
     * @param self  指向自身
     * @return int32_t  顺时针为正，逆时针为负
     */
    int32_t (*getPosition)(struct DevEncoder *self);

    /**
     * @brief 重置位置计数器为 0
     * @param self  指向自身
     */
    void (*resetPosition)(struct DevEncoder *self);
} DevEncoder;

/**
 * @brief 获取全局编码器设备句柄
 * @return DevEncoder* 指向编码器设备接口的指针
 */
DevEncoder* GetEncoder(void);

#endif /* DEV_ENCODER_H */