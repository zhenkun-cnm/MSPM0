/**
 * @file    dev_led.h
 * @brief   LED Device 层抽象接口
 * @note    四层解耦架构 - Device 层
 *          使用 struct + 函数指针模拟 OOP 契约
 */

#ifndef DEV_LED_H
#define DEV_LED_H

#include <stdint.h>
#include <stdbool.h>

/* LED 状态枚举 */
typedef enum {
    LED_STATE_OFF = 0,
    LED_STATE_ON
} DevLED_State_t;

/* LED 设备接口结构体（OOP 契约） */
typedef struct DevLED {
    void (*init)(struct DevLED *self);
    void (*on)(struct DevLED *self);
    void (*off)(struct DevLED *self);
    void (*toggle)(struct DevLED *self);
    DevLED_State_t (*getState)(struct DevLED *self);
} DevLED;

/**
 * @brief 获取全局 LED 设备句柄
 * @return DevLED* 指向 LED 设备接口的指针
 */
DevLED* GetLED(void);

#endif /* DEV_LED_H */