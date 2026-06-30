/**
 * @file    dev_tb6612.h
 * @brief   TB6612 电机驱动 Device 层接口
 * @note    四层解耦架构 - Device 层 OOP 契约
 *          控制双通道 (A/B) 直流电机，含 PWM 调速 + 方向控制
 */
#ifndef DEV_TB6612_H
#define DEV_TB6612_H

#include <stdint.h>

/* 电机方向枚举 */
typedef enum {
    TB6612_DIR_FORWARD = 0,   /* 正转 (AIN1=H, AIN2=L) */
    TB6612_DIR_REVERSE,       /* 反转 (AIN1=L, AIN2=H) */
    TB6612_DIR_BRAKE,         /* 刹车 (AIN1=L, AIN2=L) */
    TB6612_DIR_COAST          /* 惰行/停止 (AIN1=H, AIN2=H 或 PWM 关闭) */
} TB6612_Dir;

/* TB6612 设备 OOP 接口 */
typedef struct DevTB6612 {
    /**
     * @brief 初始化 TB6612 硬件（GPIO 方向 + PWM 启动配置）
     * @note  在调度器启动后调用，依赖 HAL 已完成的 GPIO 初始化
     */
    void (*init)(struct DevTB6612 *self);

    /**
     * @brief 使能电机输出（启动 PWM 输出）
     */
    void (*enable)(struct DevTB6612 *self);

    /**
     * @brief 禁能电机输出（停止 PWM，GPIO 全低 → 刹车）
     */
    void (*disable)(struct DevTB6612 *self);

    /**
     * @brief 设置电机速度
     * @param pct  占空比百分比 0~100
     */
    void (*setSpeed)(struct DevTB6612 *self, uint8_t pct);

    /**
     * @brief 设置电机方向
     * @param dir  TB6612_DIR_FORWARD/REVERSE/BRAKE/COAST
     */
    void (*setDirection)(struct DevTB6612 *self, TB6612_Dir dir);

    /**
     * @brief 获取当前速度百分比
     */
    uint8_t (*getSpeed)(struct DevTB6612 *self);

    /**
     * @brief 获取当前方向
     */
    TB6612_Dir (*getDirection)(struct DevTB6612 *self);

} DevTB6612;

/* 工厂函数：获取 TB6612 单例 */
DevTB6612* GetTB6612(void);

#endif /* DEV_TB6612_H */