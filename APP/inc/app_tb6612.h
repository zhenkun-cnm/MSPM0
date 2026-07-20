/**
 * @file    app_tb6612.h
 * @brief   App 层 TB6612 电机任务头文件 + 命令队列定义
 * @note    四层解耦架构 - App 层
 *          tft_task (菜单) 通过 g_motorCmdQueue 向 tb6612_task 发送命令
 */

#ifndef APP_TB6612_H
#define APP_TB6612_H

#include "FreeRTOS.h"
#include "queue.h"
#include "dev_tb6612.h"

/* Motor command types */
typedef enum {
    MOTOR_CMD_ONOFF = 0,    /* On/Off: val: 0=off 1=on */
    MOTOR_CMD_SPEED,        /* Speed: val: 0~100 */
    MOTOR_CMD_DIR,          /* Direction: val: TB6612_Dir */
    MOTOR_CMD_LEFT_ONLY,    /* Left wheel only (A fwd, B coast) */
    MOTOR_CMD_RIGHT_ONLY,   /* Right wheel only (B fwd, A coast) */
    MOTOR_CMD_LEFT_SPEED,   /* Left channel speed only (0-100), val=0..100 */
    MOTOR_CMD_RIGHT_SPEED,  /* Right channel speed only (0-100), val=0..100 */
    MOTOR_CMD_LEFT_SIGNED_SPEED,  /* Left signed speed, val=-100..100 */
    MOTOR_CMD_RIGHT_SIGNED_SPEED  /* Right signed speed, val=-100..100 */
} MotorCmdType;

/* 电机命令结构体 */
typedef struct {
    MotorCmdType type;      /* 命令类型 */
    int16_t      val;       /* 命令参数 (uint8_t 或 TB6612_Dir，强转到 int16_t) */
} MotorCmd;

/* 菜单→电机命令队列（在 app_init.c 中创建） */
extern QueueHandle_t g_motorCmdQueue;

/* 队列深度 */
#define MOTOR_CMD_QUEUE_LEN     8U

/* tb6612_task 任务函数声明 */
void tb6612_task(void *pvParameters);

#endif /* APP_TB6612_H */
