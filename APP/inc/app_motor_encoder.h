/**
 * @file    app_motor_encoder.h
 * @brief   App 层电机编码器任务头文件
 * @note    每 10ms 采集两路电机编码器脉冲增量
 *          全局变量 g_motorPulseCount / g_motor2PulseCount 供其他模块读取
 */
#ifndef APP_MOTOR_ENCODER_H
#define APP_MOTOR_ENCODER_H

#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR1_COUNTS_PER_OUTPUT_REV_NOMINAL  (1040L)
#define MOTOR1_COUNTS_PER_OUTPUT_REV_CAL      (1054L)
#define MOTOR2_COUNTS_PER_OUTPUT_REV_CAL      (985L)
#define MOTOR2_ENCODER_SOFTWARE_SCALE         (4L)
#define MOTOR_ENC_PERIOD_MS                   (10U)

typedef struct {
    int32_t left_delta_counts;
    int32_t right_delta_counts;
    int64_t left_total_counts;
    int64_t right_total_counts;
    TickType_t tick;
    uint32_t seq;
} MotorEncoderSnapshot_t;

bool MotorEncoder_ReadSnapshot(MotorEncoderSnapshot_t *out);

/**
 * @brief 电机编码器每 10ms 脉冲增量 (有符号)
 * @note  正值=正转增量, 负值=反转增量
 *        13 线编码器 × 4 倍频 = 52 脉冲/转
 *        减速比 1:20 → 理论 1040 脉冲/输出轴转, 实测约 1054
 */
extern int32_t g_motorPulseCount;

/**
 * @brief 电机2编码器每 10ms 脉冲增量 (有符号)
 * @note  正值=正转增量, 负值=反转增量
 *        TIMG7/DMA 仅捕获 A 相上升沿(1x), App 层乘 4 折算到 4x
 *        减速比 1:20, 实测标定约 985 脉冲/输出轴转
 */
extern int32_t g_motor2PulseCount;

/**
 * @brief 电机编码器采集任务 (双路合并)
 * @note  每 10ms 运行一次, 同时采集 Motor1 (TIMG8 QEI) 和 Motor2 (DMA-GPIO)
 *        分别写入 g_motorPulseCount / g_motor2PulseCount
 */
void motor_encoder_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* APP_MOTOR_ENCODER_H */
