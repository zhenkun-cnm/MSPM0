/**
 * @file    app_motor_encoder.h
 * @brief   App 层电机编码器任务头文件
 * @note    每 10ms 采集 TIMG8 QEI 编码器脉冲增量
 *          全局变量 g_motorPulseCount 供其他模块读取
 */
#ifndef APP_MOTOR_ENCODER_H
#define APP_MOTOR_ENCODER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 电机编码器每 10ms 脉冲增量 (有符号)
 * @note  正值=正转增量, 负值=反转增量
 *        13 线编码器 × 4 倍频 = 52 脉冲/转
 *        减速比 1:20 → 1040 脉冲/输出轴转
 */
extern int32_t g_motorPulseCount;

/**
 * @brief 电机2编码器每 10ms 脉冲增量 (有符号)
 * @note  正值=正转增量, 负值=反转增量
 *        13 线编码器 × 4 倍频 = 52 脉冲/电机转
 *        减速比 1:30 → 1560 脉冲/输出轴转
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