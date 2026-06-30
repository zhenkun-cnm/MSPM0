/**
 * @file    port_motor_encoder.h
 * @brief   Port 层电机编码器接口
 * @note    读取 TIMG8 QEI 硬件编码器计数器
 *          PB15 = PHA (TIMG8 CCP0), PB16 = PHB (TIMG8 CCP1)
 */
#ifndef PORT_MOTOR_ENCODER_H
#define PORT_MOTOR_ENCODER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 TIMG8 QEI 编码器计数器（启动计数）
 * @note  SYSCFG_DL_TB6612_ENA_init() 已配置 QEI 模式但未启动计数器，
 *        需在首次读取前显式调用此函数启动。
 */
void PORT_MOTOR_ENCODER_Init(void);

/**
 * @brief 获取 TIMG8 QEI 计数器当前值
 * @return 16-bit 计数器值 (0~65535)
 */
uint16_t PORT_MOTOR_ENCODER_GetCount(void);

#ifdef __cplusplus
}
#endif

#endif /* PORT_MOTOR_ENCODER_H */