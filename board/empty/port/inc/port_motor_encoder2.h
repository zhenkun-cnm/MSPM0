/**
 * @file    port_motor_encoder2.h
 * @brief   Port 层电机2编码器接口
 * @note    硬件架构: TIMG7 Edge-Time Capture (PA28 = A相) + DMA 环形搬运
 *          DMA 源地址 = &GPIOA->DIN31_0 (硬件总线快照, 含 PA29 = B相)
 *          每次 A 相上升沿触发 DMA, 将 32-bit GPIO 快照写入环形缓冲区
 *          10ms 软件轮询解码: 读取 A 相上升沿快照中的 B 相电平, 判向计数
 *
 *          编码器参数: DMA 1x 捕获, App 层乘 4 折算
 *          减速比 1:20, 实测约 985 脉冲/输出轴转
 */
#ifndef PORT_MOTOR_ENCODER2_H
#define PORT_MOTOR_ENCODER2_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 环形缓冲区大小 (条目数, 每条 32-bit)
 * @note  10ms 内最大原始捕获数 < 256 (软件 4x 折算后 < 1024 counts)
 */
#define ENC2_BUF_SIZE  256U

/**
 * @brief GPIO 快照中 A/B 相的位掩码
 */
#define ENC2_BIT_A     (1UL << 28)   /* PA28 = A 相 */
#define ENC2_BIT_B     (1UL << 29)   /* PA29 = B 相 */

/**
 * @brief 初始化电机2编码器硬件
 * @note  启动 TIMG7 计数器 + 配置 DMA_CH0 源/目标/传输大小 + 启动 DMA
 *        必须在 FreeRTOS 调度器启动前调用 (无阻塞 API)
 */
void PORT_MOTOR_ENCODER2_Init(void);

/**
 * @brief 从环形缓冲区中解析自上次轮询以来的脉冲增量
 * @param deltaPulses [out] 本次 10ms 内的脉冲增量 (正值=正转, 负值=反转)
 * @note  内部维护 DMA 写指针追踪与软件读指针
 *        线程安全: 仅由 motor_encoder2_task 单任务调用, 无需临界区
 */
void PORT_MOTOR_ENCODER2_Poll(int32_t *deltaPulses);

/**
 * @brief 获取 DMA 硬件写指针当前偏移
 * @return DMA 目的地址中已写入的条目偏移量 (0 ~ ENC2_BUF_SIZE-1)
 * @note  用于调试, 观察缓冲区水位
 */
uint32_t PORT_MOTOR_ENCODER2_GetDmaWriteOffset(void);

#ifdef __cplusplus
}
#endif

#endif /* PORT_MOTOR_ENCODER2_H */
