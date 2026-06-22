/**
 * @file    port_system.h
 * @brief   Port 层系统硬件初始化入口
 * @note    main() 中调度器启动前调用，完成所有 BSP 硬件初始化
 */

#ifndef PORT_SYSTEM_H
#define PORT_SYSTEM_H

/**
 * @brief 系统硬件初始化（调度器启动前调用）
 * @note  封装 SYSCFG_DL_init()，将所有纯寄存器初始化集中在此处
 */
void PORT_SYSTEM_Init(void);

#endif /* PORT_SYSTEM_H */