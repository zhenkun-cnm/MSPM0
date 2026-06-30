/**
 * @file    port_motor_encoder.c
 * @brief   Port 层电机编码器实现
 * @note    读取 TIMG8 QEI 硬件编码器计数器
 *          PB15 = PHA (TIMG8 CCP0), PB16 = PHB (TIMG8 CCP1)
 *          QEI 模式由 SYSCFG_DL_TB6612_ENA_init() 初始化
 */
#include "port_motor_encoder.h"
#include "ti_msp_dl_config.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

void PORT_MOTOR_ENCODER_Init(void)
{
    /*
     * SYSCFG_DL_TB6612_ENA_init() 已配置 TIMG8 的 QEI 模式
     * (clock source / QEI 2-input / loadValue=65535 / clock enabled),
     * 但未启动计数器。此处在首次读取前显式启动。
     *
     * DL_TimerG_startCounter() 通过写 CTL1.CNTCTL 来启动计数,
     * 不影响已配置的 QEI 模式。
     */
    DL_TimerG_startCounter(TB6612_ENA_INST);
}

uint16_t PORT_MOTOR_ENCODER_GetCount(void)
{
    /*
     * QEI 模式下, 计数器值反映在 TIMG8->COUNTERREGS.CTR 中。
     * 读取低 16 位 (QEI 配置为 16-bit 模式, loadValue=65535)。
     * 直接读硬件寄存器而非 DL_TimerG_getCaptureCompareValue(),
     * 因为后者读取的是 CCR(捕获/比较寄存器), 非计数器值。
     */
    return (uint16_t)(TIMG8->COUNTERREGS.CTR);
}
