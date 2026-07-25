/**
 * @file    port_hardfault.c
 * @brief   HardFault 栈回溯器 — 裸机 UART 直接输出
 * @note    覆盖 startup 中弱符号 HardFault_Handler
 *          不依赖 FreeRTOS，直接操作 UART0 寄存器和 SCB
 */

#include <stdint.h>
#include <ti/devices/msp/msp.h>
#include "ti_msp_dl_config.h"     /* sys_uart_INST = UART0 */
#include <ti/driverlib/driverlib.h>
#include "FreeRTOS.h"
#include "task.h"

/* ================================================================
 *  裸机 UART 发送 — 用 DriverLib 阻塞发送，复用 SysConfig UART0
 *  Cortex-M0+ 无 CFSR/HFSR，只输出栈帧寄存器
 * ================================================================ */

static void uart_putc(char c)
{
    DL_UART_transmitDataBlocking(sys_uart_INST, (uint8_t)c);
}

static void uart_puts(const char *s)
{
    while (*s) { uart_putc(*s++); }
}

/* 输出 32-bit hex */
static void uart_hex32(uint32_t val)
{
    uart_puts("0x");
    for (int i = 28; i >= 0; i -= 4) {
        uint8_t nibble = (val >> i) & 0xF;
        uart_putc(nibble < 10 ? '0' + nibble : 'A' + nibble - 10);
    }
}

/* ================================================================
 *  HardFault Handler（覆盖弱符号）
 *
 *  Cortex-M0+ HardFault 栈帧（硬件自动入栈）:
 *    SP[0]  = R0
 *    SP[1]  = R1
 *    SP[2]  = R2
 *    SP[3]  = R3
 *    SP[4]  = R12
 *    SP[5]  = LR (EXC_RETURN)
 *    SP[6]  = PC (故障地址)
 *    SP[7]  = xPSR
 * ================================================================ */

__attribute__((naked))
void HardFault_Handler(void)
{
    __asm volatile (
        "MOVS   R0, #4          \n"   /* 测试 LR bit2 判断使用 MSP 还是 PSP */
        "MOV    R1, LR          \n"
        "TST    R0, R1          \n"
        "BEQ    use_msp         \n"
        "MRS    R0, PSP         \n"
        "B       dump           \n"
        "use_msp:               \n"
        "MRS    R0, MSP         \n"
        "dump:                  \n"
        "BL      HardFault_C    \n"
        "B       .              \n"
    );
}

void HardFault_C(uint32_t *sp)
{
    uint32_t r0  = sp[0];
    uint32_t r1  = sp[1];
    uint32_t r2  = sp[2];
    uint32_t r3  = sp[3];
    uint32_t r12 = sp[4];
    uint32_t lr  = sp[5];
    uint32_t pc  = sp[6];
    uint32_t psr = sp[7];

    uart_puts("\r\n\r\n");
    uart_puts("================================\r\n");
    uart_puts("===  HARDFAULT  ===\r\n");
    uart_puts("================================\r\n");

    uart_puts("SP="); uart_hex32((uint32_t)sp);  uart_puts("\r\n");
    uart_puts("R0="); uart_hex32(r0);  uart_puts("\r\n");
    uart_puts("R1="); uart_hex32(r1);  uart_puts("\r\n");
    uart_puts("R2="); uart_hex32(r2);  uart_puts("\r\n");
    uart_puts("R3="); uart_hex32(r3);  uart_puts("\r\n");
    uart_puts("R12="); uart_hex32(r12); uart_puts("\r\n");
    uart_puts("LR="); uart_hex32(lr);  uart_puts("\r\n");
    uart_puts("PC="); uart_hex32(pc);  uart_puts("\r\n");
    uart_puts("PSR="); uart_hex32(psr); uart_puts("\r\n");

    /* Cortex-M0+ 无 CFSR/HFSR，PC 指向故障指令 */
    uart_puts("---\r\n");
    uart_puts("(M0+: no CFSR/HFSR)\r\n");

    uart_puts("================================\r\n");
    uart_puts("System halted.\r\n");

    /* 重启 */
    uart_puts("Rebooting in 3s...\r\n");
    for (volatile uint32_t d = 0; d < 8000000; d++) { __NOP(); }
    NVIC_SystemReset();

    while (1) { __NOP(); }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    __disable_irq();
    uart_puts("\r\n=== STACK OVERFLOW ===\r\n");
    uart_puts("task=");
    uart_puts((pcTaskName != NULL) ? pcTaskName : "unknown");
    uart_puts("\r\nSystem halted. Rebooting in 3s...\r\n");
    for (volatile uint32_t d = 0; d < 8000000U; d++) { __NOP(); }
    NVIC_SystemReset();
    while (1) { __NOP(); }
}