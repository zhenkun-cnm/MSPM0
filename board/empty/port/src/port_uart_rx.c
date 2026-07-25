/**
 * @file    port_uart_rx.c
 * @brief   UART0 interrupt-driven byte receive with ring buffer.
 */
#include "port_uart_rx.h"
#include "port_log.h"
#include "ti_msp_dl_config.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <stdbool.h>
#include <stdint.h>

/* ──────── 环形缓冲区 ──────── */
#define UART_RX_BUF_SIZE   256U

static volatile uint8_t  rx_buf[UART_RX_BUF_SIZE];
static volatile uint32_t rx_head = 0;   /* ISR 写入 */
static volatile uint32_t rx_tail = 0;   /* readByte 读出 */

static DevUartRx s_uartRx;

/* ──────── ISR: UART0 接收中断 ──────── */

void UART0_IRQHandler(void)
{
    uint8_t ch;
    uint32_t next;

    /* 逐个读取直到 RX FIFO/寄存器为空 */
    while (DL_UART_Main_receiveDataCheck(sys_uart_INST, &ch)) {
        next = (rx_head + 1U) & (UART_RX_BUF_SIZE - 1U);
        if (next != rx_tail) {
            rx_buf[rx_head] = ch;
            rx_head = next;
        }
        /* 缓冲区满则丢弃，避免覆盖旧数据 */
    }
    LOG_UART0_IRQHandler();
}

/* ──────── DevUartRx 接口 ──────── */

static void uart_rx_init(DevUartRx *self)
{
    uint8_t dummy;
    (void)self;

    /* 排空初始化前残留在硬件接收寄存器中的字节 */
    while (DL_UART_Main_receiveDataCheck(sys_uart_INST, &dummy)) {}

    /* 清空环形缓冲区 */
    rx_head = 0;
    rx_tail = 0;

    /* 使能 NVIC 中断 */
    NVIC_EnableIRQ(sys_uart_INST_INT_IRQN);
}

static bool uart_rx_read_byte(DevUartRx *self, uint8_t *ch)
{
    (void)self;

    if (ch == NULL) {
        return false;
    }

    if (rx_head == rx_tail) {
        return false;   /* 缓冲区空 */
    }

    *ch = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1U) & (UART_RX_BUF_SIZE - 1U);
    return true;
}

DevUartRx *GetUartRx(void)
{
    s_uartRx.init     = uart_rx_init;
    s_uartRx.readByte = uart_rx_read_byte;
    return &s_uartRx;
}
