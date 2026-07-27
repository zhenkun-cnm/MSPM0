/**
 * @file    port_car_uart.c
 * @brief   UART2 (PB17/PB18) interrupt-driven byte transport.
 */
#include "port_car_uart.h"
#include "ti_msp_dl_config.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

#define CAR_UART_RX_BUF_SIZE 128U

static volatile uint8_t s_rxBuf[CAR_UART_RX_BUF_SIZE];
static volatile uint32_t s_rxHead = 0U;
static volatile uint32_t s_rxTail = 0U;
static volatile uint32_t s_rxOverflowCount = 0U;
static DevCarUart s_carUart;

void UART2_IRQHandler(void)
{
    uint8_t ch;
    uint32_t next;

    while (DL_UART_Main_receiveDataCheck(car_uart_INST, &ch)) {
        next = (s_rxHead + 1U) & (CAR_UART_RX_BUF_SIZE - 1U);
        if (next == s_rxTail) {
            s_rxOverflowCount++;
        } else {
            s_rxBuf[s_rxHead] = ch;
            s_rxHead = next;
        }
    }
}

static void car_uart_init(DevCarUart *self)
{
    uint8_t dummy;

    (void)self;

    while (DL_UART_Main_receiveDataCheck(car_uart_INST, &dummy)) {
    }

    s_rxHead = 0U;
    s_rxTail = 0U;
    s_rxOverflowCount = 0U;
    NVIC_EnableIRQ(car_uart_INST_INT_IRQN);
}

static bool car_uart_read_byte(DevCarUart *self, uint8_t *ch)
{
    (void)self;

    if (ch == NULL || s_rxHead == s_rxTail) {
        return false;
    }

    *ch = s_rxBuf[s_rxTail];
    s_rxTail = (s_rxTail + 1U) & (CAR_UART_RX_BUF_SIZE - 1U);
    return true;
}

static bool car_uart_write(DevCarUart *self, const uint8_t *data, uint8_t len)
{
    uint8_t i;

    (void)self;

    if (data == NULL && len != 0U) {
        return false;
    }

    for (i = 0U; i < len; i++) {
        DL_UART_Main_transmitDataBlocking(car_uart_INST, data[i]);
    }

    return true;
}

static uint32_t car_uart_get_rx_overflow_count(DevCarUart *self)
{
    (void)self;
    return s_rxOverflowCount;
}

DevCarUart *GetCarUart(void)
{
    s_carUart.init = car_uart_init;
    s_carUart.readByte = car_uart_read_byte;
    s_carUart.write = car_uart_write;
    s_carUart.getRxOverflowCount = car_uart_get_rx_overflow_count;
    return &s_carUart;
}
