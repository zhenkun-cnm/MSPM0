/**
 * @file    dev_uart_rx.h
 * @brief   Device-layer UART receive interface for simple debug commands.
 */
#ifndef DEV_UART_RX_H
#define DEV_UART_RX_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct DevUartRx {
    void (*init)(struct DevUartRx *self);
    bool (*readByte)(struct DevUartRx *self, uint8_t *ch);
} DevUartRx;

DevUartRx *GetUartRx(void);

#ifdef __cplusplus
}
#endif

#endif /* DEV_UART_RX_H */
