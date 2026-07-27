/**
 * @file    dev_car_uart.h
 * @brief   Device-layer UART2 byte transport for inter-vehicle communication.
 */
#ifndef DEV_CAR_UART_H
#define DEV_CAR_UART_H

#include <stdbool.h>
#include <stdint.h>

typedef struct DevCarUart {
    void (*init)(struct DevCarUart *self);
    bool (*readByte)(struct DevCarUart *self, uint8_t *ch);
    bool (*write)(struct DevCarUart *self, const uint8_t *data, uint8_t len);
    uint32_t (*getRxOverflowCount)(struct DevCarUart *self);
} DevCarUart;

DevCarUart *GetCarUart(void);

#endif /* DEV_CAR_UART_H */
