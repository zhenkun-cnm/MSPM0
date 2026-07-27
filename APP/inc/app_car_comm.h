/**
 * @file    app_car_comm.h
 * @brief   Application protocol for UART2 vehicle-to-vehicle communication.
 */
#ifndef APP_CAR_COMM_H
#define APP_CAR_COMM_H

#include "FreeRTOS.h"
#include <stdbool.h>
#include <stdint.h>

#ifndef CAR_COMM_NODE_ID
#define CAR_COMM_NODE_ID 1U
#endif

#define CAR_COMM_MAX_DATA_LEN 20U
#define CAR_COMM_ACK_CMD      0xF0U
#define CAR_COMM_PING_CMD     0x01U

typedef struct {
    uint8_t seq;
    uint8_t srcId;
    uint8_t dstId;
    uint8_t cmd;
    uint8_t len;
    uint8_t data[CAR_COMM_MAX_DATA_LEN];
} CarComm_Message_t;

typedef struct {
    uint32_t txFrames;
    uint32_t txAcked;
    uint32_t txRetries;
    uint32_t txTimeouts;
    uint32_t rxFrames;
    uint32_t rxDuplicates;
    uint32_t rxQueueFull;
    uint32_t rxBadChecksum;
    uint32_t rxBadLength;
    uint32_t rxBadTail;
    uint32_t rxWrongDestination;
    uint32_t rxFrameTimeouts;
    uint32_t uartRxOverflow;
} CarComm_Stats_t;

bool CarComm_Init(void);
bool CarComm_Send(uint8_t dstId,
                  uint8_t cmd,
                  const uint8_t *data,
                  uint8_t len,
                  bool reliable);
bool CarComm_Receive(CarComm_Message_t *message, TickType_t waitTicks);
void CarComm_GetStats(CarComm_Stats_t *stats);
void car_comm_task(void *pvParameters);

#endif /* APP_CAR_COMM_H */
