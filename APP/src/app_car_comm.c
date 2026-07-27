/**
 * @file    app_car_comm.c
 * @brief   Framed UART2 protocol for low-rate inter-vehicle communication.
 */
#include "app_car_comm.h"
#include "dev_car_uart.h"
#include "port_log.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include <string.h>

#define CAR_COMM_SOF1                 0xAAU
#define CAR_COMM_SOF2                 0x55U
#define CAR_COMM_TAIL1                0x0DU
#define CAR_COMM_TAIL2                0x0AU
#define CAR_COMM_BROADCAST_ID         0xFFU
#define CAR_COMM_TX_QUEUE_LEN         8U
#define CAR_COMM_RX_QUEUE_LEN         8U
#define CAR_COMM_TASK_PERIOD_MS       5U
#define CAR_COMM_ACK_TIMEOUT_MS       50U
#define CAR_COMM_MAX_RETRIES          3U
#define CAR_COMM_FRAME_TIMEOUT_MS     20U
#define CAR_COMM_MAX_FRAME_LEN        (2U + 5U + CAR_COMM_MAX_DATA_LEN + 1U + 2U)

typedef enum {
    CAR_COMM_PARSE_SOF1 = 0,
    CAR_COMM_PARSE_SOF2,
    CAR_COMM_PARSE_SEQ,
    CAR_COMM_PARSE_SRC,
    CAR_COMM_PARSE_DST,
    CAR_COMM_PARSE_CMD,
    CAR_COMM_PARSE_LEN,
    CAR_COMM_PARSE_DATA,
    CAR_COMM_PARSE_XOR,
    CAR_COMM_PARSE_TAIL1,
    CAR_COMM_PARSE_TAIL2
} CarComm_ParseState_t;

typedef struct {
    CarComm_Message_t message;
    bool reliable;
} CarComm_TxRequest_t;

typedef struct {
    bool active;
    CarComm_TxRequest_t request;
    uint8_t seq;
    uint8_t retries;
    TickType_t sentTick;
} CarComm_PendingTx_t;

static QueueHandle_t s_txQueue = NULL;
static QueueHandle_t s_rxQueue = NULL;
static DevCarUart *s_uart = NULL;
static CarComm_Stats_t s_stats = {0};
static CarComm_ParseState_t s_parseState = CAR_COMM_PARSE_SOF1;
static CarComm_Message_t s_parseMessage;
static uint8_t s_parseXor = 0U;
static uint8_t s_dataIndex = 0U;
static TickType_t s_lastByteTick = 0;
static uint8_t s_nextSeq = 0U;
static bool s_lastRxValid = false;
static uint8_t s_lastRxSrc = 0U;
static uint8_t s_lastRxSeq = 0U;
static CarComm_PendingTx_t s_pendingTx = {0};

static void parser_reset(void)
{
    s_parseState = CAR_COMM_PARSE_SOF1;
    s_parseXor = 0U;
    s_dataIndex = 0U;
}

static uint8_t calc_xor(const CarComm_Message_t *message)
{
    uint8_t value;
    uint8_t i;

    value = message->seq ^ message->srcId ^ message->dstId ^ message->cmd ^ message->len;
    for (i = 0U; i < message->len; i++) {
        value ^= message->data[i];
    }
    return value;
}

static bool send_frame(uint8_t seq, const CarComm_Message_t *message)
{
    uint8_t frame[CAR_COMM_MAX_FRAME_LEN];
    uint8_t index = 0U;
    uint8_t i;

    if (s_uart == NULL || message == NULL || message->len > CAR_COMM_MAX_DATA_LEN) {
        return false;
    }

    frame[index++] = CAR_COMM_SOF1;
    frame[index++] = CAR_COMM_SOF2;
    frame[index++] = seq;
    frame[index++] = CAR_COMM_NODE_ID;
    frame[index++] = message->dstId;
    frame[index++] = message->cmd;
    frame[index++] = message->len;
    for (i = 0U; i < message->len; i++) {
        frame[index++] = message->data[i];
    }
    frame[index++] = seq ^ CAR_COMM_NODE_ID ^ message->dstId ^ message->cmd ^ message->len;
    for (i = 0U; i < message->len; i++) {
        frame[index - 1U] ^= message->data[i];
    }
    frame[index++] = CAR_COMM_TAIL1;
    frame[index++] = CAR_COMM_TAIL2;

    if (!s_uart->write(s_uart, frame, index)) {
        return false;
    }

    s_stats.txFrames++;
    return true;
}

static void send_ack(const CarComm_Message_t *received)
{
    CarComm_Message_t ack;

    memset(&ack, 0, sizeof(ack));
    ack.dstId = received->srcId;
    ack.cmd = CAR_COMM_ACK_CMD;
    ack.len = 1U;
    ack.data[0] = received->seq;
    (void)send_frame(s_nextSeq++, &ack);
}

static void process_valid_message(void)
{
    bool duplicate;

    if (s_parseMessage.dstId != CAR_COMM_NODE_ID &&
        s_parseMessage.dstId != CAR_COMM_BROADCAST_ID) {
        s_stats.rxWrongDestination++;
        return;
    }

    if (s_parseMessage.cmd == CAR_COMM_ACK_CMD) {
        if (s_parseMessage.len == 1U && s_pendingTx.active &&
            s_parseMessage.srcId == s_pendingTx.request.message.dstId &&
            s_parseMessage.data[0] == s_pendingTx.seq) {
            s_pendingTx.active = false;
            s_stats.txAcked++;
        }
        return;
    }

    duplicate = s_lastRxValid &&
                s_lastRxSrc == s_parseMessage.srcId &&
                s_lastRxSeq == s_parseMessage.seq;
    if (duplicate) {
        s_stats.rxDuplicates++;
        if (s_parseMessage.dstId != CAR_COMM_BROADCAST_ID) {
            send_ack(&s_parseMessage);
        }
        return;
    }

    if (xQueueSend(s_rxQueue, &s_parseMessage, 0U) != pdTRUE) {
        s_stats.rxQueueFull++;
        return;
    }

    s_lastRxValid = true;
    s_lastRxSrc = s_parseMessage.srcId;
    s_lastRxSeq = s_parseMessage.seq;
    s_stats.rxFrames++;

    if (s_parseMessage.dstId != CAR_COMM_BROADCAST_ID) {
        send_ack(&s_parseMessage);
    }
}

static void parser_consume(uint8_t ch)
{
    s_lastByteTick = xTaskGetTickCount();

    switch (s_parseState) {
    case CAR_COMM_PARSE_SOF1:
        if (ch == CAR_COMM_SOF1) {
            s_parseState = CAR_COMM_PARSE_SOF2;
        }
        break;

    case CAR_COMM_PARSE_SOF2:
        if (ch == CAR_COMM_SOF2) {
            memset(&s_parseMessage, 0, sizeof(s_parseMessage));
            s_parseState = CAR_COMM_PARSE_SEQ;
        } else if (ch != CAR_COMM_SOF1) {
            s_parseState = CAR_COMM_PARSE_SOF1;
        }
        break;

    case CAR_COMM_PARSE_SEQ:
        s_parseMessage.seq = ch;
        s_parseXor = ch;
        s_parseState = CAR_COMM_PARSE_SRC;
        break;

    case CAR_COMM_PARSE_SRC:
        s_parseMessage.srcId = ch;
        s_parseXor ^= ch;
        s_parseState = CAR_COMM_PARSE_DST;
        break;

    case CAR_COMM_PARSE_DST:
        s_parseMessage.dstId = ch;
        s_parseXor ^= ch;
        s_parseState = CAR_COMM_PARSE_CMD;
        break;

    case CAR_COMM_PARSE_CMD:
        s_parseMessage.cmd = ch;
        s_parseXor ^= ch;
        s_parseState = CAR_COMM_PARSE_LEN;
        break;

    case CAR_COMM_PARSE_LEN:
        s_parseMessage.len = ch;
        s_parseXor ^= ch;
        if (ch > CAR_COMM_MAX_DATA_LEN) {
            s_stats.rxBadLength++;
            parser_reset();
        } else if (ch == 0U) {
            s_parseState = CAR_COMM_PARSE_XOR;
        } else {
            s_dataIndex = 0U;
            s_parseState = CAR_COMM_PARSE_DATA;
        }
        break;

    case CAR_COMM_PARSE_DATA:
        s_parseMessage.data[s_dataIndex++] = ch;
        s_parseXor ^= ch;
        if (s_dataIndex >= s_parseMessage.len) {
            s_parseState = CAR_COMM_PARSE_XOR;
        }
        break;

    case CAR_COMM_PARSE_XOR:
        if (ch != s_parseXor || ch != calc_xor(&s_parseMessage)) {
            s_stats.rxBadChecksum++;
            parser_reset();
        } else {
            s_parseState = CAR_COMM_PARSE_TAIL1;
        }
        break;

    case CAR_COMM_PARSE_TAIL1:
        if (ch == CAR_COMM_TAIL1) {
            s_parseState = CAR_COMM_PARSE_TAIL2;
        } else {
            s_stats.rxBadTail++;
            parser_reset();
        }
        break;

    case CAR_COMM_PARSE_TAIL2:
        if (ch == CAR_COMM_TAIL2) {
            process_valid_message();
        } else {
            s_stats.rxBadTail++;
        }
        parser_reset();
        break;

    default:
        parser_reset();
        break;
    }
}

static void service_rx(void)
{
    uint8_t ch;

    while (s_uart->readByte(s_uart, &ch)) {
        parser_consume(ch);
    }

    if (s_parseState != CAR_COMM_PARSE_SOF1 &&
        (xTaskGetTickCount() - s_lastByteTick) >= pdMS_TO_TICKS(CAR_COMM_FRAME_TIMEOUT_MS)) {
        s_stats.rxFrameTimeouts++;
        parser_reset();
    }
}

static void service_tx(void)
{
    CarComm_TxRequest_t request;
    TickType_t now = xTaskGetTickCount();

    if (s_pendingTx.active) {
        if ((now - s_pendingTx.sentTick) < pdMS_TO_TICKS(CAR_COMM_ACK_TIMEOUT_MS)) {
            return;
        }

        if (s_pendingTx.retries >= CAR_COMM_MAX_RETRIES) {
            s_pendingTx.active = false;
            s_stats.txTimeouts++;
            return;
        }

        s_pendingTx.retries++;
        if (send_frame(s_pendingTx.seq, &s_pendingTx.request.message)) {
            s_stats.txRetries++;
            s_pendingTx.sentTick = now;
        }
        return;
    }

    if (xQueueReceive(s_txQueue, &request, 0U) != pdTRUE) {
        return;
    }

    if (!send_frame(s_nextSeq, &request.message)) {
        return;
    }

    if (request.reliable && request.message.dstId != CAR_COMM_BROADCAST_ID) {
        s_pendingTx.active = true;
        s_pendingTx.request = request;
        s_pendingTx.seq = s_nextSeq;
        s_pendingTx.retries = 0U;
        s_pendingTx.sentTick = now;
    }
    s_nextSeq++;
}

bool CarComm_Init(void)
{
    if (s_txQueue != NULL || s_rxQueue != NULL) {
        return s_txQueue != NULL && s_rxQueue != NULL;
    }

    s_txQueue = xQueueCreate(CAR_COMM_TX_QUEUE_LEN, sizeof(CarComm_TxRequest_t));
    s_rxQueue = xQueueCreate(CAR_COMM_RX_QUEUE_LEN, sizeof(CarComm_Message_t));
    if (s_txQueue == NULL || s_rxQueue == NULL) {
        return false;
    }
    return true;
}

bool CarComm_Send(uint8_t dstId,
                  uint8_t cmd,
                  const uint8_t *data,
                  uint8_t len,
                  bool reliable)
{
    CarComm_TxRequest_t request;

    if (s_txQueue == NULL || len > CAR_COMM_MAX_DATA_LEN ||
        (data == NULL && len != 0U) || cmd == CAR_COMM_ACK_CMD) {
        return false;
    }

    memset(&request, 0, sizeof(request));
    request.message.dstId = dstId;
    request.message.cmd = cmd;
    request.message.len = len;
    request.reliable = reliable;
    if (len != 0U) {
        memcpy(request.message.data, data, len);
    }

    return xQueueSend(s_txQueue, &request, 0U) == pdTRUE;
}

bool CarComm_Receive(CarComm_Message_t *message, TickType_t waitTicks)
{
    if (s_rxQueue == NULL || message == NULL) {
        return false;
    }
    return xQueueReceive(s_rxQueue, message, waitTicks) == pdTRUE;
}

void CarComm_GetStats(CarComm_Stats_t *stats)
{
    if (stats == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    *stats = s_stats;
    if (s_uart != NULL) {
        stats->uartRxOverflow = s_uart->getRxOverflowCount(s_uart);
    }
    taskEXIT_CRITICAL();
}

void car_comm_task(void *pvParameters)
{
    (void)pvParameters;

    s_uart = GetCarUart();
    if (s_uart == NULL || s_txQueue == NULL || s_rxQueue == NULL) {
        LOGE(LOG_MOD_SYS, "car comm init failed\r\n");
        vTaskDelete(NULL);
        return;
    }

    s_uart->init(s_uart);
    LOGI_INIT(LOG_MOD_SYS, "car comm UART2 ready id=%u 115200 PB17/TX PB18/RX\r\n",
              (unsigned)CAR_COMM_NODE_ID);

    while (1) {
        service_rx();
        service_tx();
        vTaskDelay(pdMS_TO_TICKS(CAR_COMM_TASK_PERIOD_MS));
    }
}
