/**
 * @file    port_log.c
 * @brief   DMA best-effort logs plus serialized reliable UART diagnostics.
 */
#include "port_log.h"
#include "ti_msp_dl_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define LOG_TEXT_BUFFER_SIZE 160U

static volatile LOG_Level_t s_level = LOG_DEFAULT_LEVEL;
static volatile uint32_t s_module_mask = LOG_MOD_ALL;
static volatile LOG_Telemetry_t s_telemetry = LOG_TELEMETRY_OFF;
static volatile uint32_t s_dropped_count = 0U;
static volatile bool s_text_busy = false;
static volatile bool s_sync_busy = false;
static volatile bool s_telemetry_sending = false;
static uint8_t s_text_buffer[LOG_TEXT_BUFFER_SIZE];
static uint8_t s_sync_buffer[LOG_TEXT_BUFFER_SIZE];
static SemaphoreHandle_t s_uart_mutex = NULL;
static volatile TaskHandle_t s_sync_waiter = NULL;

typedef struct {
    LOG_Module_t module;
    const char *name;
} LOG_ModuleEntry_t;

static const LOG_ModuleEntry_t s_modules[] = {
    {LOG_MOD_SYS, "sys"}, {LOG_MOD_CLI, "cli"}, {LOG_MOD_INS, "ins"},
    {LOG_MOD_MOTION, "motion"}, {LOG_MOD_NAV, "nav"}, {LOG_MOD_PATH, "path"},
    {LOG_MOD_TEST1, "test1"}, {LOG_MOD_GRAY, "gray"}, {LOG_MOD_GRAYLINE, "grayline"},
    {LOG_MOD_MOTOR, "motor"}, {LOG_MOD_ENCODER, "encoder"}, {LOG_MOD_IMU, "imu"},
    {LOG_MOD_MAG, "mag"}, {LOG_MOD_FLASH, "flash"}, {LOG_MOD_TFT, "tft"},
    {LOG_MOD_BUTTON, "button"}, {LOG_MOD_LED, "led"}
};

static uint32_t log_lock(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void log_unlock(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static bool log_can_wait(void)
{
    return (__get_IPSR() == 0U) && (__get_PRIMASK() == 0U) &&
           (s_uart_mutex != NULL) &&
           (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING);
}

static bool log_text_enabled(LOG_Level_t level, LOG_Module_t module)
{
    return (s_telemetry == LOG_TELEMETRY_OFF) && !s_sync_busy &&
           !s_telemetry_sending && (level != LOG_LEVEL_OFF) &&
           (level <= s_level) && (((uint32_t)module & s_module_mask) != 0U);
}

static bool log_reliable_enabled(LOG_Level_t level, LOG_Module_t module)
{
    if (level == LOG_LEVEL_ERROR) {
        return true; /* Errors are never hidden by module/level filtering. */
    }
    return (s_telemetry == LOG_TELEMETRY_OFF) && (level != LOG_LEVEL_OFF) &&
           (level <= s_level) && (((uint32_t)module & s_module_mask) != 0U);
}

static void log_start_dma(const uint8_t *buffer, uint16_t length)
{
    DL_DMA_disableChannel(DMA, DMA_CH2_CHAN_ID);
    DL_DMA_setSrcAddr(DMA, DMA_CH2_CHAN_ID, (uint32_t)buffer);
    DL_DMA_setDestAddr(DMA, DMA_CH2_CHAN_ID, (uint32_t)&sys_uart_INST->TXDATA);
    DL_DMA_setTransferSize(DMA, DMA_CH2_CHAN_ID, length);
    DL_DMA_enableChannel(DMA, DMA_CH2_CHAN_ID);
}

static void log_clear_tx_completion(void)
{
    DL_UART_Main_clearInterruptStatus(sys_uart_INST,
                                      DL_UART_MAIN_INTERRUPT_DMA_DONE_TX |
                                      DL_UART_MAIN_INTERRUPT_EOT_DONE);
}

static void log_send_blocking(const uint8_t *buffer, uint16_t length)
{
    uint16_t i;
    for (i = 0U; i < length; i++) {
        DL_UART_Main_transmitDataBlocking(sys_uart_INST, buffer[i]);
    }
}

static uint16_t log_format(uint8_t *buffer,
                           LOG_Level_t level,
                           LOG_Module_t module,
                           const char *function,
                           uint32_t line,
                           const char *fmt,
                           va_list args)
{
    static const char level_char[] = {'-', 'E', 'W', 'I', 'D'};
    int prefix_length;

    if (level == LOG_LEVEL_DEBUG && function != NULL) {
        prefix_length = snprintf((char *)buffer, LOG_TEXT_BUFFER_SIZE,
                                 "[%c][%s] %s:%lu: ", level_char[level],
                                 LOG_ModuleName(module), function, (unsigned long)line);
    } else {
        prefix_length = snprintf((char *)buffer, LOG_TEXT_BUFFER_SIZE,
                                 "[%c][%s] ", level_char[level], LOG_ModuleName(module));
    }
    if (prefix_length < 0 || (uint32_t)prefix_length >= LOG_TEXT_BUFFER_SIZE) {
        prefix_length = (int)(LOG_TEXT_BUFFER_SIZE - 1U);
    }

    (void)vsnprintf((char *)&buffer[prefix_length],
                    LOG_TEXT_BUFFER_SIZE - (uint32_t)prefix_length,
                    fmt, args);
    buffer[LOG_TEXT_BUFFER_SIZE - 1U] = '\0';
    return (uint16_t)strlen((const char *)buffer);
}

void LOG_Init(void)
{
    s_level = LOG_DEFAULT_LEVEL;
    s_module_mask = LOG_MOD_ALL;
    s_telemetry = LOG_TELEMETRY_OFF;
    s_dropped_count = 0U;
    s_text_busy = false;
    s_sync_busy = false;
    s_telemetry_sending = false;
    s_sync_waiter = NULL;
    s_uart_mutex = xSemaphoreCreateMutex();

    /* Reliable init logs need UART EOT before ins_cmd starts its RX service. */
    log_clear_tx_completion();
    NVIC_EnableIRQ(sys_uart_INST_INT_IRQN);
}

void LOG_SetLevel(LOG_Level_t level)
{
    s_level = (level <= LOG_LEVEL_DEBUG) ? level : LOG_LEVEL_DEBUG;
}

LOG_Level_t LOG_GetLevel(void)
{
    return s_level;
}

void LOG_SetModuleEnabled(LOG_Module_t module, bool enabled)
{
    if (enabled) {
        s_module_mask |= (uint32_t)module;
    } else {
        s_module_mask &= ~((uint32_t)module);
    }
}

void LOG_SetModuleMask(uint32_t mask)
{
    s_module_mask = mask & LOG_MOD_ALL;
}

uint32_t LOG_GetModuleMask(void)
{
    return s_module_mask;
}

bool LOG_IsModuleEnabled(LOG_Module_t module)
{
    return (((uint32_t)module & s_module_mask) != 0U);
}

const char *LOG_LevelName(LOG_Level_t level)
{
    switch (level) {
        case LOG_LEVEL_ERROR: return "error";
        case LOG_LEVEL_WARN:  return "warn";
        case LOG_LEVEL_INFO:  return "info";
        case LOG_LEVEL_DEBUG: return "debug";
        default:              return "off";
    }
}

const char *LOG_ModuleName(LOG_Module_t module)
{
    uint32_t i;
    for (i = 0U; i < (uint32_t)(sizeof(s_modules) / sizeof(s_modules[0])); i++) {
        if (s_modules[i].module == module) {
            return s_modules[i].name;
        }
    }
    return "unknown";
}

bool LOG_LevelFromName(const char *name, LOG_Level_t *level)
{
    if (name == NULL || level == NULL) return false;
    if (strcmp(name, "error") == 0) { *level = LOG_LEVEL_ERROR; return true; }
    if (strcmp(name, "warn") == 0)  { *level = LOG_LEVEL_WARN; return true; }
    if (strcmp(name, "info") == 0)  { *level = LOG_LEVEL_INFO; return true; }
    if (strcmp(name, "debug") == 0) { *level = LOG_LEVEL_DEBUG; return true; }
    return false;
}

bool LOG_ModuleFromName(const char *name, LOG_Module_t *module)
{
    uint32_t i;
    if (name == NULL || module == NULL) return false;
    for (i = 0U; i < (uint32_t)(sizeof(s_modules) / sizeof(s_modules[0])); i++) {
        if (strcmp(name, s_modules[i].name) == 0) {
            *module = s_modules[i].module;
            return true;
        }
    }
    return false;
}

void LOG_SetTelemetry(LOG_Telemetry_t telemetry)
{
    uint32_t primask = log_lock();
    s_telemetry = (telemetry <= LOG_TELEMETRY_GRAYLINE) ? telemetry : LOG_TELEMETRY_OFF;
    log_unlock(primask);
}

LOG_Telemetry_t LOG_GetTelemetry(void)
{
    return s_telemetry;
}

const char *LOG_TelemetryName(LOG_Telemetry_t telemetry)
{
    switch (telemetry) {
        case LOG_TELEMETRY_MOTION: return "motion";
        case LOG_TELEMETRY_GRAYLINE: return "grayline";
        default: return "off";
    }
}

bool LOG_TelemetryFromName(const char *name, LOG_Telemetry_t *telemetry)
{
    if (name == NULL || telemetry == NULL) return false;
    if (strcmp(name, "off") == 0) { *telemetry = LOG_TELEMETRY_OFF; return true; }
    if (strcmp(name, "motion") == 0) { *telemetry = LOG_TELEMETRY_MOTION; return true; }
    if (strcmp(name, "grayline") == 0) { *telemetry = LOG_TELEMETRY_GRAYLINE; return true; }
    return false;
}

static bool log_wait_for_async_idle(void)
{
    while (s_text_busy || DL_UART_Main_isBusy(sys_uart_INST)) {
        if (!log_can_wait()) return false;
        vTaskDelay(1U);
    }
    return true;
}

bool LOG_SendTelemetryFrame(LOG_Telemetry_t source,
                             const uint8_t *data,
                             uint16_t len,
                             const uint8_t *tail,
                             uint16_t tail_len)
{
    uint16_t i;
    uint32_t primask;

    if (data == NULL || tail == NULL || source == LOG_TELEMETRY_OFF || !log_can_wait()) {
        return false;
    }
    if (xSemaphoreTake(s_uart_mutex, portMAX_DELAY) != pdTRUE) return false;
    if (s_telemetry != source || !log_wait_for_async_idle()) {
        (void)xSemaphoreGive(s_uart_mutex);
        return false;
    }

    primask = log_lock();
    if (s_telemetry != source || s_sync_busy || s_telemetry_sending) {
        log_unlock(primask);
        (void)xSemaphoreGive(s_uart_mutex);
        return false;
    }
    s_telemetry_sending = true;
    log_unlock(primask);

    for (i = 0U; i < len; i++) DL_UART_Main_transmitDataBlocking(sys_uart_INST, data[i]);
    for (i = 0U; i < tail_len; i++) DL_UART_Main_transmitDataBlocking(sys_uart_INST, tail[i]);

    primask = log_lock();
    s_telemetry_sending = false;
    log_unlock(primask);
    (void)xSemaphoreGive(s_uart_mutex);
    return true;
}

uint32_t LOG_GetDroppedCount(void)
{
    uint32_t primask = log_lock();
    uint32_t dropped = s_dropped_count;
    log_unlock(primask);
    return dropped;
}

void LOG_UART0_IRQHandler(void)
{
    uint32_t status = DL_UART_Main_getEnabledInterruptStatus(
        sys_uart_INST, DL_UART_MAIN_INTERRUPT_DMA_DONE_TX | DL_UART_MAIN_INTERRUPT_EOT_DONE);

    if ((status & DL_UART_MAIN_INTERRUPT_DMA_DONE_TX) != 0U) {
        DL_UART_Main_clearInterruptStatus(sys_uart_INST, DL_UART_MAIN_INTERRUPT_DMA_DONE_TX);
    }
    if ((status & DL_UART_MAIN_INTERRUPT_EOT_DONE) != 0U) {
        BaseType_t task_woken = pdFALSE;
        TaskHandle_t waiter;

        DL_UART_Main_clearInterruptStatus(sys_uart_INST, DL_UART_MAIN_INTERRUPT_EOT_DONE);
        DL_DMA_disableChannel(DMA, DMA_CH2_CHAN_ID);
        if (s_sync_busy) {
            waiter = (TaskHandle_t)s_sync_waiter;
            s_sync_waiter = NULL;
            s_sync_busy = false;
            if (waiter != NULL) {
                vTaskNotifyGiveFromISR(waiter, &task_woken);
            }
        } else {
            s_text_busy = false;
        }
        portYIELD_FROM_ISR(task_woken);
    }
}

void LOG_Write(LOG_Level_t level,
               LOG_Module_t module,
               const char *function,
               uint32_t line,
               const char *fmt,
               ...)
{
    uint32_t primask;
    uint16_t length;
    va_list args;

    if (fmt == NULL) return;

    primask = log_lock();
    if ((__get_IPSR() != 0U) || !log_text_enabled(level, module) || s_text_busy ||
        DL_UART_Main_isBusy(sys_uart_INST)) {
        s_dropped_count++;
        log_unlock(primask);
        return;
    }
    log_clear_tx_completion();
    s_text_busy = true;
    log_unlock(primask);

    va_start(args, fmt);
    length = log_format(s_text_buffer, level, module, function, line, fmt, args);
    va_end(args);

    primask = log_lock();
    if (!log_text_enabled(level, module) || length == 0U) {
        s_text_busy = false;
        s_dropped_count++;
        log_unlock(primask);
        return;
    }
    log_start_dma(s_text_buffer, length);
    log_unlock(primask);
}

void LOG_WriteReliable(LOG_Level_t level,
                       LOG_Module_t module,
                       const char *function,
                       uint32_t line,
                       const char *fmt,
                       ...)
{
    uint32_t primask;
    uint16_t length;
    bool error_log = (level == LOG_LEVEL_ERROR);
    va_list args;

    if (fmt == NULL || __get_IPSR() != 0U) return;

    primask = log_lock();
    if (error_log) s_telemetry = LOG_TELEMETRY_OFF;
    if (!log_reliable_enabled(level, module)) {
        log_unlock(primask);
        return;
    }
    log_unlock(primask);

    if (!log_can_wait()) {
        /* Scheduler/IRQ-off boot phase: only one task runs, direct UART is safe. */
        va_start(args, fmt);
        length = log_format(s_sync_buffer, level, module, function, line, fmt, args);
        va_end(args);
        if (length != 0U) log_send_blocking(s_sync_buffer, length);
        return;
    }

    if (xSemaphoreTake(s_uart_mutex, portMAX_DELAY) != pdTRUE) return;

    if (!error_log && s_telemetry != LOG_TELEMETRY_OFF) {
        (void)xSemaphoreGive(s_uart_mutex);
        return;
    }
    while (1) {
        if (!log_wait_for_async_idle()) {
            (void)xSemaphoreGive(s_uart_mutex);
            return;
        }
        primask = log_lock();
        if (!s_text_busy && !s_telemetry_sending &&
            !DL_UART_Main_isBusy(sys_uart_INST)) {
            log_clear_tx_completion();
            s_sync_busy = true;
            s_sync_waiter = xTaskGetCurrentTaskHandle();
            log_unlock(primask);
            break;
        }
        log_unlock(primask);
    }

    va_start(args, fmt);
    length = log_format(s_sync_buffer, level, module, function, line, fmt, args);
    va_end(args);
    if (length == 0U) {
        primask = log_lock();
        s_sync_waiter = NULL;
        s_sync_busy = false;
        log_unlock(primask);
        (void)xSemaphoreGive(s_uart_mutex);
        return;
    }

    (void)ulTaskNotifyTake(pdTRUE, 0U);
    primask = log_lock();
    log_start_dma(s_sync_buffer, length);
    log_unlock(primask);
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    (void)xSemaphoreGive(s_uart_mutex);
}
