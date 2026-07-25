/**
 * @file    port_log.h
 * @brief   Unified UART text logging and exclusive JustFloat telemetry control.
 */
#ifndef PORT_LOG_H
#define PORT_LOG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    LOG_LEVEL_OFF = 0,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_WARN,
    LOG_LEVEL_INFO,
    LOG_LEVEL_DEBUG
} LOG_Level_t;

/* Keep all levels compiled by default; runtime filtering controls emitted text. */
#ifndef LOG_COMPILE_LEVEL
#define LOG_COMPILE_LEVEL LOG_LEVEL_DEBUG
#endif

#ifndef LOG_DEFAULT_LEVEL
#define LOG_DEFAULT_LEVEL LOG_LEVEL_DEBUG
#endif

typedef enum {
    LOG_MOD_SYS      = (1UL << 0),
    LOG_MOD_CLI      = (1UL << 1),
    LOG_MOD_INS      = (1UL << 2),
    LOG_MOD_MOTION   = (1UL << 3),
    LOG_MOD_NAV      = (1UL << 4),
    LOG_MOD_PATH     = (1UL << 5),
    LOG_MOD_TEST1    = (1UL << 6),
    LOG_MOD_GRAY     = (1UL << 7),
    LOG_MOD_GRAYLINE = (1UL << 8),
    LOG_MOD_MOTOR    = (1UL << 9),
    LOG_MOD_ENCODER  = (1UL << 10),
    LOG_MOD_IMU      = (1UL << 11),
    LOG_MOD_MAG      = (1UL << 12),
    LOG_MOD_FLASH    = (1UL << 13),
    LOG_MOD_TFT      = (1UL << 14),
    LOG_MOD_BUTTON   = (1UL << 15),
    LOG_MOD_LED      = (1UL << 16),
    LOG_MOD_ALL      = ((1UL << 17) - 1UL)
} LOG_Module_t;

typedef enum {
    LOG_TELEMETRY_OFF = 0,
    LOG_TELEMETRY_MOTION,
    LOG_TELEMETRY_GRAYLINE
} LOG_Telemetry_t;

/* Must run after SYSCFG_DL_init() and before any scheduler task can log. */
void LOG_Init(void);

void LOG_SetLevel(LOG_Level_t level);
LOG_Level_t LOG_GetLevel(void);
void LOG_SetModuleEnabled(LOG_Module_t module, bool enabled);
void LOG_SetModuleMask(uint32_t mask);
uint32_t LOG_GetModuleMask(void);
bool LOG_IsModuleEnabled(LOG_Module_t module);
const char *LOG_LevelName(LOG_Level_t level);
const char *LOG_ModuleName(LOG_Module_t module);
bool LOG_LevelFromName(const char *name, LOG_Level_t *level);
bool LOG_ModuleFromName(const char *name, LOG_Module_t *module);

void LOG_SetTelemetry(LOG_Telemetry_t telemetry);
LOG_Telemetry_t LOG_GetTelemetry(void);
const char *LOG_TelemetryName(LOG_Telemetry_t telemetry);
bool LOG_TelemetryFromName(const char *name, LOG_Telemetry_t *telemetry);
bool LOG_SendTelemetryFrame(LOG_Telemetry_t source,
                            const uint8_t *data,
                            uint16_t len,
                            const uint8_t *tail,
                            uint16_t tail_len);

/* Text DMA is single-record and drop-on-busy. */
uint32_t LOG_GetDroppedCount(void);

/* Port-layer hook called by UART0_IRQHandler after RX handling. */
void LOG_UART0_IRQHandler(void);

void LOG_Write(LOG_Level_t level,
               LOG_Module_t module,
               const char *function,
               uint32_t line,
               const char *fmt,
               ...);

/*
 * Reliable text waits for a whole UART DMA record to reach EOT.  It is for
 * boot diagnostics, command replies and low-priority reports only; never use
 * it from an ISR or a real-time control loop.
 */
void LOG_WriteReliable(LOG_Level_t level,
                       LOG_Module_t module,
                       const char *function,
                       uint32_t line,
                       const char *fmt,
                       ...);

#if LOG_COMPILE_LEVEL >= LOG_LEVEL_ERROR
#define LOGE(module, fmt, ...) \
    do { LOG_WriteReliable(LOG_LEVEL_ERROR, (module), NULL, 0U, (fmt), ##__VA_ARGS__); } while (0)
#else
#define LOGE(module, fmt, ...)
#endif

#if LOG_COMPILE_LEVEL >= LOG_LEVEL_INFO
#define LOGI_RELIABLE(module, fmt, ...) \
    do { LOG_WriteReliable(LOG_LEVEL_INFO, (module), NULL, 0U, (fmt), ##__VA_ARGS__); } while (0)
#define LOGI_INIT(module, fmt, ...) \
    do { LOG_WriteReliable(LOG_LEVEL_INFO, (module), NULL, 0U, (fmt), ##__VA_ARGS__); } while (0)
#else
#define LOGI_RELIABLE(module, fmt, ...)
#define LOGI_INIT(module, fmt, ...)
#endif

#if LOG_COMPILE_LEVEL >= LOG_LEVEL_WARN
#define LOGW_RELIABLE(module, fmt, ...) \
    do { LOG_WriteReliable(LOG_LEVEL_WARN, (module), NULL, 0U, (fmt), ##__VA_ARGS__); } while (0)
#else
#define LOGW_RELIABLE(module, fmt, ...)
#endif

#if LOG_COMPILE_LEVEL >= LOG_LEVEL_WARN
#define LOGW(module, fmt, ...) \
    do { LOG_Write(LOG_LEVEL_WARN, (module), NULL, 0U, (fmt), ##__VA_ARGS__); } while (0)
#else
#define LOGW(module, fmt, ...)
#endif

#if LOG_COMPILE_LEVEL >= LOG_LEVEL_INFO
#define LOGI(module, fmt, ...) \
    do { LOG_Write(LOG_LEVEL_INFO, (module), NULL, 0U, (fmt), ##__VA_ARGS__); } while (0)
#else
#define LOGI(module, fmt, ...)
#endif

#if LOG_COMPILE_LEVEL >= LOG_LEVEL_DEBUG
#define LOGD(module, fmt, ...) \
    do { LOG_Write(LOG_LEVEL_DEBUG, (module), __FUNCTION__, __LINE__, (fmt), ##__VA_ARGS__); } while (0)
#else
#define LOGD(module, fmt, ...)
#endif

#endif /* PORT_LOG_H */
