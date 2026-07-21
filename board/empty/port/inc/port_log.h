/**
 * @file    port_log.h
 * @brief   Port 层系统日志工具
 * @note    全局日志宏，无需获取对象句柄
 *          编译期可通过 LOG_LEVEL 控制调试输出级别
 *          用法: LOG_INFO("System boot ok\r\n");
 *                LOG_ERROR("UART timeout, err=%d\r\n", err);
 *                LOG_DEBUG("LED toggle count=%u\r\n", cnt);
 */

#ifndef PORT_LOG_H
#define PORT_LOG_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* 日志级别定义 */
#ifndef LOG_LEVEL
#define LOG_LEVEL   3   /* 默认 INFO 级别 */
#endif

#define LOG_LEVEL_RAW   1
#define LOG_LEVEL_ERROR 2
#define LOG_LEVEL_INFO  3
#define LOG_LEVEL_DEBUG 4

/* 全局 LOG 抑制标志：置 true 后所有 LOG_RAW/INFO/ERROR/DEBUG 静默 */
extern bool g_log_suppress;

/* 底层输出函数声明 */
void LOG_OutputChar(char c);

/* ──────── 日志宏 ──────── */

#if LOG_LEVEL >= LOG_LEVEL_RAW
    #define LOG_RAW(fmt, ...)   do { \
        if (!g_log_suppress) { log_printf_internal(fmt, ##__VA_ARGS__); } \
    } while(0)
#else
    #define LOG_RAW(fmt, ...)
#endif

#if LOG_LEVEL >= LOG_LEVEL_ERROR
    #define LOG_ERROR(fmt, ...)  do { \
        if (!g_log_suppress) { log_printf_internal("[ERR] " fmt, ##__VA_ARGS__); } \
    } while(0)
#else
    #define LOG_ERROR(fmt, ...)
#endif

#if LOG_LEVEL >= LOG_LEVEL_INFO
    #define LOG_INFO(fmt, ...)   do { \
        if (!g_log_suppress) { log_printf_internal("[INFO] " fmt, ##__VA_ARGS__); } \
    } while(0)
#else
    #define LOG_INFO(fmt, ...)
#endif

#if LOG_LEVEL >= LOG_LEVEL_DEBUG
    #define LOG_DEBUG(fmt, ...)  do { \
        if (!g_log_suppress) { log_printf_internal("[DBG] %s:%d: " fmt, __FUNCTION__, __LINE__, ##__VA_ARGS__); } \
    } while(0)
#else
    #define LOG_DEBUG(fmt, ...)
#endif

/* ──────── 周期性信息打印开关（独立控制） ──────── */
#ifndef LOG_PRINT_INS_ENABLE
#define LOG_PRINT_INS_ENABLE     0   /* app_ins.c: [INS] 位姿 */
#endif
#ifndef LOG_PRINT_ATT_ENABLE
#define LOG_PRINT_ATT_ENABLE     0   /* app_imu.c: [ATT] 姿态 */
#endif
#ifndef LOG_PRINT_MOTOR_ENABLE
#define LOG_PRINT_MOTOR_ENABLE   0   /* app_motor_encoder.c: M1/M2 编码器 */
#endif
#ifndef LOG_PRINT_PID_ENABLE
#define LOG_PRINT_PID_ENABLE    0    /* app_motion.c: VOFA+ JustFloat */
#endif

/**
 * @brief 内部 printf 格式化输出引擎
 */
void log_printf_internal(const char *fmt, ...);

/**
 * @brief JustFloat 原始字节块发送（无格式化开销）
 * @note  直接调用 DL_UART_Main_transmitDataBlocking，适用于 VOFA+ 等二进制协议
 * @param data  字节数组指针
 * @param len   字节数
 */
void LOG_SendRawBytes(const uint8_t *data, uint16_t len);

#endif /* PORT_LOG_H */