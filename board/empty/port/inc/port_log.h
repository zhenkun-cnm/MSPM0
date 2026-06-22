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

/* 底层输出函数声明 */
void LOG_OutputChar(char c);

/* ──────── 日志宏 ──────── */

#if LOG_LEVEL >= LOG_LEVEL_RAW
    #define LOG_RAW(fmt, ...)   do { \
        log_printf_internal(fmt, ##__VA_ARGS__); \
    } while(0)
#else
    #define LOG_RAW(fmt, ...)
#endif

#if LOG_LEVEL >= LOG_LEVEL_ERROR
    #define LOG_ERROR(fmt, ...)  do { \
        log_printf_internal("[ERR] " fmt, ##__VA_ARGS__); \
    } while(0)
#else
    #define LOG_ERROR(fmt, ...)
#endif

#if LOG_LEVEL >= LOG_LEVEL_INFO
    #define LOG_INFO(fmt, ...)   do { \
        log_printf_internal("[INFO] " fmt, ##__VA_ARGS__); \
    } while(0)
#else
    #define LOG_INFO(fmt, ...)
#endif

#if LOG_LEVEL >= LOG_LEVEL_DEBUG
    #define LOG_DEBUG(fmt, ...)  do { \
        log_printf_internal("[DBG] %s:%d: " fmt, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
    } while(0)
#else
    #define LOG_DEBUG(fmt, ...)
#endif

/**
 * @brief 内部 printf 格式化输出引擎
 */
void log_printf_internal(const char *fmt, ...);

#endif /* PORT_LOG_H */