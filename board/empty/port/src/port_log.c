/**
 * @file    port_log.c
 * @brief   Port 层系统日志实现
 * @note    基于 UART0 串口输出，阻塞式逐字节发送
 *          依赖 ti_msp_dl_config.h 中 UART 初始化完成
 */

#include "port_log.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <stdarg.h>
#include <stdio.h>

#if defined(__ARMCC_VERSION)
__asm(".global __use_no_semihosting\n");

struct __FILE {
    int handle;
};

FILE __stdout;

int fputc(int ch, FILE *stream)
{
    (void)stream;
    LOG_OutputChar((char)ch);
    return ch;
}

void _sys_exit(int return_code)
{
    (void)return_code;
    while (1) {
    }
}
#endif

/* ========== 底层 UART 输出 ========== */

void LOG_OutputChar(char c)
{
    /* 阻塞发送一个字节 */
    DL_UART_Main_transmitDataBlocking(UART0, (uint8_t)c);

    /* 换行符时自动输出回车，兼容终端 */
    if (c == '\n') {
        DL_UART_Main_transmitDataBlocking(UART0, (uint8_t)'\r');
    }
}

/* ========== 格式化输出引擎 ========== */

void log_printf_internal(const char *fmt, ...)
{
    char buf[128];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    for (char *p = buf; *p != '\0'; p++) {
        LOG_OutputChar(*p);
    }
}
