/**
 * @file    dev_tft.h
 * @brief   TFT Display (ST7735) Device 层抽象接口
 * @note    四层解耦架构 - Device 层
 *          使用 struct + 函数指针模拟 OOP 契约
 */

#ifndef DEV_TFT_H
#define DEV_TFT_H

#include <stdint.h>
#include <stdbool.h>

/* TFT 屏幕参数 */
#define TFT_WIDTH       160
#define TFT_HEIGHT      80

/* TFT 色彩定义 (RGB565) */
#define TFT_BLACK       0x0000
#define TFT_WHITE       0xFFFF
#define TFT_RED         0xF800
#define TFT_GREEN       0x07E0
#define TFT_BLUE        0x001F
#define TFT_CYAN        0x07FF
#define TFT_MAGENTA     0xF81F
#define TFT_YELLOW      0xFFE0
#define TFT_ORANGE      0xFD20
#define TFT_GRAY        0x8410

/* TFT 设备接口结构体（OOP 契约） */
typedef struct DevTFT {
    void (*init)(struct DevTFT *self);
    void (*fillScreen)(struct DevTFT *self, uint16_t color);
    void (*drawPixel)(struct DevTFT *self, uint16_t x, uint16_t y, uint16_t color);
    void (*drawChar)(struct DevTFT *self, uint16_t x, uint16_t y, char ch,
                     uint16_t color, uint16_t bg);
    void (*printString)(struct DevTFT *self, uint16_t x, uint16_t y,
                        const char *str, uint16_t color, uint16_t bg);
    void (*setCursor)(struct DevTFT *self, uint16_t x, uint16_t y);
    void (*print)(struct DevTFT *self, const char *str, uint16_t color, uint16_t bg);
    void (*fillRect)(struct DevTFT *self, uint16_t x, uint16_t y,
                     uint16_t w, uint16_t h, uint16_t color);
    /* printf 风格格式化输出（内部 vsnprintf 后调用 printString） */
    void (*printf)(struct DevTFT *self, uint16_t x, uint16_t y,
                   uint16_t color, uint16_t bg, const char *fmt, ...);
} DevTFT;

/**
 * @brief 获取全局 TFT 设备句柄
 * @return DevTFT* 指向 TFT 设备接口的指针
 */
DevTFT* GetTFT(void);

#endif /* DEV_TFT_H */