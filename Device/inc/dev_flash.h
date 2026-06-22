/**
 * @file    dev_flash.h
 * @brief   Flash Device 层抽象接口
 * @note    四层解耦架构 - Device 层
 *          使用 struct + 函数指针模拟 OOP 契约
 *          支持初始化、读JEDEC ID、扇区擦除、页编程、读取
 */

#ifndef DEV_FLASH_H
#define DEV_FLASH_H

#include <stdint.h>
#include <stdbool.h>

/* JEDEC ID 结构：3 字节（厂商 + 型号 + 容量） */
typedef struct {
    uint8_t manufacturer;   /* EF = Winbond */
    uint8_t memoryType;     /* 40 = W25Q128 */
    uint8_t capacity;       /* 18 = 128M-bit */
} DevFlash_JEDECID_t;

/* Flash 设备接口结构体（OOP 契约） */
typedef struct DevFlash {
    /**
     * @brief 初始化 Flash 硬件
     * @param self  指向自身
     */
    void (*init)(struct DevFlash *self);

    /**
     * @brief 读取 JEDEC ID
     * @param self  指向自身
     * @param id    输出 JEDEC ID 指针
     * @return true 读取成功
     */
    bool (*readJEDECID)(struct DevFlash *self, DevFlash_JEDECID_t *id);

    /**
     * @brief 扇区擦除（4KB）
     * @param self    指向自身
     * @param address 起始地址（24-bit）
     * @return true 擦除成功
     */
    bool (*sectorErase)(struct DevFlash *self, uint32_t address);

    /**
     * @brief 页编程（最多 256 字节）
     * @param self    指向自身
     * @param address 起始地址（24-bit）
     * @param data    数据指针
     * @param len     数据长度（≤256）
     * @return true 编程成功
     */
    bool (*pageProgram)(struct DevFlash *self, uint32_t address, const uint8_t *data, uint16_t len);

    /**
     * @brief 读取数据
     * @param self    指向自身
     * @param address 起始地址（24-bit）
     * @param buf     输出缓冲区
     * @param len     读取长度
     * @return true 读取成功
     */
    bool (*read)(struct DevFlash *self, uint32_t address, uint8_t *buf, uint16_t len);
} DevFlash;

/**
 * @brief 获取全局 Flash 设备句柄
 * @return DevFlash* 指向 Flash 设备接口的指针
 */
DevFlash* GetFlash(void);

#endif /* DEV_FLASH_H */