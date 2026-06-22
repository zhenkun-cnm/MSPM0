/**
 * @file    app_flash.c
 * @brief   App 层 Flash 验证任务
 * @note    四层解耦架构 - App 层
 *          通过 Device 句柄操作 Flash
 *          1. 读取 JEDEC ID
 *          2. 擦除第一页 → 写入 0x01 0x02 0x03 → 读出验证
 */

#include "app_flash.h"
#include "dev_flash.h"
#include "port_log.h"
#include "FreeRTOS.h"
#include "task.h"

#define TEST_ADDR           0x00000000  /* 第一页起始地址 */
static const uint8_t s_writeData[3] = {0x01, 0x02, 0x03};

void flash_init_task(void *pvParameters)
{
    (void)pvParameters;

    /* 获取 Flash 设备句柄 */
    DevFlash *flash = GetFlash();
    if (flash == NULL) {
        LOG_ERROR("[FLASH] Device handle is NULL!\r\n");
        vTaskDelete(NULL);
        return;
    }

    /* 初始化 Flash 硬件 */
    flash->init(flash);

    /* ────── Step 1: 读取 JEDEC ID ────── */
    DevFlash_JEDECID_t id;
    if (flash->readJEDECID(flash, &id)) {
        LOG_INFO("====================================\r\n");
        LOG_INFO("  Flash JEDEC ID: %02X %02X %02X\r\n",
                 id.manufacturer, id.memoryType, id.capacity);

        if (id.manufacturer == 0xEF && id.memoryType == 0x40) {
            uint32_t sizeMB = 0;
            if (id.capacity == 0x18) sizeMB = 16;
            else if (id.capacity == 0x17) sizeMB = 8;
            else if (id.capacity == 0x19) sizeMB = 32;
            LOG_INFO("  Detected: Winbond SPI Flash (%uMB)\r\n", sizeMB);
        } else {
            LOG_INFO("  Warning: Unknown Flash (manuf=%02X)\r\n", id.manufacturer);
        }
    } else {
        LOG_ERROR("[FLASH] Failed to read JEDEC ID!\r\n");
        vTaskDelete(NULL);
        return;
    }

    /* ────── Step 2: 擦除第一页 ────── */
    LOG_INFO("  Erasing sector 0x%08lX ...\r\n", (unsigned long)TEST_ADDR);
    flash->sectorErase(flash, TEST_ADDR);
    LOG_INFO("  Erase done\r\n");

    /* ────── Step 3: 写入 3 字节 ────── */
    LOG_INFO("  Writing: %02X %02X %02X ...\r\n",
             s_writeData[0], s_writeData[1], s_writeData[2]);
    flash->pageProgram(flash, TEST_ADDR, s_writeData, 3);
    LOG_INFO("  Write done\r\n");

    /* ────── Step 4: 读出验证 ────── */
    uint8_t readBuf[8] = {0};
    flash->read(flash, TEST_ADDR, readBuf, 8);

    LOG_INFO("  Read back: ");
    for (int i = 0; i < 8; i++) {
        LOG_RAW("%02X ", readBuf[i]);
    }
    LOG_RAW("\r\n");

    /* ────── Step 5: 结果判定 ────── */
    bool match = (readBuf[0] == s_writeData[0]) &&
                 (readBuf[1] == s_writeData[1]) &&
                 (readBuf[2] == s_writeData[2]);

    if (match) {
        LOG_INFO("  Verification: PASS\r\n");
    } else {
        LOG_INFO("  Verification: FAIL (expected %02X %02X %02X)\r\n",
                 s_writeData[0], s_writeData[1], s_writeData[2]);
    }

    LOG_INFO("====================================\r\n");

    /* 验证完毕，删除自身 */
    vTaskDelete(NULL);
}