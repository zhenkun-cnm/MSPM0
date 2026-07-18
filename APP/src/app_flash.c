/**
 * @file    app_flash.c
 * @brief   App �?Flash 验证任务
 * @note    四层解耦架�?- App �?
 *          通过 Device 句柄操作 Flash
 *          1. 读取 JEDEC ID
 *          2. 擦除第一�?�?写入 0x01 0x02 0x03 �?读出验证
 *          支持 W25Q64�?MB, EF 40 17�?
 */

#include "app_flash.h"
#include "app_stack_monitor.h"
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
        app_stack_monitor_clear_task(APP_STACK_MON_FLASH);
        vTaskDelete(NULL);
        return;
    }

    /* 初始�?Flash 硬件 */
    flash->init(flash);

    /* ────── Step 1: 读取 JEDEC ID ────── */
    DevFlash_JEDECID_t id;
    if (flash->readJEDECID(flash, &id)) {
        LOG_INFO("====================================\r\n");
        LOG_INFO("  Flash JEDEC ID: %02X %02X %02X\r\n",
                 id.manufacturer, id.memoryType, id.capacity);

        /* W25Q64: manufacturer = EF, memoryType = 40, capacity = 17 (64M-bit / 8MB) */
        if (id.manufacturer == 0xEF && id.memoryType == 0x40) {
            uint32_t sizeMB = 0;
            if (id.capacity == 0x17) {
                sizeMB = 8;     /* W25Q64: 64M-bit = 8MB */
            } else if (id.capacity == 0x18) {
                sizeMB = 16;    /* W25Q128: 128M-bit = 16MB */
            } else if (id.capacity == 0x19) {
                sizeMB = 32;    /* W25Q256: 256M-bit = 32MB */
            } else {
                LOG_INFO("  Detected: Winbond SPI Flash (capacity=0x%02X)\r\n",
                         id.capacity);
            }
            if (sizeMB > 0) {
                LOG_INFO("  Detected: Winbond SPI Flash (%uMB)\r\n", sizeMB);
            }
        } else {
            LOG_INFO("  Warning: Unknown Flash (manuf=%02X)\r\n", id.manufacturer);
        }
    } else {
        LOG_ERROR("[FLASH] Failed to read JEDEC ID!\r\n");
        app_stack_monitor_clear_task(APP_STACK_MON_FLASH);
        vTaskDelete(NULL);
        return;
    }

    /* ────── Step 2: 擦除第一�?────── */
    LOG_INFO("  Erasing sector 0x%08lX ...\r\n", (unsigned long)TEST_ADDR);
    if (!flash->sectorErase(flash, TEST_ADDR)) {
        LOG_ERROR("[FLASH] Sector erase FAILED at 0x%08lX!\r\n",
                  (unsigned long)TEST_ADDR);
        app_stack_monitor_clear_task(APP_STACK_MON_FLASH);
        vTaskDelete(NULL);
        return;
    }
    LOG_INFO("  Erase done\r\n");

    /* ────── Step 3: 写入 3 字节 ────── */
    LOG_INFO("  Writing: %02X %02X %02X ...\r\n",
             s_writeData[0], s_writeData[1], s_writeData[2]);
    if (!flash->pageProgram(flash, TEST_ADDR, s_writeData, 3)) {
        LOG_ERROR("[FLASH] Page program FAILED!\r\n");
        app_stack_monitor_clear_task(APP_STACK_MON_FLASH);
        vTaskDelete(NULL);
        return;
    }
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

    /* 验证完毕，删除自�?*/
    app_stack_monitor_clear_task(APP_STACK_MON_FLASH);
    vTaskDelete(NULL);
}