/**
 * @file    port_flash.c
 * @brief   Port 层 W25Q128 Flash 驱动实现
 * @note    实现 dev_flash.h 的 DevFlash OOP 契约
 *          使用 SPI1 硬件外设 + GPIO CS 控制
 *          调用 SysConfig 生成的 SYSCFG_DL_W25Q64_init() 初始化 SPI
 */

#include "port_flash.h"
#include "port_imu.h"       /* g_spi1_mutex */
#include "ti_msp_dl_config.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

/* Flash 操作宏：自动加锁/解锁 SPI1 */
#define SPI1_LOCK()   do { if (g_spi1_mutex) xSemaphoreTake(g_spi1_mutex, pdMS_TO_TICKS(500)); } while(0)
#define SPI1_UNLOCK() do { if (g_spi1_mutex) xSemaphoreGive(g_spi1_mutex); } while(0)

/* W25Q 指令集 */
#define CMD_WRITE_ENABLE    0x06    /* 写使能 */
#define CMD_SECTOR_ERASE    0x20    /* 4KB 扇区擦除 */
#define CMD_PAGE_PROGRAM    0x02    /* 页编程（≤256 字节） */
#define CMD_READ_DATA       0x03    /* 读取数据 */
#define CMD_READ_STATUS1    0x05    /* 读状态寄存器 1 */

/* 状态寄存器位 */
#define SR1_BUSY            (1 << 0)  /* BUSY 位 */
#define SR1_WEL             (1 << 1)  /* 写使能锁存位 */

/* SPI 超时计数器 */
#define SPI_TIMEOUT_CNT     50000U

/* Device 层接口实例 */
static DevFlash s_flashIf;

/* ================================================================
 *  CS GPIO 控制
 * ================================================================ */

void PORT_FLASH_CS_Low(void)
{
    DL_GPIO_clearPins(CS_PORT, CS_SPI1_CS_PIN);
}

void PORT_FLASH_CS_High(void)
{
    DL_GPIO_setPins(CS_PORT, CS_SPI1_CS_PIN);
}

/* ================================================================
 *  SPI 单字节收发
 * ================================================================ */

uint8_t PORT_FLASH_SPI_TransferByte(uint8_t tx)
{
    uint32_t timeout;

    /* 等待 TX FIFO 可写 */
    timeout = SPI_TIMEOUT_CNT;
    while (!DL_SPI_isTXFIFOEmpty(W25Q64_INST)) {
        if (--timeout == 0) break;
    }

    /* 发送字节 */
    DL_SPI_transmitDataBlocking8(W25Q64_INST, tx);

    /* 等待 RX FIFO 有数据 */
    timeout = SPI_TIMEOUT_CNT;
    while (DL_SPI_isRXFIFOEmpty(W25Q64_INST)) {
        if (--timeout == 0) break;
    }

    /* 读取接收字节 */
    return DL_SPI_receiveDataBlocking8(W25Q64_INST);
}

/* ================================================================
 *  底层 Flash 操作函数
 * ================================================================ */

/* 写使能 */
static void cmd_write_enable(void)
{
    SPI1_LOCK();
    PORT_FLASH_CS_Low();
    PORT_FLASH_SPI_TransferByte(CMD_WRITE_ENABLE);
    PORT_FLASH_CS_High();
    SPI1_UNLOCK();
}

/* 等待 BUSY 完成 */
static void cmd_wait_busy(void)
{
    uint32_t timeout;

    SPI1_LOCK();
    PORT_FLASH_CS_Low();
    PORT_FLASH_SPI_TransferByte(CMD_READ_STATUS1);

    timeout = SPI_TIMEOUT_CNT;
    while (PORT_FLASH_SPI_TransferByte(0xFF) & SR1_BUSY) {
        if (--timeout == 0) break;
    }

    PORT_FLASH_CS_High();
    SPI1_UNLOCK();
}

/* 发送 24-bit 地址（MSB first） */
static void send_address(uint32_t addr)
{
    PORT_FLASH_SPI_TransferByte((uint8_t)(addr >> 16));
    PORT_FLASH_SPI_TransferByte((uint8_t)(addr >> 8));
    PORT_FLASH_SPI_TransferByte((uint8_t)(addr));
}

/* ================================================================
 *  DevFlash 接口函数实现
 * ================================================================ */

static void flash_init(DevFlash *self)
{
    (void)self;

    /* 初始化 CS 引脚为输出（默认高电平） */
    DL_GPIO_initDigitalOutput(CS_SPI1_CS_IOMUX);
    DL_GPIO_setPins(CS_PORT, CS_SPI1_CS_PIN);
    DL_GPIO_enableOutput(CS_PORT, CS_SPI1_CS_PIN);

    /* SPI1 初始化由 SysConfig 生成 */
    SYSCFG_DL_W25Q64_init();
}

static bool flash_readJEDECID(DevFlash *self, DevFlash_JEDECID_t *id)
{
    uint8_t buf[3];

    if (id == NULL) {
        return false;
    }

    PORT_FLASH_CS_Low();
    PORT_FLASH_SPI_TransferByte(0x9F);     /* JEDEC ID 命令 */
    buf[0] = PORT_FLASH_SPI_TransferByte(0xFF);
    buf[1] = PORT_FLASH_SPI_TransferByte(0xFF);
    buf[2] = PORT_FLASH_SPI_TransferByte(0xFF);
    PORT_FLASH_CS_High();

    id->manufacturer = buf[0];
    id->memoryType   = buf[1];
    id->capacity     = buf[2];

    return true;
}

static bool flash_sectorErase(DevFlash *self, uint32_t address)
{
    (void)self;

    cmd_write_enable();

    PORT_FLASH_CS_Low();
    PORT_FLASH_SPI_TransferByte(CMD_SECTOR_ERASE);
    send_address(address);
    PORT_FLASH_CS_High();

    cmd_wait_busy();

    return true;
}

static bool flash_pageProgram(DevFlash *self, uint32_t address,
                              const uint8_t *data, uint16_t len)
{
    uint16_t i;

    (void)self;

    if (data == NULL || len == 0 || len > 256) {
        return false;
    }

    cmd_write_enable();

    PORT_FLASH_CS_Low();
    PORT_FLASH_SPI_TransferByte(CMD_PAGE_PROGRAM);
    send_address(address);
    for (i = 0; i < len; i++) {
        PORT_FLASH_SPI_TransferByte(data[i]);
    }
    PORT_FLASH_CS_High();

    cmd_wait_busy();

    return true;
}

static bool flash_read(DevFlash *self, uint32_t address,
                       uint8_t *buf, uint16_t len)
{
    uint16_t i;

    (void)self;

    if (buf == NULL || len == 0) {
        return false;
    }

    PORT_FLASH_CS_Low();
    PORT_FLASH_SPI_TransferByte(CMD_READ_DATA);
    send_address(address);
    for (i = 0; i < len; i++) {
        buf[i] = PORT_FLASH_SPI_TransferByte(0xFF);
    }
    PORT_FLASH_CS_High();

    return true;
}

/* ================================================================
 *  注册 Device 接口
 * ================================================================ */

DevFlash* GetFlash(void)
{
    s_flashIf.init         = flash_init;
    s_flashIf.readJEDECID  = flash_readJEDECID;
    s_flashIf.sectorErase  = flash_sectorErase;
    s_flashIf.pageProgram  = flash_pageProgram;
    s_flashIf.read         = flash_read;

    return &s_flashIf;
}