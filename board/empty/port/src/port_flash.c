/**
 * @file    port_flash.c
 * @brief   Port 层 W25Q64 Flash 驱动实现
 * @note    实现 dev_flash.h 的 DevFlash OOP 契约
 *          使用 SPI1 硬件外设 + GPIO CS 控制
 *          调用 SysConfig 生成的 SYSCFG_DL_W25Q64_init() 初始化 SPI
 */

#include "port_flash.h"
#include "ti_msp_dl_config.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

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
    PORT_FLASH_CS_Low();
    PORT_FLASH_SPI_TransferByte(CMD_WRITE_ENABLE);
    PORT_FLASH_CS_High();
}

/* 等待 BUSY 完成，返回 false 表示超时 */
static bool cmd_wait_busy(void)
{
    uint32_t timeout;
    uint8_t sr;

    PORT_FLASH_CS_Low();
    PORT_FLASH_SPI_TransferByte(CMD_READ_STATUS1);

    timeout = SPI_TIMEOUT_CNT;
    do {
        sr = PORT_FLASH_SPI_TransferByte(0xFF);
        if (--timeout == 0) {
            PORT_FLASH_CS_High();
            return false;
        }
    } while (sr & SR1_BUSY);

    PORT_FLASH_CS_High();
    return true;
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

    /* 防错：若读回全 0xFF 或全 0x00，说明 SPI 无响应 */
    if ((buf[0] == 0xFF && buf[1] == 0xFF && buf[2] == 0xFF) ||
        (buf[0] == 0x00 && buf[1] == 0x00 && buf[2] == 0x00)) {
        return false;
    }

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

    return cmd_wait_busy();
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

    return cmd_wait_busy();
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