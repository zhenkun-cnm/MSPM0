/**
 * @file    port_imu.c
 * @brief   Port 层 ICM-20608 硬件 SPI 驱动实现
 * @note    实现 dev_imu.h 的 DevIMU OOP 契约
 *          使用 SPI1 硬件外设 + GPIO CS PB6（与 W25Q64 共享 SPI1 总线）
 *
 *          严格按 ICM-20608 手册:
 *          - §6.5 SPI Interface (p.28): R/W + 7bit addr, MSB 先, 上升沿锁存
 *          - §3.5 SPI Timing (p.15): Mode 0/3, 最大 8MHz
 *          - §4.10 Clocking (p.22): 推荐 CLKSEL=1 自动选时钟
 *          - §6.1 (p.25): SPI 下必须写 I2C_IF_DIS 禁用 I2C
 *          - Table 1 (p.7): 陀螺量程 FS_SEL[1:0]
 *          - Table 2 (p.8): 加计量程 AFS_SEL[1:0]
 *
 *          ICM-20608 无 Bank 系统（与 ICM-20948 不同），寄存器直访。
 */

#include "port_imu.h"
#include "port_log.h"
#include "ti_msp_dl_config.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include "FreeRTOS.h"
#include "semphr.h"

/* SPI1 全局互斥锁 — IMU 和 Flash 共享总线时保护 */
SemaphoreHandle_t g_spi1_mutex = NULL;

/* 获取 SPI1 总线（最多等 100ms） */
static inline bool spi1_lock(void)
{
    if (g_spi1_mutex == NULL) return true; /* Not created yet, single-threaded */
    return (xSemaphoreTake(g_spi1_mutex, pdMS_TO_TICKS(100)) == pdTRUE);
}
static inline void spi1_unlock(void)
{
    if (g_spi1_mutex != NULL) xSemaphoreGive(g_spi1_mutex);
}

/* ================================================================
 *  CS 引脚定义 — PB6 (IOMUX_PINCM23)
 * ================================================================ */
#define IMU_CS_PORT          GPIOB
#define IMU_CS_PIN           DL_GPIO_PIN_6
#define IMU_CS_IOMUX         (IOMUX_PINCM23)

/* ================================================================
 *  ICM-20608 寄存器地址（手册 §6 及标准寄存器映射）
 * ================================================================ */
#define REG_WHO_AM_I         0x75    /* 器件 ID，读回 0xAE */
#define REG_USER_CTRL        0x03    /* bit4: I2C_IF_DIS 禁用 I2C (p.25) */
#define REG_PWR_MGMT_1       0x6B    /* 电源管理 (p.24) */
#define REG_PWR_MGMT_2       0x6C    /* 电源管理 2 (p.24) */
#define REG_SMPLRT_DIV       0x19    /* 采样率分频 */
#define REG_CONFIG           0x1A    /* DLPF 配置 */
#define REG_GYRO_CONFIG      0x1B    /* 陀螺配置 FS_SEL[1:0] (p.7) */
#define REG_ACCEL_CONFIG     0x1C    /* 加计配置 AFS_SEL[1:0] (p.8) */
#define REG_ACCEL_CONFIG2    0x1D    /* 加计配置 2 (p.24) */

/* 传感器数据寄存器 — 手册 §4.11 Sensor Data Registers (p.22) */
#define REG_ACCEL_XOUT_H     0x3B    /* 加速度计 X 高字节，连续 6 字节 */
#define REG_GYRO_XOUT_H      0x43    /* 陀螺仪 X 高字节，连续 6 字节 */
#define REG_TEMP_OUT_H       0x41    /* 温度传感器高字节，连续 2 字节 */

/* --- PWR_MGMT_1 (0x6B) 位定义 (p.24) --- */
#define PWR_MGMT_1_DEVICE_RESET  (1 << 7)  /* bit7: 自清复位 */
#define PWR_MGMT_1_SLEEP         (1 << 6)  /* bit6: 休眠 */
#define PWR_MGMT_1_CLKSEL_AUTO   (0x01)    /* CLKSEL[2:0]=001 自动选时钟 (p.22) */

/* --- USER_CTRL (0x03) 位定义 (p.25) --- */
#define USER_CTRL_I2C_IF_DIS     (1 << 4)  /* bit4: 禁用 I2C 强制 SPI */

/* --- 量程配置 (p.7, p.8) --- */
#define ACCEL_FS_2G              0x00      /* AFS_SEL=00: ±2g, 16384 LSB/g */
#define ACCEL_FS_4G              0x08      /* AFS_SEL=01: ±4g */
#define ACCEL_FS_8G              0x10      /* AFS_SEL=10: ±8g */
#define ACCEL_FS_16G             0x18      /* AFS_SEL=11: ±16g */

#define GYRO_FS_250DPS           0x00      /* FS_SEL=00: ±250°/s, 131 LSB/(°/s) */
#define GYRO_FS_500DPS           0x08      /* FS_SEL=01: ±500°/s */
#define GYRO_FS_1000DPS          0x10      /* FS_SEL=10: ±1000°/s */
#define GYRO_FS_2000DPS          0x18      /* FS_SEL=11: ±2000°/s */

/* SPI 收发超时 */
#define SPI_TIMEOUT_CNT          50000U

/* Device 层接口实例 */
static DevIMU s_imuIf;

/* ================================================================
 *  CS 控制
 * ================================================================ */

void PORT_IMU_CS_Low(void)
{
    DL_GPIO_clearPins(IMU_CS_PORT, IMU_CS_PIN);
}

void PORT_IMU_CS_High(void)
{
    DL_GPIO_setPins(IMU_CS_PORT, IMU_CS_PIN);
}

/* ================================================================
 *  SPI 单字节收发（复用 SPI1 硬件外设）
 *
 *  手册 §6.5 (p.28):
 *  - MSB 先
 *  - 上升沿锁存（Mode 0: CPOL=0, CPHA=0）
 *  - W25Q64_init() 已配置 SPI1 为 Mode 0，可直接复用
 * ================================================================ */

uint8_t PORT_IMU_SPI_TransferByte(uint8_t tx)
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
 *  寄存器读写
 *
 *  手册 §6.5 (p.28) SPI Address 格式:
 *  MSB                     LSB
 *  R/W  A6  A5  A4  A3  A2  A1  A0
 *  - R/W=0: 写操作
 *  - R/W=1: 读操作
 *  每次传输在 CS 拉低期间完成，至少 16 个 SCLK 周期（2 字节）
 * ================================================================ */

void PORT_IMU_WriteReg(uint8_t regAddr, uint8_t data)
{
    /* 手册 §6.5: R/W=0 + 7bit 地址 */
    uint8_t addrByte = regAddr & 0x7F;

    PORT_IMU_CS_Low();
    PORT_IMU_SPI_TransferByte(addrByte);
    PORT_IMU_SPI_TransferByte(data);
    PORT_IMU_CS_High();
}

uint8_t PORT_IMU_ReadReg(uint8_t regAddr)
{
    /* 手册 §6.5: R/W=1 + 7bit 地址 */
    uint8_t addrByte = 0x80 | (regAddr & 0x7F);
    uint8_t data;

    PORT_IMU_CS_Low();
    PORT_IMU_SPI_TransferByte(addrByte);
    data = PORT_IMU_SPI_TransferByte(0xFF);  /* dummy 字节，同时接收 */
    PORT_IMU_CS_High();

    return data;
}

void PORT_IMU_ReadRegs(uint8_t regAddr, uint8_t *buf, uint16_t len)
{
    uint8_t addrByte = 0x80 | (regAddr & 0x7F);
    uint16_t i;

    if (buf == NULL || len == 0) return;

    PORT_IMU_CS_Low();
    PORT_IMU_SPI_TransferByte(addrByte);

    for (i = 0; i < len; i++) {
        buf[i] = PORT_IMU_SPI_TransferByte(0xFF);
    }

    PORT_IMU_CS_High();
}

/* ================================================================
 *  延时函数
 * ================================================================ */

/* ~1us 延时 @ 80MHz */
static void delay_1us(void)
{
    /* 80 NOP @ 80MHz ≈ 1us */
    for (volatile uint32_t i = 0; i < 80; i++) {
        __NOP();
    }
}

static void delay_ms(uint32_t ms)
{
    for (volatile uint32_t i = 0; i < ms; i++) {
        for (volatile uint32_t j = 0; j < 1000; j++) {
            delay_1us();
        }
    }
}

/* ================================================================
 *  CS GPIO 初始化
 *
 *  SPI1 硬件外设由 SysConfig 生成的 W25Q64_init() 初始化（Mode 0），
 *  ICM-20608 复用同一总线，只需配置独立 CS 引脚。
 * ================================================================ */

void PORT_IMU_InitGPIO(void)
{
    /* CS = PB6 — 输出，默认高电平（未选中） */
    DL_GPIO_initDigitalOutput(IMU_CS_IOMUX);
    PORT_IMU_CS_High();
    DL_GPIO_enableOutput(IMU_CS_PORT, IMU_CS_PIN);
}

/* ================================================================
 *  DevIMU 接口函数实现
 * ================================================================ */

/**
 * @brief 软复位并初始化 ICM-20608
 *
 * 手册初始化流程:
 * 1. 上电等待 100ms（寄存器读写启动时间，p.11）
 * 2. 软复位: PWR_MGMT_1 bit7=1，等待自清 (p.24)
 * 3. 退出睡眠: CLKSEL=001 自动时钟 (p.22 §4.10)
 * 4. 禁用 I2C: USER_CTRL bit4=1 (p.25 §6.1)
 * 5. 配置量程: ACCEL_CONFIG / GYRO_CONFIG (p.7, p.8)
 */
static bool g_imu_init_ok = false;

static void imu_init(DevIMU *self)
{
    uint8_t rb;
    uint8_t whoami;
    (void)self;

    LOG_INFO("========================================\r\n");
    LOG_INFO("[IMU] ICM-20608 Init Start\r\n");
    LOG_INFO("[IMU] CS=PB6, SPI=SPI1 (shared with W25Q64)\r\n");
    LOG_INFO("========================================\r\n");

    /* 1. 初始化 CS 引脚 */
    PORT_IMU_InitGPIO();
    LOG_INFO("[IMU] CS GPIO init done (PB6)\r\n");

    /* 2. 上电等待 ≥100ms（手册 Table 6 p.11） */
    LOG_INFO("[IMU] Wait 100ms for power-up...\r\n");
    delay_ms(100);

    /* 3. 软复位 */
    LOG_INFO("[IMU] Device reset...\r\n");
    PORT_IMU_WriteReg(REG_PWR_MGMT_1, PWR_MGMT_1_DEVICE_RESET);
    delay_ms(100);

    /* 4. 退出睡眠，CLKSEL=1 自动选最佳时钟源 (§4.10 p.22) */
    PORT_IMU_WriteReg(REG_PWR_MGMT_1, PWR_MGMT_1_CLKSEL_AUTO);
    delay_ms(10);
    rb = PORT_IMU_ReadReg(REG_PWR_MGMT_1);
    if (rb != PWR_MGMT_1_CLKSEL_AUTO) {
        LOG_ERROR("[IMU] FAIL: PWR_MGMT_1 wrote 0x%02X read 0x%02X\r\n",
                  PWR_MGMT_1_CLKSEL_AUTO, rb);
        g_imu_init_ok = false;
        return;
    }
    LOG_INFO("[IMU] PWR_MGMT_1 OK (CLKSEL=auto)\r\n");

    /* 5. 禁用 I2C 接口，强制 SPI（手册 §6.1 p.25 明确要求） */
    PORT_IMU_WriteReg(REG_USER_CTRL, USER_CTRL_I2C_IF_DIS);
    delay_ms(1);
    rb = PORT_IMU_ReadReg(REG_USER_CTRL);
    if (rb != USER_CTRL_I2C_IF_DIS) {
        LOG_ERROR("[IMU] FAIL: USER_CTRL wrote 0x%02X read 0x%02X\r\n",
                  USER_CTRL_I2C_IF_DIS, rb);
        g_imu_init_ok = false;
        return;
    }
    LOG_INFO("[IMU] USER_CTRL OK (I2C disabled, SPI only)\r\n");

    /* 6. WHO_AM_I 验证 */
    whoami = PORT_IMU_ReadReg(REG_WHO_AM_I);
    if (whoami == 0xAF) {
        LOG_INFO("[IMU] WHO_AM_I OK: 0x%02X (ICM-20608 confirmed)\r\n", whoami);
    } else {
        LOG_ERROR("[IMU] FAIL: WHO_AM_I expected 0xAF got 0x%02X\r\n", whoami);
        g_imu_init_ok = false;
        return;
    }

    /* 7. 配置加速度计量程 ±2g（手册 Table 2 p.8） */
    PORT_IMU_WriteReg(REG_ACCEL_CONFIG, ACCEL_FS_2G);
    LOG_INFO("[IMU] ACCEL_CONFIG: +/-2g\r\n");

    /* 8. 配置陀螺仪量程 ±250°/s（手册 Table 1 p.7） */
    PORT_IMU_WriteReg(REG_GYRO_CONFIG, GYRO_FS_250DPS);
    LOG_INFO("[IMU] GYRO_CONFIG: +/-250dps\r\n");

    /* 9. 配置采样率分频 & DLPF */
    PORT_IMU_WriteReg(REG_SMPLRT_DIV, 0x00);
    PORT_IMU_WriteReg(REG_CONFIG, 0x00);
    LOG_INFO("[IMU] ODR=1kHz, DLPF=250Hz\r\n");

    g_imu_init_ok = true;
    LOG_INFO("========================================\r\n");
    LOG_INFO("[IMU] ICM-20608 Init SUCCESS\r\n");
    LOG_INFO("========================================\r\n");
}

/**
 * @brief 读取 WHO_AM_I 器件 ID
 * @return 0xAE = ICM-20608 (手册 Part Number p.32: IC268G marking)
 */
static uint8_t imu_whoAmI(DevIMU *self)
{
    (void)self;
    uint8_t id = 0;

    /* 先检查 init 状态 */
    if (!g_imu_init_ok) {
        LOG_ERROR("[IMU] whoAmI called but init not OK\r\n");
        return 0;
    }

    /* 多次重试，避免上电瞬态误判 */
    for (int i = 0; i < 5; i++) {
        id = PORT_IMU_ReadReg(REG_WHO_AM_I);
        if (id == 0xAF) {
            LOG_INFO("[IMU] whoAmI: 0x%02X OK\r\n", id);
            return id;
        }
        delay_ms(10);
    }
    LOG_ERROR("[IMU] whoAmI: expected 0xAF got 0x%02X\r\n", id);
    return id;
}

/**
 * @brief 读取 6 轴传感器数据
 *
 * 手册 §4.11 (p.22): 传感器数据寄存器为只读，可通过串口随时读取。
 * 数据格式: 16-bit 二进制补码，MSB 先。
 *
 * 从 0x3B 连续读取 14 字节:
 * ACCEL_X[15:8], ACCEL_X[7:0]     (0x3B-0x3C)
 * ACCEL_Y[15:8], ACCEL_Y[7:0]     (0x3D-0x3E)
 * ACCEL_Z[15:8], ACCEL_Z[7:0]     (0x3F-0x40)
 * TEMP[15:8],  TEMP[7:0]          (0x41-0x42)
 * GYRO_X[15:8], GYRO_X[7:0]      (0x43-0x44)
 * GYRO_Y[15:8], GYRO_Y[7:0]      (0x45-0x46)
 * GYRO_Z[15:8], GYRO_Z[7:0]      (0x47-0x48)
 */
static bool imu_readSensorData(DevIMU *self, DevIMU_SensorData_t *data)
{
    uint8_t buf[14];

    if (data == NULL) return false;

    /* 突发读 14 字节，从 ACCEL_XOUT_H (0x3B) 开始 */
    PORT_IMU_ReadRegs(REG_ACCEL_XOUT_H, buf, 14);

    /* 解析加速度计（手册 Table 2 p.8: 16-bit 二进制补码） */
    data->accelX = (int16_t)((buf[0] << 8) | buf[1]);
    data->accelY = (int16_t)((buf[2] << 8) | buf[3]);
    data->accelZ = (int16_t)((buf[4] << 8) | buf[5]);

    /* 跳过温度 buf[6..7] */

    /* 解析陀螺仪（手册 Table 1 p.7: 16-bit 二进制补码） */
    data->gyroX  = (int16_t)((buf[8]  << 8) | buf[9]);
    data->gyroY  = (int16_t)((buf[10] << 8) | buf[11]);
    data->gyroZ  = (int16_t)((buf[12] << 8) | buf[13]);

    return true;
}

/* ================================================================
 *  注册 Device 接口
 * ================================================================ */

DevIMU* GetIMU(void)
{
    s_imuIf.init           = imu_init;
    s_imuIf.whoAmI         = imu_whoAmI;
    s_imuIf.readSensorData = imu_readSensorData;

    return &s_imuIf;
}