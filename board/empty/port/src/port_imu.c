/**
 * @file    port_imu.c
 * @brief   Port 层 ICM-20948 软件 SPI 驱动实现
 * @note    实现 dev_imu.h 的 DevIMU OOP 契约
 *          严格按手册 6.5 节及 Fig 2 SPI 时序图：
 *          - CPOL=0, CPHA=0 (空闲SCK=0, 上升沿锁存)
 *          - tSU;CS ≥ 8ns, tHD;CS ≥ 500ns
 *          - R/W bit7 + 7bit 地址, MSB 先
 *          使用 GPIO bit-bang，纯轮询，不依赖硬件 SPI
 *
 *          引脚:
 *          PB9  - SCK
 *          PB8  - MOSI
 *          PB7  - MISO
 *          PB6  - CS
 */

#include "port_imu.h"
#include "port_log.h"
#include "ti_msp_dl_config.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

/* ================================================================
 *  引脚宏定义
 * ================================================================ */
#define IMU_PORT            GPIOB
#define IMU_SCK_PIN         DL_GPIO_PIN_9
#define IMU_MOSI_PIN        DL_GPIO_PIN_8
#define IMU_MISO_PIN        DL_GPIO_PIN_7
#define IMU_CS_PIN          DL_GPIO_PIN_6

/* ================================================================
 *  ICM-20948 寄存器定义
 * ================================================================ */
/* --- Bank 0 寄存器 --- */
#define REG_BANK_SEL        0x7F    /* 任意 bank 可访问；bank 号在 bit[5:4] */
#define REG_WHO_AM_I        0x00    /* Bank 0，读回 0xEA */
#define REG_USER_CTRL       0x03    /* Bank 0 */
#define REG_PWR_MGMT_1      0x06    /* Bank 0 */
#define REG_ACCEL_XOUT_H    0x2D    /* Bank 0（数据寄存器，非 Bank 2！） */
#define REG_GYRO_XOUT_H     0x33    /* Bank 0 */

/* --- Bank 2 寄存器（量程配置） --- */
#define REG_GYRO_SMPLRT_DIV 0x00    /* Bank 2，陀螺 ODR 分频 */
#define REG_ACCEL_CONFIG    0x14    /* Bank 2 */
#define REG_ACCEL_SMPLRT_DIV_1 0x10 /* Bank 2，加计 ODR 分频高字节 */
#define REG_ACCEL_SMPLRT_DIV_2 0x11 /* Bank 2，加计 ODR 分频低字节 */
#define REG_GYRO_CONFIG_1   0x01    /* Bank 2 */

/* USER_CTRL(0x03) 位：I2C_IF_DIS 是 bit4(0x10)，禁用 I2C 从机接口强制 SPI。
 * 注意 bit5 是 I2C_MST_EN（I2C 主模式），不要混用。 */
#define USER_CTRL_I2C_IF_DIS   (1 << 4)

/* PWR_MGMT_1(0x06) 位 */
#define PWR_MGMT_1_DEVICE_RESET (1 << 7)   /* 0x80 软复位，复位后自清 */
#define PWR_MGMT_1_CLKSEL_AUTO  (0x01)     /* 清 SLEEP/LP，CLKSEL=1 自动选时钟 */

/* Device 层接口实例 */
static DevIMU s_imuIf;

/* ================================================================
 *  GPIO 操作宏
 * ================================================================ */
#define IMU_SCK_LOW()    DL_GPIO_clearPins(IMU_PORT, IMU_SCK_PIN)
#define IMU_SCK_HIGH()   DL_GPIO_setPins(IMU_PORT, IMU_SCK_PIN)
#define IMU_MOSI_LOW()   DL_GPIO_clearPins(IMU_PORT, IMU_MOSI_PIN)
#define IMU_MOSI_HIGH()  DL_GPIO_setPins(IMU_PORT, IMU_MOSI_PIN)
#define IMU_CS_LOW()     DL_GPIO_clearPins(IMU_PORT, IMU_CS_PIN)
#define IMU_CS_HIGH()    DL_GPIO_setPins(IMU_PORT, IMU_CS_PIN)
#define IMU_MISO_READ()  ((DL_GPIO_readPins(IMU_PORT, IMU_MISO_PIN) & IMU_MISO_PIN) ? 1 : 0)

/* ~1us 延时 @ 80MHz (80 NOP) */
static inline void delay_1us(void)
{
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
}

/* ~10ms 延时 @ 80MHz */
static void delay_10ms(void)
{
    volatile uint32_t i;
    for (i = 0; i < 10000; i++) {
        delay_1us();
    }
}

/* ~100ms 延时 */
static void delay_100ms(void)
{
    volatile uint32_t i;
    for (i = 0; i < 10; i++) {
        delay_10ms();
    }
}

/* SCK 半周期延时：控制软件 SPI 频率，并给 LSF0108 的 RC 边沿留建立时间 */
static inline void spi_delay_half(void)
{
    for (uint32_t i = 0; i < IMU_SCK_HALF_PERIOD_US; i++) {
        delay_1us();
    }
}

/* ================================================================
 *  软件 SPI 字节收发（MSB 先，上升沿锁存）
 *
 *  手册 Fig 2 时序要求:
 *  tSU;CS   ≥ 8ns    (CS 建立时间)
 *  tHD;CS   ≥ 500ns  (CS 保持时间)
 *  tSU;SDI  ≥ 5ns    (SDI 建立时间)
 *  tHD;SDI  ≥ 7ns    (SDI 保持时间)
 *  tVD;SDO  ≤ 59ns   (SDO 输出有效时间)
 *
 *  一个 bit 周期: ~2us @ 80MHz, 约 500kHz SCLK
 * ================================================================ */

static uint8_t spi_transfer_byte(uint8_t txByte)
{
    uint8_t rxByte = 0;

    for (int i = 7; i >= 0; i--)
    {
        /* SCK=0 低相位：设置 MOSI（含 tSU;SDI 建立时间） */
        IMU_SCK_LOW();
        if (txByte & (1 << i)) {
            IMU_MOSI_HIGH();
        } else {
            IMU_MOSI_LOW();
        }
        spi_delay_half();

        /* SCK↑ 上升沿：ICM-20948 内部锁存 MOSI，同时输出本位 MISO。
         * 整个高相位用于让 LSF0108 的 RC 边沿稳定，相位末尾再采样 MISO。 */
        IMU_SCK_HIGH();
        spi_delay_half();

        /* 高相位末尾采样 MISO（边沿已稳定，留足 LSF0108 充电时间） */
        if (IMU_MISO_READ()) {
            rxByte |= (1 << i);
        }
    }

    /* 传输完成，SCK 回到低 */
    IMU_SCK_LOW();

    return rxByte;
}

/* ================================================================
 *  公有 API 实现
 * ================================================================ */

void PORT_IMU_InitGPIO(void)
{
    /* SCK (PB9) - 输出，默认低电平 */
    DL_GPIO_initDigitalOutput(IOMUX_PINCM26);
    IMU_SCK_LOW();
    DL_GPIO_enableOutput(IMU_PORT, IMU_SCK_PIN);

    /* MOSI (PB8) - 输出，默认低电平 */
    DL_GPIO_initDigitalOutput(IOMUX_PINCM25);
    IMU_MOSI_LOW();
    DL_GPIO_enableOutput(IMU_PORT, IMU_MOSI_PIN);

    /* CS (PB6) - 输出，默认高电平 */
    DL_GPIO_initDigitalOutput(IOMUX_PINCM23);
    IMU_CS_HIGH();
    DL_GPIO_enableOutput(IMU_PORT, IMU_CS_PIN);

    /* MISO (PB7) - 输入 */
    DL_GPIO_initDigitalInput(IOMUX_PINCM24);
}

void PORT_IMU_WriteReg(uint8_t regAddr, uint8_t data)
{
    uint8_t addrByte = regAddr & 0x7F;

    IMU_CS_LOW();               /* tSU;CS 开始 */
    delay_1us();                /* tSU;CS ≥ 8ns */

    spi_transfer_byte(addrByte);
    spi_transfer_byte(data);

    delay_1us();                /* tHD;CS ≥ 500ns */
    IMU_CS_HIGH();
}

uint8_t PORT_IMU_ReadReg(uint8_t regAddr)
{
    uint8_t addrByte = 0x80 | (regAddr & 0x7F);
    uint8_t data;

    IMU_CS_LOW();
    delay_1us();

    spi_transfer_byte(addrByte);
    data = spi_transfer_byte(0xFF);

    delay_1us();
    IMU_CS_HIGH();

    return data;
}

void PORT_IMU_ReadRegs(uint8_t regAddr, uint8_t *buf, uint16_t len)
{
    uint8_t addrByte = 0x80 | (regAddr & 0x7F);

    if (buf == NULL || len == 0) return;

    IMU_CS_LOW();
    delay_1us();

    spi_transfer_byte(addrByte);

    for (uint16_t i = 0; i < len; i++) {
        buf[i] = spi_transfer_byte(0xFF);
    }

    delay_1us();
    IMU_CS_HIGH();
}

void PORT_IMU_SetBank(uint8_t bank)
{
    /* bank 号位于 REG_BANK_SEL 的 bit[5:4]，必须左移 4 位 */
    PORT_IMU_WriteReg(REG_BANK_SEL, (uint8_t)((bank & 0x03) << 4));
}

/* ================================================================
 *  DevIMU 接口函数实现
 * ================================================================ */

/* 写寄存器并回读校验（仅用于初始化，热路径不调用）。
 * 调用前需保证当前处于该寄存器所在 bank。 */
static bool imu_write_verify(uint8_t reg, uint8_t val)
{
    uint8_t rb;

    PORT_IMU_WriteReg(reg, val);
    rb = PORT_IMU_ReadReg(reg);
    if (rb != val) {
        LOG_ERROR("[IMU] reg 0x%02X verify fail: wrote 0x%02X read 0x%02X\r\n",
                  reg, val, rb);
        return false;
    }
    return true;
}

/* ICM-20948 软复位：Bank0 给 PWR_MGMT_1 置 DEVICE_RESET 位，等待复位完成（位自清） */
static void imu_device_reset(void)
{
    PORT_IMU_SetBank(0);
    PORT_IMU_WriteReg(REG_PWR_MGMT_1, PWR_MGMT_1_DEVICE_RESET);
    delay_100ms();   /* 复位需要时间，DEVICE_RESET 位完成后自清 */
}

static void imu_init(DevIMU *self)
{
    (void)self;

    /* 初始化 GPIO（CS 默认高、SCK 默认低） */
    PORT_IMU_InitGPIO();

    /* 上电后等 100ms */
    delay_100ms();

    /* 软复位，进入已知初态 */
    imu_device_reset();

    /* 复位后位于 Bank 0 */
    PORT_IMU_SetBank(0);

    /* 退出睡眠、CLKSEL=1 自动选时钟（写后回读校验） */
    if (!imu_write_verify(REG_PWR_MGMT_1, PWR_MGMT_1_CLKSEL_AUTO)) {
        LOG_ERROR("[IMU] PWR_MGMT_1 config failed\r\n");
    }
    delay_10ms();

    /* 禁用 I2C 从机接口，强制 SPI（写后回读校验） */
    if (!imu_write_verify(REG_USER_CTRL, USER_CTRL_I2C_IF_DIS)) {
        LOG_ERROR("[IMU] USER_CTRL(I2C_IF_DIS) config failed\r\n");
    }

    /* 切到 Bank 2 配置量程 */
    PORT_IMU_SetBank(2);
    imu_write_verify(REG_ACCEL_CONFIG, 0x00);    /* ±2g */
    imu_write_verify(REG_GYRO_CONFIG_1, 0x00);   /* ±250dps */

    /* 回到 Bank 0（数据寄存器所在 bank） */
    PORT_IMU_SetBank(0);
}

static uint8_t imu_whoAmI(DevIMU *self)
{
    (void)self;
    uint8_t id = 0;

    /* 少量重试，避免上电瞬态/边沿未稳导致的误判 */
    for (int i = 0; i < 5; i++) {
        PORT_IMU_SetBank(0);
        id = PORT_IMU_ReadReg(REG_WHO_AM_I);
        if (id == 0xEA) {
            return id;
        }
        delay_10ms();
    }
    return id;
}

static bool imu_readSensorData(DevIMU *self, DevIMU_SensorData_t *data)
{
    uint8_t buf[12];

    if (data == NULL) return false;

    /* 加速度/陀螺数据寄存器在 Bank 0（非 Bank 2） */
    PORT_IMU_SetBank(0);
    PORT_IMU_ReadRegs(REG_ACCEL_XOUT_H, buf, 12);

    data->accelX = (int16_t)((buf[0] << 8) | buf[1]);
    data->accelY = (int16_t)((buf[2] << 8) | buf[3]);
    data->accelZ = (int16_t)((buf[4] << 8) | buf[5]);
    data->gyroX  = (int16_t)((buf[6] << 8) | buf[7]);
    data->gyroY  = (int16_t)((buf[8] << 8) | buf[9]);
    data->gyroZ  = (int16_t)((buf[10] << 8) | buf[11]);

    return true;
}

DevIMU* GetIMU(void)
{
    s_imuIf.init           = imu_init;
    s_imuIf.whoAmI         = imu_whoAmI;
    s_imuIf.readSensorData = imu_readSensorData;

    return &s_imuIf;
}