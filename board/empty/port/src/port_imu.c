/**
 * @file    port_imu.c
 * @brief   Port 层 ICM-20608 硬件 I2C 驱动实现
 * @note    I2C0: PA0=SDA, PA1=SCL, 400kHz
 *          ICM-20608 I2C 地址: AD0=0 → 0x68
 */

#include "port_imu.h"
#include "port_log.h"
#include "ti_msp_dl_config.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

#define REG_WHO_AM_I         0x75
#define ICM_WHO_AM_I_EXPECTED 0xAF
#define REG_USER_CTRL        0x03
#define REG_PWR_MGMT_1       0x6B
#define REG_PWR_MGMT_2       0x6C
#define REG_SMPLRT_DIV       0x19
#define REG_CONFIG           0x1A
#define REG_GYRO_CONFIG      0x1B
#define REG_ACCEL_CONFIG     0x1C
#define REG_ACCEL_CONFIG2    0x1D
#define REG_ACCEL_XOUT_H     0x3B
#define REG_GYRO_XOUT_H      0x43

#define PWR_MGMT_1_DEVICE_RESET  (1 << 7)
#define PWR_MGMT_1_SLEEP         (1 << 6)
#define PWR_MGMT_1_CLKSEL_AUTO   (0x01)
#define ACCEL_FS_16G             0x18
#define GYRO_FS_2000DPS          0x18

#define I2C_TIMEOUT_MS          50
#define I2C_WAIT_CYCLES         (I2C_TIMEOUT_MS * 10000U)
#define I2C_ERROR_MASK          (DL_I2C_CONTROLLER_STATUS_ERROR | \
                                 DL_I2C_CONTROLLER_STATUS_ARBITRATION_LOST)
#define ICM_I2C_ADDR_ALT        0x69
#define ICM_PROBE_RETRY_COUNT   8
#define ICM_PROBE_RETRY_DELAY_MS 150
#define ICM_RESET_DELAY_MS      500
#define ICM_WAKE_DELAY_MS       100
#define ICM_WHOAMI_RETRY_COUNT  5
#define ICM_WHOAMI_RETRY_DELAY_MS 50

static bool g_imu_init_ok = false;
static bool g_i2c_diag_done = false;
static uint8_t g_icm_i2c_addr = ICM_I2C_ADDR;

static void delay_ms(uint32_t ms)
{
    for (volatile uint32_t i = 0; i < ms; i++)
        for (volatile uint32_t j = 0; j < 80000; j++) __NOP();
}

static void i2c_log_status(const char *phase, uint8_t addr, uint32_t status)
{
    LOG_RAW("[I2C] %s addr=0x%02X status=0x%08lX%s%s%s\r\n",
            phase, addr, (unsigned long)status,
            (status & DL_I2C_CONTROLLER_STATUS_ERROR) ? " ERROR" : "",
            (status & DL_I2C_CONTROLLER_STATUS_ARBITRATION_LOST) ? " ARB_LOST" : "",
            (status & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) ? " BUS_BUSY" : "");
}

static void i2c_prepare_transfer(void)
{
    DL_I2C_resetControllerTransfer(I2C_0_INST);
    DL_I2C_flushControllerRXFIFO(I2C_0_INST);
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
}

static bool i2c_wait_done_ex(const char *phase, uint8_t addr, bool quiet)
{
    uint32_t status;
    volatile uint32_t timeout = I2C_WAIT_CYCLES;

    do {
        status = DL_I2C_getControllerStatus(I2C_0_INST);
        if (status & I2C_ERROR_MASK) {
            if (!quiet) i2c_log_status(phase, addr, status);
            return false;
        }
        if ((DL_I2C_getTransactionCount(I2C_0_INST) == 0U) &&
            ((status & DL_I2C_CONTROLLER_STATUS_BUSY) == 0U)) {
            status = DL_I2C_getControllerStatus(I2C_0_INST);
            if (status & I2C_ERROR_MASK) {
                if (!quiet) i2c_log_status(phase, addr, status);
                return false;
            }
            return true;
        }
    } while (--timeout);

    if (!quiet) i2c_log_status("TIMEOUT", addr, status);
    return false;
}

static bool i2c_wait_done(const char *phase, uint8_t addr)
{
    return i2c_wait_done_ex(phase, addr, false);
}

static bool i2c_wait_rx_count(uint16_t needed, const char *phase)
{
    uint32_t status;
    volatile uint32_t timeout = I2C_WAIT_CYCLES;

    do {
        if (DL_I2C_getControllerRXFIFOCounter(I2C_0_INST) >= needed) {
            return true;
        }

        status = DL_I2C_getControllerStatus(I2C_0_INST);
        if (status & I2C_ERROR_MASK) {
            i2c_log_status(phase, g_icm_i2c_addr, status);
            return false;
        }
    } while (--timeout);

    LOG_RAW("[ICM-20608] %s RX_EMPTY count=%lu need=%u status=0x%08lX\r\n",
            phase,
            (unsigned long)DL_I2C_getControllerRXFIFOCounter(I2C_0_INST),
            needed,
            (unsigned long)DL_I2C_getControllerStatus(I2C_0_INST));
    return false;
}

static bool i2c_probe_addr(uint8_t addr)
{
    uint8_t probe = 0x00;

    i2c_prepare_transfer();
    if (DL_I2C_fillControllerTXFIFO(I2C_0_INST, &probe, 1) != 1U) {
        return false;
    }
    DL_I2C_startControllerTransfer(I2C_0_INST, addr,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    return i2c_wait_done_ex("PROBE", addr, true);
}

static void i2c_bus_diag_once(void)
{
    uint8_t addr;
    bool found = false;

    if (g_i2c_diag_done) return;
    g_i2c_diag_done = true;

    LOG_RAW("[I2C] Lines: SDA=%s SCL=%s status=0x%08lX\r\n",
            (DL_I2C_getSDAStatus(I2C_0_INST) == DL_I2C_CONTROLLER_SDA_HIGH) ? "HIGH" : "LOW",
            (DL_I2C_getSCLStatus(I2C_0_INST) == DL_I2C_CONTROLLER_SCL_HIGH) ? "HIGH" : "LOW",
            (unsigned long)DL_I2C_getControllerStatus(I2C_0_INST));

    LOG_RAW("[I2C] Scan start\r\n");
    for (addr = 0x08; addr <= 0x77; addr++) {
        if (i2c_probe_addr(addr)) {
            found = true;
            LOG_RAW("[I2C] ACK addr=0x%02X\r\n", addr);
        }
    }
    if (!found) {
        LOG_RAW("[I2C] Scan found no ACK devices\r\n");
    }
}

static bool icm_select_i2c_addr(void)
{
    for (uint8_t attempt = 1; attempt <= ICM_PROBE_RETRY_COUNT; attempt++) {
        bool ack68 = i2c_probe_addr(ICM_I2C_ADDR);
        bool ack69 = i2c_probe_addr(ICM_I2C_ADDR_ALT);

        LOG_RAW("[ICM-20608] Probe %u/%u addr=0x%02X: %s\r\n",
                attempt, ICM_PROBE_RETRY_COUNT, ICM_I2C_ADDR, ack68 ? "ACK" : "NACK");
        LOG_RAW("[ICM-20608] Probe %u/%u addr=0x%02X: %s\r\n",
                attempt, ICM_PROBE_RETRY_COUNT, ICM_I2C_ADDR_ALT, ack69 ? "ACK" : "NACK");

        if (ack69) {
            g_icm_i2c_addr = ICM_I2C_ADDR_ALT;
            LOG_RAW("[ICM-20608] Using ICM addr=0x%02X\r\n", g_icm_i2c_addr);
            return true;
        }
        if (ack68) {
            g_icm_i2c_addr = ICM_I2C_ADDR;
            LOG_RAW("[ICM-20608] Using ICM addr=0x%02X\r\n", g_icm_i2c_addr);
            return true;
        }

        i2c_prepare_transfer();
        delay_ms(ICM_PROBE_RETRY_DELAY_MS);
    }

    LOG_RAW("[ICM-20608] FAIL: no ICM address ACK after retries\r\n");
    return false;
}

static bool i2c_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t txBuf[2] = {reg, data};

    i2c_prepare_transfer();
    if (DL_I2C_fillControllerTXFIFO(I2C_0_INST, txBuf, 2) != 2U) {
        LOG_RAW("[ICM-20608] TX FIFO fill failed\r\n");
        return false;
    }
    DL_I2C_startControllerTransfer(I2C_0_INST, g_icm_i2c_addr,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    return i2c_wait_done("WRITE", g_icm_i2c_addr);
}

static bool i2c_read_reg(uint8_t reg, uint8_t *value)
{
    if (!value) return false;

    i2c_prepare_transfer();
    if (DL_I2C_fillControllerTXFIFO(I2C_0_INST, &reg, 1) != 1U) {
        LOG_RAW("[ICM-20608] TX FIFO fill failed\r\n");
        return false;
    }
    DL_I2C_startControllerTransferAdvanced(I2C_0_INST, g_icm_i2c_addr,
                                           DL_I2C_CONTROLLER_DIRECTION_TX, 1,
                                           DL_I2C_CONTROLLER_START_ENABLE,
                                           DL_I2C_CONTROLLER_STOP_DISABLE,
                                           DL_I2C_CONTROLLER_ACK_DISABLE);
    if (!i2c_wait_done("REG", g_icm_i2c_addr)) return false;

    DL_I2C_startControllerTransfer(I2C_0_INST, g_icm_i2c_addr,
                                   DL_I2C_CONTROLLER_DIRECTION_RX, 1);
    if (!i2c_wait_done("READ", g_icm_i2c_addr)) return false;

    if (!i2c_wait_rx_count(1, "READ")) return false;
    *value = DL_I2C_receiveControllerData(I2C_0_INST);
    return true;
}

static bool i2c_read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint16_t i;
    if (!buf || !len) return false;

    i2c_prepare_transfer();
    if (DL_I2C_fillControllerTXFIFO(I2C_0_INST, &reg, 1) != 1U) {
        LOG_RAW("[ICM-20608] TX FIFO fill failed\r\n");
        return false;
    }
    DL_I2C_startControllerTransferAdvanced(I2C_0_INST, g_icm_i2c_addr,
                                           DL_I2C_CONTROLLER_DIRECTION_TX, 1,
                                           DL_I2C_CONTROLLER_START_ENABLE,
                                           DL_I2C_CONTROLLER_STOP_DISABLE,
                                           DL_I2C_CONTROLLER_ACK_DISABLE);
    if (!i2c_wait_done("REGS", g_icm_i2c_addr)) return false;

    DL_I2C_startControllerTransfer(I2C_0_INST, g_icm_i2c_addr,
                                   DL_I2C_CONTROLLER_DIRECTION_RX, len);
    if (!i2c_wait_done("READS", g_icm_i2c_addr)) return false;

    if (!i2c_wait_rx_count(len, "READS")) return false;
    for (i = 0; i < len; i++)
        buf[i] = DL_I2C_receiveControllerData(I2C_0_INST);
    return true;
}

void PORT_IMU_Init(void)
{
    uint8_t whoami;
    bool whoamiRead = false;

    LOG_RAW("[ICM-20608] Init start (auto I2C addr)\r\n");
    i2c_bus_diag_once();
    if (!icm_select_i2c_addr()) {
        g_imu_init_ok = false;
        return;
    }
    delay_ms(100);
    LOG_RAW("[ICM-20608] Device reset...\r\n");
    if (!i2c_write_reg(REG_PWR_MGMT_1, PWR_MGMT_1_DEVICE_RESET)) {
        g_imu_init_ok = false;
        LOG_RAW("[ICM-20608] FAIL: reset write failed!\r\n");
        return;
    }
    delay_ms(ICM_RESET_DELAY_MS);
    i2c_prepare_transfer();

    if (!icm_select_i2c_addr()) {
        g_imu_init_ok = false;
        LOG_RAW("[ICM-20608] FAIL: no ICM ACK after reset!\r\n");
        return;
    }

    if (!i2c_write_reg(REG_PWR_MGMT_1, PWR_MGMT_1_CLKSEL_AUTO)) {
        g_imu_init_ok = false;
        LOG_RAW("[ICM-20608] FAIL: wake write failed!\r\n");
        return;
    }
    delay_ms(ICM_WAKE_DELAY_MS);
    i2c_prepare_transfer();

    for (uint8_t attempt = 1; attempt <= ICM_WHOAMI_RETRY_COUNT; attempt++) {
        if (i2c_read_reg(REG_WHO_AM_I, &whoami)) {
            whoamiRead = true;
            break;
        }

        LOG_RAW("[ICM-20608] WHO_AM_I retry %u/%u\r\n",
                attempt, ICM_WHOAMI_RETRY_COUNT);
        delay_ms(ICM_WHOAMI_RETRY_DELAY_MS);
        i2c_prepare_transfer();
    }

    if (!whoamiRead) {
        g_imu_init_ok = false;
        LOG_RAW("[ICM-20608] FAIL: WHO_AM_I read failed!\r\n");
        return;
    }
    LOG_RAW("[ICM-20608] WHO_AM_I = 0x%02X (expected 0x%02X)\r\n",
            whoami, ICM_WHO_AM_I_EXPECTED);
    if (whoami == ICM_WHO_AM_I_EXPECTED) {
        g_imu_init_ok = true;
        LOG_RAW("[ICM-20608] I2C OK\r\n");
    } else {
        g_imu_init_ok = false;
        LOG_RAW("[ICM-20608] FAIL: WHO_AM_I mismatch!\r\n");
        return;
    }
    if (!i2c_write_reg(REG_ACCEL_CONFIG, ACCEL_FS_16G) ||
        !i2c_write_reg(REG_ACCEL_CONFIG2, 0x03) ||
        !i2c_write_reg(REG_GYRO_CONFIG, GYRO_FS_2000DPS) ||
        !i2c_write_reg(REG_SMPLRT_DIV, 4) ||
        !i2c_write_reg(REG_CONFIG, 3) ||
        !i2c_write_reg(REG_PWR_MGMT_2, 0x00)) {
        g_imu_init_ok = false;
        LOG_RAW("[ICM-20608] FAIL: config write failed!\r\n");
        return;
    }
    LOG_RAW("[ICM-20608] Init completed (I2C)\r\n");
}

bool PORT_IMU_IsOk(void) { return g_imu_init_ok; }

void PORT_IMU_ReadAccelRaw(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6];
    if (!g_imu_init_ok) { *ax = *ay = *az = 0; return; }
    if (!i2c_read_regs(REG_ACCEL_XOUT_H, buf, 6)) { *ax = *ay = *az = 0; return; }
    *ax = (int16_t)((buf[0] << 8) | buf[1]);
    *ay = (int16_t)((buf[2] << 8) | buf[3]);
    *az = (int16_t)((buf[4] << 8) | buf[5]);
}

void PORT_IMU_ReadGyroRaw(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6];
    if (!g_imu_init_ok) { *gx = *gy = *gz = 0; return; }
    if (!i2c_read_regs(REG_GYRO_XOUT_H, buf, 6)) { *gx = *gy = *gz = 0; return; }
    *gx = (int16_t)((buf[0] << 8) | buf[1]);
    *gy = (int16_t)((buf[2] << 8) | buf[3]);
    *gz = (int16_t)((buf[4] << 8) | buf[5]);
}

uint8_t PORT_IMU_ReadReg(uint8_t reg)
{
    uint8_t value = 0;
    (void)i2c_read_reg(reg, &value);
    return value;
}

void PORT_IMU_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    (void)i2c_read_regs(reg, buf, len);
}
