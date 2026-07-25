/**
 * @file    port_lis3mdl.c
 * @brief   Port 层 LIS3MDLRT 磁力计 I2C 驱动实现
 * @note    I2C0: PA0=SDA, PA1=SCL, 400kHz
 *          LIS3MDLRT I2C 地址: SD0=1 → 0x1E
 */
 
#include "port_lis3mdl.h"
#include "port_log.h"
#include "ti_msp_dl_config.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

#define REG_WHO_AM_I         0x0F
#define REG_CTRL_REG1        0x20
#define REG_CTRL_REG2        0x21
#define REG_CTRL_REG3        0x22
#define REG_CTRL_REG4        0x23
#define REG_CTRL_REG5        0x24
#define REG_OUT_X_L          0x28

#define I2C_TIMEOUT_MS          50
#define I2C_WAIT_CYCLES         (I2C_TIMEOUT_MS * 10000U)
#define I2C_ERROR_MASK          (DL_I2C_CONTROLLER_STATUS_ERROR | \
                                 DL_I2C_CONTROLLER_STATUS_ARBITRATION_LOST)
#define LIS3MDL_REG_AUTO_INC    0x80

static bool g_lis3mdl_init_ok = false;
static bool g_i2c_diag_done = false;

static void delay_ms(uint32_t ms)
{
    for (volatile uint32_t i = 0; i < ms; i++)
        for (volatile uint32_t j = 0; j < 80000; j++) __NOP();
}

static void i2c_log_status(const char *phase, uint8_t addr, uint32_t status)
{
    LOGE(LOG_MOD_MAG, "%s addr=0x%02X status=0x%08lX%s%s%s\r\n",
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

    LOGD(LOG_MOD_MAG, "Lines: SDA=%s SCL=%s status=0x%08lX\r\n",
            (DL_I2C_getSDAStatus(I2C_0_INST) == DL_I2C_CONTROLLER_SDA_HIGH) ? "HIGH" : "LOW",
            (DL_I2C_getSCLStatus(I2C_0_INST) == DL_I2C_CONTROLLER_SCL_HIGH) ? "HIGH" : "LOW",
            (unsigned long)DL_I2C_getControllerStatus(I2C_0_INST));

    LOGD(LOG_MOD_MAG, "Scan start\r\n");
    for (addr = 0x08; addr <= 0x77; addr++) {
        if (i2c_probe_addr(addr)) {
            found = true;
            LOGD(LOG_MOD_MAG, "ACK addr=0x%02X\r\n", addr);
        }
    }
    if (!found) {
        LOGD(LOG_MOD_MAG, "Scan found no ACK devices\r\n");
    }
}

static bool i2c_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t txBuf[2] = {reg, data};

    i2c_prepare_transfer();
    if (DL_I2C_fillControllerTXFIFO(I2C_0_INST, txBuf, 2) != 2U) {
        LOGE(LOG_MOD_MAG, "TX FIFO fill failed\r\n");
        return false;
    }
    DL_I2C_startControllerTransfer(I2C_0_INST, LIS3MDL_I2C_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    return i2c_wait_done("WRITE", LIS3MDL_I2C_ADDR);
}

static bool i2c_read_reg(uint8_t reg, uint8_t *value)
{
    if (!value) return false;

    i2c_prepare_transfer();
    if (DL_I2C_fillControllerTXFIFO(I2C_0_INST, &reg, 1) != 1U) {
        LOGE(LOG_MOD_MAG, "TX FIFO fill failed\r\n");
        return false;
    }
    DL_I2C_startControllerTransferAdvanced(I2C_0_INST, LIS3MDL_I2C_ADDR,
                                           DL_I2C_CONTROLLER_DIRECTION_TX, 1,
                                           DL_I2C_CONTROLLER_START_ENABLE,
                                           DL_I2C_CONTROLLER_STOP_DISABLE,
                                           DL_I2C_CONTROLLER_ACK_DISABLE);
    if (!i2c_wait_done("REG", LIS3MDL_I2C_ADDR)) return false;

    DL_I2C_startControllerTransfer(I2C_0_INST, LIS3MDL_I2C_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_RX, 1);
    if (!i2c_wait_done("READ", LIS3MDL_I2C_ADDR)) return false;

    if (DL_I2C_getControllerRXFIFOCounter(I2C_0_INST) < 1U) {
        LOGE(LOG_MOD_MAG, "READ RX_EMPTY status=0x%08lX\r\n",
                (unsigned long)DL_I2C_getControllerStatus(I2C_0_INST));
        return false;
    }
    *value = DL_I2C_receiveControllerData(I2C_0_INST);
    return true;
}

static bool i2c_read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint16_t i;
    if (!buf || !len) return false;

    i2c_prepare_transfer();
    if (DL_I2C_fillControllerTXFIFO(I2C_0_INST, &reg, 1) != 1U) {
        LOGE(LOG_MOD_MAG, "TX FIFO fill failed\r\n");
        return false;
    }
    DL_I2C_startControllerTransferAdvanced(I2C_0_INST, LIS3MDL_I2C_ADDR,
                                           DL_I2C_CONTROLLER_DIRECTION_TX, 1,
                                           DL_I2C_CONTROLLER_START_ENABLE,
                                           DL_I2C_CONTROLLER_STOP_DISABLE,
                                           DL_I2C_CONTROLLER_ACK_DISABLE);
    if (!i2c_wait_done("REGS", LIS3MDL_I2C_ADDR)) return false;

    DL_I2C_startControllerTransfer(I2C_0_INST, LIS3MDL_I2C_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_RX, len);
    if (!i2c_wait_done("READS", LIS3MDL_I2C_ADDR)) return false;

    if (DL_I2C_getControllerRXFIFOCounter(I2C_0_INST) < len) {
        LOGE(LOG_MOD_MAG, "READS RX_EMPTY count=%lu need=%u status=0x%08lX\r\n",
                (unsigned long)DL_I2C_getControllerRXFIFOCounter(I2C_0_INST),
                len, (unsigned long)DL_I2C_getControllerStatus(I2C_0_INST));
        return false;
    }
    for (i = 0; i < len; i++)
        buf[i] = DL_I2C_receiveControllerData(I2C_0_INST);
    return true;
}

void PORT_LIS3MDL_Init(void)
{
    uint8_t whoami;
    i2c_bus_diag_once();
    delay_ms(10);
    if (!i2c_read_reg(REG_WHO_AM_I, &whoami)) {
        g_lis3mdl_init_ok = false;
        LOGE(LOG_MOD_MAG, "FAIL: WHO_AM_I read failed!\n");
        return;
    }
    if (whoami != 0x3D) {
        g_lis3mdl_init_ok = false;
        LOGE(LOG_MOD_MAG, "FAIL!\n");
        return;
    }
    if (!i2c_write_reg(REG_CTRL_REG2, 0x04)) { g_lis3mdl_init_ok = false; LOGE(LOG_MOD_MAG, "FAIL: reset write failed!\n"); return; }
    delay_ms(10);
    if (!i2c_write_reg(REG_CTRL_REG1, 0x7C) ||
        !i2c_write_reg(REG_CTRL_REG2, 0x60) ||
        !i2c_write_reg(REG_CTRL_REG3, 0x00) ||
        !i2c_write_reg(REG_CTRL_REG4, 0x0C) ||
        !i2c_write_reg(REG_CTRL_REG5, 0x00)) {
        g_lis3mdl_init_ok = false;
        LOGE(LOG_MOD_MAG, "FAIL: config write failed!\n");
        return;
    }
    g_lis3mdl_init_ok = true;
    LOGI_INIT(LOG_MOD_MAG, "LIS3MDL ready: addr=0x%02X WHO_AM_I=0x%02X\r\n",
            LIS3MDL_I2C_ADDR, whoami);
}

bool PORT_LIS3MDL_IsOk(void) { return g_lis3mdl_init_ok; }

void PORT_LIS3MDL_ReadMagRaw(int16_t *mx, int16_t *my, int16_t *mz)
{
    uint8_t buf[6];
    if (!g_lis3mdl_init_ok) { *mx = *my = *mz = 0; return; }
    if (!i2c_read_regs(REG_OUT_X_L | LIS3MDL_REG_AUTO_INC, buf, 6)) { *mx = *my = *mz = 0; return; }
    *mx = (int16_t)((buf[1] << 8) | buf[0]);
    *my = (int16_t)((buf[3] << 8) | buf[2]);
    *mz = (int16_t)((buf[5] << 8) | buf[4]);
}
