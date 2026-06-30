/**
 * @file    port_tb6612.c
 * @brief   Port 层 TB6612 电机驱动实现
 * @note    四层解耦架构 - Port 层
 *          实现 dev_tb6612.h 的 DevTB6612 OOP 契约
 *          PWM:  TIMG0 (DC_MOTOR), 40MHz, period=4000 → 10kHz
 *                 PA12=CCP0 (PWMA), PA13=CCP1 (PWMB)
 *          GPIO: PA22(AIN1), PB21(AIN2), PB23(BIN1), PA23(BIN2)
 *          所有 GPIO 已在 SYSCFG_DL_GPIO_init() 中配置为 DigitalOutput + 低电平
 *
 *          TB6612 双通道同步控制（A/B 同速同向）
 *          方向真值表 (以 A 通道为例，B 通道同理):
 *            AIN1=H AIN2=L  →  正转 (Forward)
 *            AIN1=L AIN2=H  →  反转 (Reverse)
 *            AIN1=L AIN2=L  →  短接刹车 (Brake)
 *            AIN1=H AIN2=H  →  惰行/停止 (Coast)
 */

#include "port_tb6612.h"
#include "ti_msp_dl_config.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

/* PWM 周期 (来自 gDC_MOTORConfig.period = 4000 @ 40MHz → 10kHz) */
#define PWM_PERIOD              4000U

/* 将 0-100 百分比换算为 CCP 比较值 */
#define PCT_TO_CCP(pct)         ((uint16_t)(((uint32_t)(pct) * PWM_PERIOD) / 100U))

/* Device 层接口单例 */
static DevTB6612 s_tb6612If;

/* 运行期状态 */
static uint8_t   s_speed   = 0;       /* 0-100 占空比百分比 */
static TB6612_Dir s_dir    = TB6612_DIR_COAST;

/* ========== 静态函数：方向 GPIO 驱动 ========== */

static void dir_set_forward(void)
{
    /* A 通道 */
    DL_GPIO_setPins(TB6612_AIN1_PORT, TB6612_AIN1_PIN);
    DL_GPIO_clearPins(TB6612_AIN2_PORT, TB6612_AIN2_PIN);
    /* B 通道 */
    DL_GPIO_setPins(TB6612_BIN1_PORT, TB6612_BIN1_PIN);
    DL_GPIO_clearPins(TB6612_BIN2_PORT, TB6612_BIN2_PIN);
}

static void dir_set_reverse(void)
{
    DL_GPIO_clearPins(TB6612_AIN1_PORT, TB6612_AIN1_PIN);
    DL_GPIO_setPins(TB6612_AIN2_PORT, TB6612_AIN2_PIN);
    DL_GPIO_clearPins(TB6612_BIN1_PORT, TB6612_BIN1_PIN);
    DL_GPIO_setPins(TB6612_BIN2_PORT, TB6612_BIN2_PIN);
}

static void dir_set_brake(void)
{
    DL_GPIO_clearPins(TB6612_AIN1_PORT, TB6612_AIN1_PIN);
    DL_GPIO_clearPins(TB6612_AIN2_PORT, TB6612_AIN2_PIN);
    DL_GPIO_clearPins(TB6612_BIN1_PORT, TB6612_BIN1_PIN);
    DL_GPIO_clearPins(TB6612_BIN2_PORT, TB6612_BIN2_PIN);
}

static void dir_set_coast(void)
{
    DL_GPIO_setPins(TB6612_AIN1_PORT, TB6612_AIN1_PIN);
    DL_GPIO_setPins(TB6612_AIN2_PORT, TB6612_AIN2_PIN);
    DL_GPIO_setPins(TB6612_BIN1_PORT, TB6612_BIN1_PIN);
    DL_GPIO_setPins(TB6612_BIN2_PORT, TB6612_BIN2_PIN);
}

static void dir_apply(TB6612_Dir dir)
{
    switch (dir) {
        case TB6612_DIR_FORWARD:  dir_set_forward();  break;
        case TB6612_DIR_REVERSE:  dir_set_reverse();  break;
        case TB6612_DIR_BRAKE:    dir_set_brake();    break;
        case TB6612_DIR_COAST:    dir_set_coast();    break;
        default:                  dir_set_coast();    break;
    }
}

/* ========== 静态函数：PWM 驱动 ========== */

static void pwm_set_duty(uint8_t pct)
{
    uint16_t ccpVal;

    if (pct > 100U) pct = 100U;
    ccpVal = PCT_TO_CCP(pct);

    /* A/B 两路同占空比 */
    DL_TimerG_setCaptureCompareValue(DC_MOTOR_INST, ccpVal, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(DC_MOTOR_INST, ccpVal, DL_TIMER_CC_1_INDEX);
}

/* ========== OOP 接口实现 ========== */

static void tb6612_init(DevTB6612 *self)
{
    (void)self;

    /* GPIO 方向引脚已在 SYSCFG_DL_GPIO_init() 中初始化完毕；
     * TIMG0 已在 SYSCFG_DL_DC_MOTOR_init() 中配置 PWM 并启动。
     * 这里只需确保初始状态：惰行、占空比 0。 */
    dir_set_coast();
    pwm_set_duty(0);
    s_speed = 0;
    s_dir   = TB6612_DIR_COAST;
}

static void tb6612_enable(DevTB6612 *self)
{
    (void)self;
    /* TIMG0 在 HAL 初始化时已启动(.startTimer = DL_TIMER_START)，
     * 这里恢复方向与占空比即可让电机按已设参数运转。 */
    dir_apply(s_dir);
    pwm_set_duty(s_speed);
}

static void tb6612_disable(DevTB6612 *self)
{
    (void)self;
    /* 占空比归零 + 全部方向引脚拉低 → 刹车停止 */
    pwm_set_duty(0);
    dir_set_brake();
}

static void tb6612_setSpeed(DevTB6612 *self, uint8_t pct)
{
    (void)self;

    if (pct > 100U) pct = 100U;
    s_speed = pct;
    pwm_set_duty(s_speed);
}

static void tb6612_setDirection(DevTB6612 *self, TB6612_Dir dir)
{
    (void)self;

    s_dir = dir;
    dir_apply(s_dir);
}

static uint8_t tb6612_getSpeed(DevTB6612 *self)
{
    (void)self;
    return s_speed;
}

static TB6612_Dir tb6612_getDirection(DevTB6612 *self)
{
    (void)self;
    return s_dir;
}

/* ========== 单通道控制（公开函数，供 App 层调用） ========== */

/*
 * Left wheel only: A channel forward, B channel coast.
 * AIN1=H AIN2=L (fwd), BIN1=H BIN2=H (coast)
 */
void PORT_TB6612_LeftOnly(void)
{
    /* A channel forward */
    DL_GPIO_setPins(TB6612_AIN1_PORT, TB6612_AIN1_PIN);
    DL_GPIO_clearPins(TB6612_AIN2_PORT, TB6612_AIN2_PIN);
    /* B channel coast */
    DL_GPIO_setPins(TB6612_BIN1_PORT, TB6612_BIN1_PIN);
    DL_GPIO_setPins(TB6612_BIN2_PORT, TB6612_BIN2_PIN);
}

/*
 * Right wheel only: B channel forward, A channel coast.
 * BIN1=H BIN2=L (fwd), AIN1=H AIN2=H (coast)
 */
void PORT_TB6612_RightOnly(void)
{
    /* B channel forward */
    DL_GPIO_setPins(TB6612_BIN1_PORT, TB6612_BIN1_PIN);
    DL_GPIO_clearPins(TB6612_BIN2_PORT, TB6612_BIN2_PIN);
    /* A channel coast */
    DL_GPIO_setPins(TB6612_AIN1_PORT, TB6612_AIN1_PIN);
    DL_GPIO_setPins(TB6612_AIN2_PORT, TB6612_AIN2_PIN);
}

/*
 * Set left channel (CCP0 / PA12 / PWMA) duty only, leave right unchanged.
 */
void PORT_TB6612_SetLeftDuty(uint8_t pct)
{
    uint16_t ccpVal;
    if (pct > 100U) pct = 100U;
    ccpVal = (uint16_t)(((uint32_t)pct * PWM_PERIOD) / 100U);
    DL_TimerG_setCaptureCompareValue(DC_MOTOR_INST, ccpVal, DL_TIMER_CC_0_INDEX);
}

/*
 * Set right channel (CCP1 / PA13 / PWMB) duty only, leave left unchanged.
 */
void PORT_TB6612_SetRightDuty(uint8_t pct)
{
    uint16_t ccpVal;
    if (pct > 100U) pct = 100U;
    ccpVal = (uint16_t)(((uint32_t)pct * PWM_PERIOD) / 100U);
    DL_TimerG_setCaptureCompareValue(DC_MOTOR_INST, ccpVal, DL_TIMER_CC_1_INDEX);
}

/* ========== 工厂函数 ========== */

DevTB6612* GetTB6612(void)
{
    s_tb6612If.init         = tb6612_init;
    s_tb6612If.enable       = tb6612_enable;
    s_tb6612If.disable      = tb6612_disable;
    s_tb6612If.setSpeed     = tb6612_setSpeed;
    s_tb6612If.setDirection = tb6612_setDirection;
    s_tb6612If.getSpeed     = tb6612_getSpeed;
    s_tb6612If.getDirection = tb6612_getDirection;

    return &s_tb6612If;
}