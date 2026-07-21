/**
 * @file    port_led.c
 * @brief   Port 层 LED 驱动实现
 * @note    实现 dev_led.h 的 DevLED OOP 契约
 *          调用 BSP 层 DL_GPIO_* API 操作 PB22
 */

#include "port_led.h"
#include "ti_msp_dl_config.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

/* LED 硬件引脚定义 */

/* Device 层接口实例 */
static DevLED s_ledIf;

/* ========== 静态函数实现 ========== */

static void led_init(DevLED *self)
{
    (void)self;
    DL_GPIO_clearPins(LED_PORT, LED_PIN_22_PIN);
}

static void led_on(DevLED *self)
{
    (void)self;
    DL_GPIO_setPins(LED_PORT, LED_PIN_22_PIN);
}

static void led_off(DevLED *self)
{
    (void)self;
    DL_GPIO_clearPins(LED_PORT, LED_PIN_22_PIN);
}

static void led_toggle(DevLED *self)
{
    (void)self;
    DL_GPIO_togglePins(LED_PORT, LED_PIN_22_PIN);
}

static DevLED_State_t led_getState(DevLED *self)
{
    (void)self;
    return (DL_GPIO_readPins(LED_PORT, LED_PIN_22_PIN) == 0) ?
           LED_STATE_OFF : LED_STATE_ON;
}

/* ========== 硬件初始化（调度器启动前调用） ========== */

void PORT_LED_InitHW(void)
{
    /* LED 引脚已在 SYSCFG_DL_GPIO_init() 中完成初始化 */
}

/* ========== 注册 Device 接口 ========== */

DevLED* GetLED(void)
{
    s_ledIf.init      = led_init;
    s_ledIf.on        = led_on;
    s_ledIf.off       = led_off;
    s_ledIf.toggle    = led_toggle;
    s_ledIf.getState  = led_getState;

    return &s_ledIf;
}
