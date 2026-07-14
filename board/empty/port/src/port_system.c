/**
 * @file    port_system.c
 * @brief   Port 层系统硬件初始化实现
 * @note    封装 SYSCFG_DL_init()，将所有 TI DriverLib 寄存器初始化集中在此
 *          调度器启动前在 main() 中调用
 *          必须在 ICM-20948 初始化之前将 CS 拉高，确保 SPI 模式
 */

#include "port_system.h"
#include "ti_msp_dl_config.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

void PORT_SYSTEM_Init(void)
{
    /* TI DriverLib 默认外设初始化（时钟、GPIO、UART、SysTick） */
    SYSCFG_DL_init();

    /*
     * WIP hardware uses PB6/PB7/PB8/PB9 for Flash SPI, but these are also the
     * old IMU SPI pins. Keep them quiet until Flash/TFT explicitly initialize
     * SPI later, so ICM I2C probing is not affected by early SPI pinmux.
     */
    DL_SPI_disable(W25Q64_INST);
    DL_SPI_reset(W25Q64_INST);
    DL_GPIO_initDigitalOutput(CS_SPI1_CS_IOMUX);
    DL_GPIO_setPins(CS_PORT, CS_SPI1_CS_PIN);
    DL_GPIO_enableOutput(CS_PORT, CS_SPI1_CS_PIN);
    DL_GPIO_initDigitalInput(GPIO_W25Q64_IOMUX_POCI);
    DL_GPIO_initDigitalInput(GPIO_W25Q64_IOMUX_PICO);
    DL_GPIO_initDigitalInput(GPIO_W25Q64_IOMUX_SCLK);
    DL_GPIO_disableOutput(GPIOB, GPIO_W25Q64_POCI_PIN |
                                 GPIO_W25Q64_PICO_PIN |
                                 GPIO_W25Q64_SCLK_PIN);

    /* 提前点亮 TFT 背光 BLK(PB10)：
     * SysConfig 未显式配置 BLK 为 GPIO 输出时，PB10 可能处于高阻输入态。
     * 在此强制设为输出高电平，确保背光立即点亮，后续 tft_gpio_init() 可安全覆盖。 */
    DL_GPIO_initDigitalOutput(LCD_LCD_BLK_IOMUX);
    DL_GPIO_setPins(LCD_PORT, LCD_LCD_BLK_PIN);
    DL_GPIO_enableOutput(LCD_PORT, LCD_LCD_BLK_PIN);
}
