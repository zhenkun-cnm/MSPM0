/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define GPIO_HFXT_PORT                                                     GPIOA
#define GPIO_HFXIN_PIN                                             DL_GPIO_PIN_5
#define GPIO_HFXIN_IOMUX                                         (IOMUX_PINCM10)
#define GPIO_HFXOUT_PIN                                            DL_GPIO_PIN_6
#define GPIO_HFXOUT_IOMUX                                        (IOMUX_PINCM11)
#define CPUCLK_FREQ                                                     80000000
/* Defines for SYSPLL_ERR_01 Workaround */
/* Represent 1.000 as 1000 */
#define FLOAT_TO_INT_SCALE                                               (1000U)
#define FCC_EXPECTED_RATIO                                                  1000
#define FCC_UPPER_BOUND                       (FCC_EXPECTED_RATIO * (1 + 0.003))
#define FCC_LOWER_BOUND                       (FCC_EXPECTED_RATIO * (1 - 0.003))

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);


/* Defines for sys_uart */
#define sys_uart_INST                                                      UART0
#define sys_uart_INST_FREQUENCY                                         40000000
#define sys_uart_INST_IRQHandler                                UART0_IRQHandler
#define sys_uart_INST_INT_IRQN                                    UART0_INT_IRQn
#define GPIO_sys_uart_RX_PORT                                              GPIOA
#define GPIO_sys_uart_TX_PORT                                              GPIOA
#define GPIO_sys_uart_RX_PIN                                      DL_GPIO_PIN_11
#define GPIO_sys_uart_TX_PIN                                      DL_GPIO_PIN_10
#define GPIO_sys_uart_IOMUX_RX                                   (IOMUX_PINCM22)
#define GPIO_sys_uart_IOMUX_TX                                   (IOMUX_PINCM21)
#define GPIO_sys_uart_IOMUX_RX_FUNC                    IOMUX_PINCM22_PF_UART0_RX
#define GPIO_sys_uart_IOMUX_TX_FUNC                    IOMUX_PINCM21_PF_UART0_TX
#define sys_uart_BAUD_RATE                                              (115200)
#define sys_uart_IBRD_40_MHZ_115200_BAUD                                    (21)
#define sys_uart_FBRD_40_MHZ_115200_BAUD                                    (45)




/* Defines for W25Q64 */
#define W25Q64_INST                                                        SPI1
#define W25Q64_INST_IRQHandler                                  SPI1_IRQHandler
#define W25Q64_INST_INT_IRQN                                      SPI1_INT_IRQn
#define GPIO_W25Q64_PICO_PORT                                             GPIOA
#define GPIO_W25Q64_PICO_PIN                                     DL_GPIO_PIN_18
#define GPIO_W25Q64_IOMUX_PICO                                  (IOMUX_PINCM40)
#define GPIO_W25Q64_IOMUX_PICO_FUNC                  IOMUX_PINCM40_PF_SPI1_PICO
#define GPIO_W25Q64_POCI_PORT                                             GPIOA
#define GPIO_W25Q64_POCI_PIN                                     DL_GPIO_PIN_16
#define GPIO_W25Q64_IOMUX_POCI                                  (IOMUX_PINCM38)
#define GPIO_W25Q64_IOMUX_POCI_FUNC                  IOMUX_PINCM38_PF_SPI1_POCI
/* GPIO configuration for W25Q64 */
#define GPIO_W25Q64_SCLK_PORT                                             GPIOA
#define GPIO_W25Q64_SCLK_PIN                                     DL_GPIO_PIN_17
#define GPIO_W25Q64_IOMUX_SCLK                                  (IOMUX_PINCM39)
#define GPIO_W25Q64_IOMUX_SCLK_FUNC                  IOMUX_PINCM39_PF_SPI1_SCLK



/* Port definition for Pin Group LED */
#define LED_PORT                                                         (GPIOB)

/* Defines for PIN_22: GPIOB.22 with pinCMx 50 on package pin 21 */
#define LED_PIN_22_PIN                                          (DL_GPIO_PIN_22)
#define LED_PIN_22_IOMUX                                         (IOMUX_PINCM50)
/* Port definition for Pin Group CS */
#define CS_PORT                                                          (GPIOA)

/* Defines for SPI1_CS: GPIOA.15 with pinCMx 37 on package pin 8 */
#define CS_SPI1_CS_PIN                                          (DL_GPIO_PIN_15)
#define CS_SPI1_CS_IOMUX                                         (IOMUX_PINCM37)
/* Port definition for Pin Group ENCODER */
#define ENCODER_PORT                                                     (GPIOA)

/* Defines for KEY: GPIOA.26 with pinCMx 59 on package pin 30 */
#define ENCODER_KEY_PIN                                         (DL_GPIO_PIN_26)
#define ENCODER_KEY_IOMUX                                        (IOMUX_PINCM59)
/* Defines for encodera: GPIOA.24 with pinCMx 54 on package pin 25 */
#define ENCODER_encodera_PIN                                    (DL_GPIO_PIN_24)
#define ENCODER_encodera_IOMUX                                   (IOMUX_PINCM54)
/* Defines for encoderb: GPIOA.25 with pinCMx 55 on package pin 26 */
#define ENCODER_encoderb_PIN                                    (DL_GPIO_PIN_25)
#define ENCODER_encoderb_IOMUX                                   (IOMUX_PINCM55)
/* Port definition for Pin Group ICM */
#define ICM_PORT                                                         (GPIOB)

/* Defines for ICM_MISO: GPIOB.7 with pinCMx 24 on package pin 59 */
#define ICM_ICM_MISO_PIN                                         (DL_GPIO_PIN_7)
#define ICM_ICM_MISO_IOMUX                                       (IOMUX_PINCM24)
/* Defines for ICM_MOSI: GPIOB.8 with pinCMx 25 on package pin 60 */
#define ICM_ICM_MOSI_PIN                                         (DL_GPIO_PIN_8)
#define ICM_ICM_MOSI_IOMUX                                       (IOMUX_PINCM25)
/* Defines for ICM_CS: GPIOB.6 with pinCMx 23 on package pin 58 */
#define ICM_ICM_CS_PIN                                           (DL_GPIO_PIN_6)
#define ICM_ICM_CS_IOMUX                                         (IOMUX_PINCM23)
/* Defines for ICM_SCK: GPIOB.9 with pinCMx 26 on package pin 61 */
#define ICM_ICM_SCK_PIN                                          (DL_GPIO_PIN_9)
#define ICM_ICM_SCK_IOMUX                                        (IOMUX_PINCM26)
/* Port definition for Pin Group LCD */
#define LCD_PORT                                                         (GPIOB)

/* Defines for LCD_RES: GPIOB.13 with pinCMx 30 on package pin 1 */
#define LCD_LCD_RES_PIN                                         (DL_GPIO_PIN_13)
#define LCD_LCD_RES_IOMUX                                        (IOMUX_PINCM30)
/* Defines for LCD_DC: GPIOB.12 with pinCMx 29 on package pin 64 */
#define LCD_LCD_DC_PIN                                          (DL_GPIO_PIN_12)
#define LCD_LCD_DC_IOMUX                                         (IOMUX_PINCM29)
/* Defines for LCD_CS: GPIOB.11 with pinCMx 28 on package pin 63 */
#define LCD_LCD_CS_PIN                                          (DL_GPIO_PIN_11)
#define LCD_LCD_CS_IOMUX                                         (IOMUX_PINCM28)
/* Defines for LCD_BLK: GPIOB.10 with pinCMx 27 on package pin 62 */
#define LCD_LCD_BLK_PIN                                         (DL_GPIO_PIN_10)
#define LCD_LCD_BLK_IOMUX                                        (IOMUX_PINCM27)




/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);
void SYSCFG_DL_sys_uart_init(void);
void SYSCFG_DL_W25Q64_init(void);

void SYSCFG_DL_SYSTICK_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
