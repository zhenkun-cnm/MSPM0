/**
 * @file    port_tb6612.h
 * @brief   Port layer TB6612 motor driver header
 * @note    4-layer decoupled architecture - Port layer
 *          Pins per syscfg/HAL:
 *            PWMA=PA12(TIMG0 CCP0), PWMB=PA13(TIMG0 CCP1)
 *            AIN1=PA22, AIN2=PB21, BIN1=PB23, BIN2=PA23
 *          PWM 10kHz (period=4000 @ 40MHz)
 */
#ifndef PORT_TB6612_H
#define PORT_TB6612_H

#include "dev_tb6612.h"

/* Single-channel helpers for menu-driven tests */
void PORT_TB6612_LeftOnly(void);
void PORT_TB6612_RightOnly(void);

/* Independent left/right duty cycle control (0-100%) */
void PORT_TB6612_SetLeftDuty(uint8_t pct);
void PORT_TB6612_SetRightDuty(uint8_t pct);

#endif /* PORT_TB6612_H */
