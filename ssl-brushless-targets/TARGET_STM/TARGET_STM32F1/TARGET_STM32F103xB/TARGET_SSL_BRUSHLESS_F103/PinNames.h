/* mbed Microcontroller Library
 * SPDX-License-Identifier: BSD-3-Clause
 ******************************************************************************
 *
 * Copyright (c) 2016-2021 STMicroelectronics.
 * All rights reserved.
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 *
 * Automatically generated from STM32CubeMX/db/mcu/STM32F103R(8-B)Tx.xml
 */

/* MBED TARGET LIST: SSL_BRUSHLESS_F103 */

#ifndef MBED_PINNAMES_H
#define MBED_PINNAMES_H

#include "PinNamesTypes.h"
#include "cmsis.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ALT0 = 0x100,
} ALTx;

typedef enum {
    PA_0 = 0x00,
    PA_1 = 0x01,
    PA_2 = 0x02,
    PA_3 = 0x03,
    PA_4 = 0x04,
    PA_5 = 0x05,
    PA_6 = 0x06,
    PA_7 = 0x07,
    PA_7_ALT0 = PA_7 | ALT0, // same pin used for alternate HW
    PA_8 = 0x08,
    PA_9 = 0x09,
    PA_10 = 0x0A,
    PA_11 = 0x0B,
    PA_12 = 0x0C,
    PA_13 = 0x0D,
    PA_14 = 0x0E,
    PA_15 = 0x0F,
    PB_0 = 0x10,
    PB_0_ALT0 = PB_0 | ALT0, // same pin used for alternate HW
    PB_1 = 0x11,
    PB_1_ALT0 = PB_1 | ALT0, // same pin used for alternate HW
    PB_2 = 0x12,
    PB_3 = 0x13,
    PB_4 = 0x14,
    PB_5 = 0x15,
    PB_6 = 0x16,
    PB_7 = 0x17,
    PB_8 = 0x18,
    PB_9 = 0x19,
    PB_10 = 0x1A,
    PB_11 = 0x1B,
    PB_12 = 0x1C,
    PB_13 = 0x1D,
    PB_14 = 0x1E,
    PB_15 = 0x1F,
    PC_0 = 0x20,
    PC_1 = 0x21,
    PC_2 = 0x22,
    PC_3 = 0x23,
    PC_4 = 0x24,
    PC_5 = 0x25,
    PC_6 = 0x26,
    PC_7 = 0x27,
    PC_8 = 0x28,
    PC_9 = 0x29,
    PC_10 = 0x2A,
    PC_11 = 0x2B,
    PC_12 = 0x2C,
    PC_13 = 0x2D,
    PC_14 = 0x2E,
    PC_15 = 0x2F,
    PD_0 = 0x30,
    PD_1 = 0x31,
    PD_2 = 0x32,

    /**** ADC internal channels ****/

    ADC_TEMP = 0xF0, // Internal pin virtual value
    ADC_VREF = 0xF1, // Internal pin virtual value

#ifdef TARGET_FF_ARDUINO_UNO
    // Arduino Uno (Rev3) pins
    ARDUINO_UNO_A0 = PA_0,
    ARDUINO_UNO_A1 = PA_1,
    ARDUINO_UNO_A2 = PA_4,
    ARDUINO_UNO_A3 = PB_0,
    ARDUINO_UNO_A4 = PC_1,
    ARDUINO_UNO_A5 = PC_0,

    ARDUINO_UNO_D0 = PA_3,
    ARDUINO_UNO_D1 = PA_2,
    ARDUINO_UNO_D2 = PA_10,
    ARDUINO_UNO_D3 = PB_3,
    ARDUINO_UNO_D4 = PB_5,
    ARDUINO_UNO_D5 = PB_4,
    ARDUINO_UNO_D6 = PB_10,
    ARDUINO_UNO_D7 = PA_8,
    ARDUINO_UNO_D8 = PA_9,
    ARDUINO_UNO_D9 = PC_7,
    ARDUINO_UNO_D10 = PB_6,
    ARDUINO_UNO_D11 = PA_7,
    ARDUINO_UNO_D12 = PA_6,
    ARDUINO_UNO_D13 = PA_5,
    ARDUINO_UNO_D14 = PB_9,
    ARDUINO_UNO_D15 = PB_8,
#endif

    // STDIO for console print
#ifdef MBED_CONF_TARGET_STDIO_UART_TX
    CONSOLE_TX = MBED_CONF_TARGET_STDIO_UART_TX,
#else
    CONSOLE_TX = PC_4, // NOT CONNECTED ON THE BOARD
#endif
#ifdef MBED_CONF_TARGET_STDIO_UART_RX
    CONSOLE_RX = MBED_CONF_TARGET_STDIO_UART_RX,
#else
    CONSOLE_RX = PC_5, // NOT CONNECTED ON THE BOARD
#endif

    /**** SSL Brushless L4A6 signal namings (see datasheet) ****/
    CAN1_RX = PB_8, // CAN1_RX, optional, R29 must be mount
    CAN1_TX = PB_9, // CAN1_TX, optional, R28 must be mount

    PWM_U = PA_8,
    PWM_V = PA_9,
    PWM_W = PA_10,

    EN_U = PC_6,
    EN_V = PC_7,
    EN_W = PC_8,

    ENC_MOSI = PB_15,
    ENC_MISO = PB_14,
    ENC_SCK = PB_13,
    ENC_CS = PB_12,
    ENC_CS_EXT = PB_14, // BE CAREFULL, SAME PIN AS AS5047D

    BAT_ADC = PB_0,

    DRV_CLK = PA_5,
    DRV_MOSI = PA_7,
    DRV_MISO = PA_6,
    DRV_CS = PA_4,
    //    DRV_BOOT    = PH_3, // PH_3 DO NOT EXIST ON F103 MCU
    DRV_RX = PA_3,
    DRV_TX = PA_2,

    HALL_U = PA_0, // WKUP
    HALL_V = PA_1,
    HALL_W = PB_10,

    I_U = PC_0,
    I_V = PC_3,
    I_W = PC_1,
    I_REF = PC_2,

    /**** USB pins ****/
    UDB_FS = PA_11,
    USB_DP = PA_12,

    /**** OSCILLATOR pins ****/
    RCC_OSC32_IN = PC_14,
    RCC_OSC32_OUT = PC_15,
    RCC_OSC_IN = PD_0,
    RCC_OSC_OUT = PD_1,

    /**** DEBUG pins ****/
    SYS_JTCK_SWCLK = PA_14,
    SYS_JTDI = PA_15,
    SYS_JTDO_SWO = PB_3,
    SYS_JTMS_SWDIO = PA_13,
    //    SYS_WKUP2 = PC_13, // NOT CONNECTED ON THE BOARD

    // Not connected
    NC = (int)0xFFFFFFFF
} PinName;

// Standardized LED and button names
#define LED1 PB_1
// #define BUTTON1  PH_3  // PH_3 DO NOT EXIST ON F103 MCU

#ifdef __cplusplus
}
#endif

#endif
